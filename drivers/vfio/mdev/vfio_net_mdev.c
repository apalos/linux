#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/mm_types.h>
#include <linux/vfio.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>
#include <uapi/linux/net_mdev.h>

#define DRIVER_VERSION  "0.1"
#define DRIVER_AUTHOR   "Linaro"
#define DRIVER_DESC     "VFIO based driver for mediated network device"

/* private structures */
struct netmdev_driver {
	struct device_driver *driver;
	struct netmdev_driver_ops *drv_ops;
};

struct iovamap {
	u64 iova;
	void *vaddr;
	struct device *dev;
	u32 size; /* maximum of 32MB */
	enum dma_data_direction direction;
};

struct netmdev {
	union {
		/* kernel visibility only, not part of UAPI*/
		char private[4096 * 2];
		struct {
			struct net_device *netdev;
			struct netmdev_driver_ops drv_ops;
			/* FIXME USE A LINKED LIST */
			int mappings_count;
			struct iovamap mappings[128];
		};
	};
	union {
		/* shadow features & statistics page */
		/* part of UAPI */
		char uapioffset[4096];
		struct netmdev_uapi uapi;
	};
} ;

/* globals */
struct netmdev_driver netmdev_known_drivers[256];
int netmdev_known_drivers_count = 0;
/* foward definitions & helper functions */
static struct netmdev_driver_ops *netmdev_get_driver_ops(struct device_driver *driver);

static struct net_device *get_netdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev;

	netmdev = mdev_get_drvdata(mdev);
	if (!netmdev)
		return NULL;

	return netmdev->netdev;
}

/* SYSFS structure for the mdev_parent device */
static ssize_t available_instances_show(struct kobject *kobj, struct device *dev,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", 1);
}
static MDEV_TYPE_ATTR_RO(available_instances);

static ssize_t device_api_show(struct kobject *kobj, struct device *dev,
			       char *buf)
{
	return sprintf(buf, "%s\n", VFIO_DEVICE_API_PCI_STRING);
}
static MDEV_TYPE_ATTR_RO(device_api);

static struct attribute *sysfs_vfnetdev_attributes[] = {
	&mdev_type_attr_device_api.attr,
	&mdev_type_attr_available_instances.attr,
	NULL,
};

static struct attribute_group sysfs_vfnetdev_type = {
	.name = "netmdev",
	.attrs = sysfs_vfnetdev_attributes,
};

/* Only 1 supported for now */
static struct attribute_group *sysfs_type_list[] = {
	&sysfs_vfnetdev_type,
	NULL
};

/* SYSFS structure for created mdevices */
static ssize_t netdev_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mdev_device *mdev;
	struct net_device *netdev;

	mdev = mdev_from_dev(dev);
	if (!mdev)
		return scnprintf(buf, PAGE_SIZE, "mdev not found\n");

	netdev = get_netdev(mdev);
	if (!netdev)
		return scnprintf(buf, PAGE_SIZE, "ndev-mdev not found\n");

	return scnprintf(buf, PAGE_SIZE, "%.16s\n", netdev->name);
}

static ssize_t netdev_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct mdev_device *mdev;
	struct net_device *port;
	struct netmdev *netmdev;
	char name[IFNAMSIZ + 1];
	struct netmdev_driver_ops *ops;

	if (count < 2)
		return -1;

	mdev = mdev_from_dev(dev);
	if (!mdev)
		return -ENODEV;

	netmdev = mdev_get_drvdata(mdev);
	if (netmdev)
		return -EINVAL;

	netmdev = kzalloc(sizeof(struct netmdev), GFP_KERNEL);
	if (!netmdev)
		return -ENOMEM;
	mdev_set_drvdata(mdev, netmdev);

	if (count > IFNAMSIZ)
		return -EINVAL;

	memset(name, 0, sizeof(name));
	scnprintf(name, IFNAMSIZ + 1, "%.*s", (int)count - 1, buf);
	port = dev_get_by_name(&init_net, name);
	if (!port)
		return -ENODEV;

	/* FIXME find a way to check if this is the parent device */
	//if (&port->dev != mdev_parent_dev(mdev)) return -1;
	ops = netmdev_get_driver_ops(mdev_parent_dev(mdev)->driver);
	if (ops) {
		memcpy(&netmdev->drv_ops, ops, sizeof(netmdev->drv_ops));
	} else {
		dev_put(port);
		return -1;
	}
	netmdev->netdev = port;

	return count;
}
static DEVICE_ATTR_RW(netdev);

static struct attribute *sysfs_mdev_vfnetdev_attributes[] = {
	&dev_attr_netdev.attr,
	NULL,
};

static struct attribute_group sysfs_mdev_vfnetdev_group = {
	.name = "netmdev",
	.attrs = sysfs_mdev_vfnetdev_attributes,
};

static const struct attribute_group *sysfs_mdev_groups[] = {
	&sysfs_mdev_vfnetdev_group,
	NULL,
};

static int netmdev_sysfs_create(struct kobject *kobj, struct mdev_device *mdev)
{
	return 0;
}

static int netmdev_sysfs_remove(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *port;

	if (netmdev) {
		port = get_netdev(mdev);
		if (port)
			dev_put(port);
		kfree(netmdev);
		mdev_set_drvdata(mdev, NULL);
	}

	return 0;
}

/* NETMDEV file operations
userland obtains fd through IOCTL on the VFIO container: this calls netmdev_dev_open.
closing the FD or terminating the userland process in anyway will result in netmdev_dev_release
*/
static int netmdev_dev_open(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *port;

	port = get_netdev(mdev);
	if (!port)
		return -ENODEV;
	netif_tx_stop_all_queues(port);
	port->priv_flags |= IFF_VFNETDEV;
	netmdev->drv_ops.transition_start(port);

	/* FIXME : uggly and dangerous, let's find a clean way to shadow the values*/
	memcpy(&netmdev->uapi, &port->features, sizeof(struct netmdev_uapi));

	return 0;
}

static void netmdev_dev_release(struct mdev_device *mdev)
{
	struct net_device *port;
	int i;
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	/* TODO export shadow stats to net_device */
	if (!netmdev)
		return;

	port = get_netdev(mdev);
	if (!port)
		return;

	port->priv_flags &= ~IFF_VFNETDEV;
	netmdev->drv_ops.transition_back(port);

	while (netmdev->mappings_count > 0) {
		i = --netmdev->mappings_count;
		dma_unmap_single(netmdev->mappings[i].dev, netmdev->mappings[i].iova,
				 netmdev->mappings[i].size,
				 netmdev->mappings[i].direction);
		kfree(netmdev->mappings[i].vaddr);
	}

	netif_tx_start_all_queues(port);

	return;
}

static long netmdev_dev_ioctl(struct mdev_device *mdev, unsigned int cmd,
			      unsigned long arg)
{
	unsigned long minsz;
	struct net_device *netdev;
	struct netmdev *netmdev;
	struct vfio_device_info device_info;
	struct vfio_region_info region_info;
	struct vfio_irq_info irq_info;
	int ret;

	if (!mdev)
		return -EINVAL;

	netdev = get_netdev(mdev);
	netmdev = mdev_get_drvdata(mdev);

	if (!netdev)
		return -ENODEV;

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
		minsz = offsetofend(struct vfio_device_info, num_irqs);

		if (copy_from_user(&device_info, (void __user *)arg, minsz))
			return -EFAULT;

		if (device_info.argsz < minsz)
			return -EINVAL;

		/* shadow page + rx_ring and tx_ring*/
		ret = netmdev->drv_ops.get_device_info(netdev, &device_info);

		return copy_to_user((void __user *)arg, &device_info, minsz) ?
			-EFAULT : 0;
	case VFIO_DEVICE_GET_REGION_INFO:
		minsz = offsetofend(struct vfio_region_info, offset);

		if (copy_from_user(&region_info, (void __user *)arg, minsz))
			return -EFAULT;

		if (region_info.argsz < minsz)
			return -EINVAL;

		if (region_info.index == VFIO_PCI_NUM_REGIONS + 1) {
			/* do not give access to first page */
			region_info.offset = (u64)netmdev + PAGE_SIZE;
			region_info.size = sizeof(struct netmdev) - PAGE_SIZE;
			ret = 0;
		} else {
			ret = netmdev->drv_ops.get_region_info(netdev, &region_info);
		}

		if (ret < 0)
			return ret;

		return copy_to_user((void __user *)arg, &region_info, minsz) ?
			-EFAULT : 0;
	case VFIO_DEVICE_GET_IRQ_INFO:
		minsz = offsetofend(struct vfio_irq_info, count);

		if (copy_from_user(&irq_info, (void __user *)arg, minsz))
			return -EFAULT;

		if (irq_info.argsz < minsz || irq_info.index != 0)
			return -EINVAL;

		irq_info.count = 1;
		irq_info.flags = VFIO_IRQ_INFO_EVENTFD |
			VFIO_IRQ_INFO_MASKABLE |
			VFIO_IRQ_INFO_AUTOMASKED;
		return copy_to_user((void __user *)arg, &irq_info, minsz) ?
			-EFAULT : 0;
	case VFIO_IOMMU_MAP_DMA: {
		struct vfio_iommu_type1_dma_map map;
		struct vm_area_struct *vma;
		void *data;
		struct device *parent_dev;
		int node;
		dma_addr_t mapping;
		int ret = -EINVAL;

		/* allocate DMA area and map it where the userland asks
		 * userland need to mmap an area WITHOUT allocating pages:
		 * mmap(vaddr,size, PROT_READ | PROT_WRITE, MAP_SHARED |
		 * MAP_ANONYMOUS | MAP_NORESERVE | MAP_FIXED, -1, 0
		 * MAP_NORESERVE ensures only VA space is booked, no pages are
		 * mapped * the mapping must be the entire area, not partial on
		 * the vma
		 */

		if (netmdev->mappings_count >= 128)
			return -EFAULT;

		minsz = offsetofend(struct vfio_iommu_type1_dma_map, size);

		if (copy_from_user(&map, (void __user *)arg, minsz)) {
			ret = -EFAULT;
			goto out;
		}

		if (map.argsz < minsz)
			goto out;

		/*
		 * locates the containing vma for the required map.vaddr
		 * the vma must point to the entire zone allocated by mmap in
		 * userland
		 */
		vma = find_vma(current->mm, map.vaddr);
		if (!vma)
			return -EFAULT;
		if (map.vaddr >= vma->vm_end)
			return -EFAULT;
		/* the iova will be returned as part of the ioctl to the userland */
		parent_dev = mdev_parent_dev(mdev);

		node = netdev->dev.parent ? dev_to_node(netdev->dev.parent) : -1;

		data = kmalloc_node(map.size, GFP_KERNEL, node);
		if (!data)
			/* return ret? */
			return -ENOMEM;

		mapping = dma_map_single(parent_dev, data, map.size,
					 DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(parent_dev, mapping))) {
			if (net_ratelimit())
				dev_err(parent_dev,
					"Failed to dma_map_single buffer for userland!\n");
			kfree(data);
			goto out;
		}
		map.iova = mapping;
		ret = io_remap_pfn_range(vma, map.vaddr,
					 virt_to_phys(data) >> PAGE_SHIFT,
					 map.size, vma->vm_page_prot);
		printk(KERN_INFO"VFIO_IOMMU_MAP_DMA: io_remap_pfn_range %llx -> physmem <- @%llx, %lld:%d\n",
		       map.vaddr, map.iova, map.size, ret);

		if (ret) {
			dma_unmap_single(parent_dev, mapping, map.size, DMA_BIDIRECTIONAL);
			kfree(data);
			printk(KERN_ERR"VFIO_IOMMU_MAP_DMA: io_remap_pfn_range failed\n");
			return -EFAULT;
		}

		netmdev->mappings[netmdev->mappings_count].dev = parent_dev;
		netmdev->mappings[netmdev->mappings_count].vaddr = data;
		netmdev->mappings[netmdev->mappings_count].iova = mapping;
		netmdev->mappings[netmdev->mappings_count].size = map.size;
		netmdev->mappings[netmdev->mappings_count].direction = DMA_BIDIRECTIONAL;
		netmdev->mappings_count++;

		return copy_to_user((void __user *)arg, &map, minsz) ?
			-EFAULT : 0;
out:
		return ret;
	}
	case VFIO_NETMDEV_TRANSITION_COMPLETE:
		netmdev->drv_ops.transition_complete(netdev);
		return 0;
	case VFIO_DEVICE_RESET:
		/* FIXME add callback */
		return 0;
        default:
                return -EOPNOTSUPP;
	}

	return -EINVAL;
}

static int netmdev_dev_mmap(struct mdev_device *mdev, struct vm_area_struct *vma)
{
	struct net_device *netdev;
	struct netmdev *netmdev;
	int ret = 0;
	u64 phys_len, req_len, pgoff, req_start;
	unsigned int index;
	unsigned long phys_pfn;

	/* userland wants to access ring descrptors that was pre-allocated
	 * by the kernel
	 * note: userland need to user IOCTL MAP to CREATE packet buffers
	 */
	netdev = get_netdev(mdev);
	netmdev = mdev_get_drvdata(mdev);

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;
	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;

	index = vma->vm_pgoff >> (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT);
	switch (index) {
	/* e1000e and r8169 for now only need BAR0/BAR2 respectively */
	case VFIO_PCI_BAR0_REGION_INDEX ... VFIO_PCI_BAR5_REGION_INDEX:
	/* Rx / Tx descriptors */
	case VFIO_PCI_NUM_REGIONS + 2:
	case VFIO_PCI_NUM_REGIONS + 3:
		ret = netmdev->drv_ops.get_mmap_info(vma, netdev, &phys_pfn,
						     &phys_len);
		if (ret)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}


	req_len = vma->vm_end - vma->vm_start;
	pgoff = vma->vm_pgoff &
		((1U << (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT)) - 1);
	req_start = pgoff << PAGE_SHIFT;

	if (req_start + req_len > phys_len)
			return -EINVAL;

	vma->vm_private_data = NULL;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_pgoff = phys_pfn + pgoff;

	return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       req_len, vma->vm_page_prot);
}

/* FIXME migrate the mmaping to sparse capabilities model. Use this as a guide */
#if 0
static int netmdev_dev_mmap(struct mdev_device *mdev, struct vm_area_struct *vma)
{
	struct net_device *netdev;
	struct netmdev *netmdev;
	struct vfio_region_info info;
	int ret = 0;
	int i;
	u64 req_len;

	/* userland wants to access pre-allocated kernel ring descrptors
	 * note: userland need to user IOCTL MAP to CREATE packet buffers
	 */
	netdev = get_netdev(mdev);
	netmdev = mdev_get_drvdata(mdev);

	/* FIXME ensure the size of the map is also correct */
	req_len = PAGE_ALIGN(vma->vm_end - vma->vm_start);

	/* check that we try to map only authorized areas */
	/* FIXME how to get the number of regions ? */
	/* FIXME deal with PCI config space and non PCI devices later */
	for (i = VFIO_PCI_NUM_REGIONS + 1; i < VFIO_PCI_NUM_REGIONS + 5; i++) {
		info.argsz = sizeof(info);
		info.index = i;

		if (info.index == VFIO_PCI_NUM_REGIONS + 1) {
			/* FIXME: look for a better way to get ALL regions */
			info.offset = (u64)netmdev + PAGE_SIZE;
			info.size = sizeof(struct netmdev) - PAGE_SIZE; /* do not give access to first page */
			ret = 0;
		} else {
			ret = netmdev->drv_ops.get_region_info(netdev, &info);
		}

		if (ret != 0)
			return -EINVAL;
		printk(KERN_INFO"%llx==%llx %llx==%llx\n", info.offset >> PAGE_SHIFT, (u64)vma->vm_pgoff, info.size, (u64)( vma->vm_end - vma->vm_start));

		if ((info.offset >> PAGE_SHIFT) == vma->vm_pgoff && info.size == (vma->vm_end - vma->vm_start))
			goto do_the_map;
	}
	return -EINVAL;

do_the_map:

	vma->vm_private_data = NULL;
	/* FIXME this should be uncached memory but it sounds the driver does not map in non cached. strange...*/
	//vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = remap_pfn_range(vma, vma->vm_start, virt_to_phys((void*)(vma->vm_pgoff << PAGE_SHIFT)) >> PAGE_SHIFT, req_len, vma->vm_page_prot);

	printk(KERN_INFO"vfnetdev_map %lx-%llx -> @%llx-%llx : %d\n",
		vma->vm_start, vma->vm_start + req_len,
		virt_to_phys((void*)(vma->vm_pgoff << PAGE_SHIFT)), virt_to_phys((void*)(vma->vm_pgoff << PAGE_SHIFT)) + req_len,
		ret
	);

	return ret;
}
#endif

static const struct mdev_parent_ops netmdev_sysfs_ops = {
	.supported_type_groups = sysfs_type_list,
	.mdev_attr_groups = sysfs_mdev_groups,
	.create = netmdev_sysfs_create,
	.remove = netmdev_sysfs_remove,

	.open = netmdev_dev_open,
	.release = netmdev_dev_release,
	.read = NULL,
	.write = NULL,
	.mmap = netmdev_dev_mmap,
	.ioctl = netmdev_dev_ioctl,
};

/* We need all callbacks to be available */
static int netmdev_check_cbacks(struct netmdev_driver_ops *drv_ops)
{
	return (!drv_ops || !drv_ops->transition_start ||
		!drv_ops->transition_complete || !drv_ops->get_region_info ||
		!drv_ops->transition_back);
}

/* netmdev_driver stuff */
static int netmdev_register_driver(struct device_driver *driver,
				   struct netmdev_driver_ops *drv_ops)
{
	if (netmdev_check_cbacks(drv_ops))
		return -EINVAL;

	netmdev_known_drivers[netmdev_known_drivers_count].driver = driver;
	netmdev_known_drivers[netmdev_known_drivers_count].drv_ops= drv_ops;
	netmdev_known_drivers_count++;

	return 0;
}

static struct netmdev_driver_ops *netmdev_get_driver_ops(struct device_driver *driver)
{
	int i;

	printk(KERN_INFO"netmdev_get_driver_ops(%p)\n", driver);
	for (i = 0; i < netmdev_known_drivers_count; i++)
	{
		if (netmdev_known_drivers[i].driver == driver) {
			printk(KERN_INFO"found driver(%p, %p)\n", driver,
			       netmdev_known_drivers[i].driver);
			return	netmdev_known_drivers[i].drv_ops;
		}
	}
	printk(KERN_ERR"netmdev_get_driver_ops could not find driver\n");
	return NULL;
}

int netmdev_unregister_device(struct device *dev)
{
	mdev_unregister_device(dev);
	return 0;
}
EXPORT_SYMBOL(netmdev_unregister_device);

int netmdev_register_device(struct device *dev, struct netmdev_driver_ops *ops)
{
	int ret;
	ret = mdev_register_device(dev, &netmdev_sysfs_ops);
	if (ret < 0)
		return ret;
	return netmdev_register_driver(dev->driver, ops);
}
EXPORT_SYMBOL(netmdev_register_device);

static int __init netmdev_init(void)
{
	memset(netmdev_known_drivers, 0, sizeof(netmdev_known_drivers));
	return 0;
}

static void __exit netmdev_exit(void)
{
}

module_init(netmdev_init)
module_exit(netmdev_exit)

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
