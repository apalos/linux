#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/mm_types.h>
#include <linux/kernel.h>
#include <linux/vfio.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>
#include <uapi/linux/net_mdev.h>
#include "mdev_private.h"

#ifdef CONFIG_X86
#include <asm/set_memory.h>
#include <asm/pat.h>
#endif
#include <linux/mm.h>
#include "mdev_net_private.h"

#define DRIVER_VERSION  "0.1"
#define DRIVER_AUTHOR   "Linaro"
#define DRIVER_DESC     "VFIO based driver for mediated network device"

#define MDEV_WC
/* globals */
struct netmdev_driver netmdev_known_drivers[256];
int netmdev_known_drivers_count = 0;
/* foward definitions & helper functions */
static struct netmdev_driver_ops *netmdev_get_driver_ops(struct device_driver *driver);

struct net_device *mdev_get_netdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev;

	netmdev = mdev_get_drvdata(mdev);
	if (!netmdev)
		return NULL;

	return netmdev->netdev;
}
EXPORT_SYMBOL(mdev_get_netdev);

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

	netdev = mdev_get_netdev(mdev);
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

	INIT_LIST_HEAD(&netmdev->mapping_list_head);
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
		return -EINVAL;
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
		port = mdev_get_netdev(mdev);
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

	port = mdev_get_netdev(mdev);
	if (!port)
		return -ENODEV;

	netif_tx_stop_all_queues(port);

	port->priv_flags |= IFF_VFNETDEV;
	if (netmdev->drv_ops.transition_start(mdev))
		return -EINVAL;

	return 0;
}

static void netmdev_dev_release(struct mdev_device *mdev)
{
	struct net_device *port;
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct iovamap *mapping, *n;

	/* TODO export shadow stats to net_device */
	if (!netmdev)
		return;

	port = mdev_get_netdev(mdev);
	if (!port)
		return;

	port->priv_flags &= ~IFF_VFNETDEV;
	netmdev->drv_ops.transition_back(mdev);

	list_for_each_entry_safe(mapping, n, &netmdev->mapping_list_head,
				 list) {
		dev_warn(&mdev->dev, "stale mapping %d @ 0x%p -> 0x@%llx\n",
		       mapping->size, mapping->cookie, mapping->iova);
		dma_free_attrs(mapping->dev, mapping->size,
			       mapping->cookie, mapping->iova,
			       DMA_ATTR_NO_KERNEL_MAPPING |
			       DMA_ATTR_WRITE_COMBINE);
		list_del(&mapping->list);
	}

	netif_tx_start_all_queues(port);

	return;
}

/*
 * Allocate DMA area and map it where the userland asks.
 * Userland need to mmap an area WITHOUT allocating pages:
 * mmap(vaddr,size, PROT_READ | PROT_WRITE, MAP_SHARED |
 * MAP_ANONYMOUS | MAP_NORESERVE | MAP_FIXED, -1, 0
 * MAP_NORESERVE ensures only VA space is booked, no pages are mapped.
 * The mapping must be the entire area, not partial on the vma.
 */
static int netmdev_vfio_mmap_dma(struct mdev_device *mdev,
				 struct vfio_iommu_type1_dma_map *param)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	enum dma_data_direction direction;
	struct vm_area_struct *vma;
	struct iovamap *mapping;
#ifdef MDEV_WC
	int ret;
#endif

	if (param->flags & ~(VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE))
		return -EINVAL;

	if ((param->size & ~PAGE_MASK) || (param->vaddr & ~PAGE_MASK))
		return -EINVAL;

	/*
	 * locates the containing vma for the required map.vaddr.
	 * The vma must point to the entire zone allocated by mmap in
	 * userland.
	 */
	vma = find_vma(current->mm, param->vaddr);
	if (!vma)
		return -EFAULT;

	if (param->vaddr + param->size > vma->vm_end)
		return -EINVAL;

	if (param->flags & VFIO_DMA_MAP_FLAG_READ) {
		if (param->flags & VFIO_DMA_MAP_FLAG_WRITE)
			direction = DMA_BIDIRECTIONAL;
		else
			direction = DMA_TO_DEVICE;
	} else {
		if (param->flags & VFIO_DMA_MAP_FLAG_WRITE)
			direction = DMA_FROM_DEVICE;
		else
			return -EINVAL;
	}

	/* Allocate new netmdev mapping */
	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return -ENOMEM;

	INIT_LIST_HEAD(&mapping->list);
	mapping->dev = mdev_parent_dev(mdev);
	mapping->size = param->size;
	mapping->direction = direction;


	/*
	 * TODO: allocate close to NUMA node like ixgbe does
	 *   set_dev_node(mapping->dev, dma_node);
	 *   dma_alloc_coherent();
	 *   set_dev_node(dev, orig_node);
	 */
	/*
	 * TODO: Not all platforms support cache coherent DMA.
	 * dma_alloc_coherent() will fallback to allocate non-cacheable
	 * memory which won't provide acceptable packet IO performance.
	 * We shall implement non-coherent allocation and cache sync API
	 * to invalidate cache in RX path and flush cache in TX path.
	 * Possible solution is to use DMA_ATTR_NON_CONSISTENT with
	 * dma_alloc_attrs() and let the application request cache
	 * synchronization via a syscall.
	 */
	/*
	 * TODO: add dev_hold() and dev_put() to make sure we don't exit
	 * before all mappings are removed from userspace.
	 */
	mapping->cookie =
	    dma_alloc_attrs(mapping->dev, mapping->size, &mapping->iova,
			    GFP_KERNEL,
#ifndef MDEV_WC
			    DMA_ATTR_NO_KERNEL_MAPPING |
#endif
			    DMA_ATTR_WRITE_COMBINE);
	if (!mapping->cookie) {
		kfree(mapping);
		return -EFAULT;
	}

#ifdef MDEV_WC
	ret = set_memory_wc((unsigned long)(mapping->cookie),
			    mapping->size >> PAGE_SHIFT);
	if (!ret)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (io_remap_pfn_range(vma, vma->vm_start,
			    page_to_pfn(virt_to_page(mapping->cookie)),
			    mapping->size, vma->vm_page_prot) < 0) {
		set_memory_wb((unsigned long)mapping->cookie,
			      mapping->size >> PAGE_SHIFT);
		dma_free_attrs(mapping->dev, mapping->size,
			       mapping->cookie, mapping->iova,
			       DMA_ATTR_WRITE_COMBINE);
		kfree(mapping);
		return -EFAULT;
	}
#else
	if (dma_mmap_attrs
	    (mapping->dev, vma, mapping->cookie, mapping->iova,
	     mapping->size,
	     DMA_ATTR_NO_KERNEL_MAPPING | DMA_ATTR_WRITE_COMBINE) < 0) {
		dma_free_attrs(mapping->dev, mapping->size,
			       mapping->cookie, mapping->iova,
			       DMA_ATTR_NO_KERNEL_MAPPING |
			       DMA_ATTR_WRITE_COMBINE);
		kfree(mapping);
		return -EFAULT;
	}
#endif
	/* Pass the IOVA to userspace */
	param->iova = mapping->iova;

	dev_dbg(&mdev->dev, "new iommu dma mapping %d @ 0x%p -> 0x@%llx\n",
		 mapping->size, mapping->cookie, mapping->iova);

	list_add_tail(&mapping->list, &netmdev->mapping_list_head);

	return 0;
}

static int netmdev_vfio_unmmap_dma(struct mdev_device *mdev,
				   struct vfio_iommu_type1_dma_unmap
				   *param)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct iovamap *mapping, *n;

	list_for_each_entry_safe(mapping, n, &netmdev->mapping_list_head,
				 list) {
		if (param->iova == mapping->iova
		    && param->size == mapping->size) {
			dev_dbg(&mdev->dev,
				"remove iommu dma mapping %d @ 0x%p -> 0x@%llx\n",
				mapping->size, mapping->cookie, mapping->iova);
#ifdef MDEV_WC
			set_memory_wb((unsigned long)mapping->cookie,
				      mapping->size >> PAGE_SHIFT);
#endif
			dma_free_attrs(mapping->dev, mapping->size,
				       mapping->cookie, mapping->iova,
				       DMA_ATTR_NO_KERNEL_MAPPING |
				       DMA_ATTR_WRITE_COMBINE);
			list_del(&mapping->list);
			return 0;
		}
	}

	return -EINVAL;
}

static struct
vfio_region_info_cap_sparse_mmap *netmdev_fill_sparse(struct mdev_net_region *region)
{
	struct vfio_region_info_cap_sparse_mmap *sparse;
	int i;

	sparse = kzalloc(sizeof(*sparse) +
			 (region->caps.nr_areas * sizeof(*sparse->areas)),
			 GFP_KERNEL);
	if (!sparse)
		return NULL;

	for (i = 0; i < region->caps.nr_areas; i++) {
		sparse->areas[i].offset = region->caps.sparse[i].offset;
		sparse->areas[i].size =
		    region->caps.sparse[i].nr_pages << PAGE_SHIFT;
	}
	sparse->nr_areas = region->caps.nr_areas;

	return sparse;
}

struct mdev_net_region *region_from_index(struct mdev_device *mdev, int index)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	int i;

	if (!netmdev->vdev.regions)
		return NULL;

	for (i = 0; i < netmdev->vdev.used_regions; i++) {
		if (netmdev->vdev.regions[i].offset ==
			VFIO_PCI_INDEX_TO_OFFSET(index))
			return &netmdev->vdev.regions[i];
	}

	return NULL;
}

static long netmdev_dev_ioctl(struct mdev_device *mdev, unsigned int cmd,
			      unsigned long arg)
{
	unsigned long minsz;
	struct net_device *netdev;
	struct netmdev *netmdev;
	struct vfio_device_info device_info;
	struct vfio_region_info reg_info;
	struct vfio_irq_info irq_info;
	struct vfio_info_cap caps = { .buf = NULL, .size = 0 };
	struct vfio_region_info_cap_type cap_type;
	struct vfio_region_info_cap_sparse_mmap *sparse = NULL;
	struct vfio_iommu_type1_dma_map dma_map;
	struct vfio_iommu_type1_dma_unmap dma_unmap;
	int ret = 0;
	int max_regions = 0;
	struct mdev_net_region *region;

	if (!mdev)
		return -EINVAL;

	netdev = mdev_get_netdev(mdev);
	netmdev = mdev_get_drvdata(mdev);

	if (!netdev)
		return -ENODEV;

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
		minsz = offsetofend(struct vfio_device_info, num_irqs);
		if (!netmdev->vdev.regions) {
			dev_err(&mdev->dev, "vdev not initialized properly\n");
			return -EFAULT;
		}

		if (copy_from_user(&device_info, (void __user *)arg, minsz))
			return -EFAULT;

		if (device_info.argsz < minsz)
			return -EINVAL;

		/* shadow page + rx_ring and tx_ring*/
		device_info.flags = netmdev->vdev.bus_flags;
		device_info.num_irqs = netmdev->vdev.num_irqs;
		device_info.num_regions = netmdev->vdev.bus_regions +
			netmdev->vdev.extra_regions;

		return copy_to_user((void __user *)arg, &device_info, minsz) ?
			-EFAULT : 0;
	case VFIO_DEVICE_GET_REGION_INFO:
		max_regions = netmdev->vdev.bus_regions +
			netmdev->vdev.extra_regions;

		minsz = offsetofend(struct vfio_region_info, offset);
		if (copy_from_user(&reg_info, (void __user *)arg, minsz))
			return -EFAULT;

		if (reg_info.argsz < minsz)
			return -EINVAL;

		if(reg_info.index > max_regions)
			return -EINVAL;

		region = region_from_index(mdev, reg_info.index);
		if (!region)
			return -EINVAL;

		reg_info.size = region->nr_pages << PAGE_SHIFT;
		reg_info.offset = region->offset;
		reg_info.flags = region->flags;

		if (reg_info.flags & VFIO_REGION_INFO_FLAG_CAPS) {
			cap_type.type = region->caps.type;
			cap_type.subtype = region->caps.subtype;

			ret = vfio_info_add_capability(&caps,
					VFIO_REGION_INFO_CAP_TYPE,
					&cap_type);
			if (ret)
				return ret;
		}

		if (region->caps.nr_areas) {
			sparse = netmdev_fill_sparse(region);
			if (sparse) {
				ret = vfio_info_add_capability(&caps,
					VFIO_REGION_INFO_CAP_SPARSE_MMAP, sparse);
				kfree(sparse);
			}
			if (ret)
				return ret;
		}

		if (caps.size) {
			if (reg_info.argsz < sizeof(reg_info) + caps.size) {
				reg_info.argsz = sizeof(reg_info) + caps.size;
				reg_info.cap_offset = 0;
			} else {
				vfio_info_cap_shift(&caps, sizeof(reg_info));
				if (copy_to_user((void __user *)arg +
						  sizeof(reg_info), caps.buf,
						  caps.size)) {
					kfree(caps.buf);
					return -EFAULT;
				}
				reg_info.cap_offset = sizeof(reg_info);
			}
			kfree(caps.buf);
		}

		return copy_to_user((void __user *)arg, &reg_info, minsz) ?
			-EFAULT : 0;
	case VFIO_DEVICE_GET_IRQ_INFO:
		/* XXX FIXME insert these to vdev? */
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
	case VFIO_IOMMU_MAP_DMA:
		minsz = offsetofend(struct vfio_iommu_type1_dma_map, size);

		if (copy_from_user(&dma_map, (void __user *)arg, minsz))
			ret = -EFAULT;

		if (dma_map.argsz < minsz)
			return -EINVAL;

		ret = netmdev_vfio_mmap_dma(mdev, &dma_map);
		if (ret < 0)
			return ret;

		return copy_to_user((void __user *)arg, &dma_map, minsz) ?
			-EFAULT : 0;
	case VFIO_IOMMU_UNMAP_DMA:
		minsz = offsetofend(struct vfio_iommu_type1_dma_unmap, size);

		if (copy_from_user(&dma_unmap, (void __user *)arg, minsz))
			ret = -EFAULT;

		if (dma_unmap.argsz < minsz)
			return -EINVAL;

		return netmdev_vfio_unmmap_dma(mdev, &dma_unmap);
        default:
                return -EOPNOTSUPP;
	}

	return -EINVAL;
}

static int netmdev_dev_mmap(struct mdev_device *mdev, struct vm_area_struct *vma)
{
	struct net_device *netdev;
	struct netmdev *netmdev;
	u64 req_len;
	unsigned int index;
	unsigned long pfn, nr_pages;
	struct mdev_net_region *region;
	u64 offset = vma->vm_pgoff << PAGE_SHIFT;
	u32 max;

	/* userland wants to access ring descrptors that was pre-allocated
	 * by the kernel
	 * note: userland need to user IOCTL MAP to CREATE packet buffers
	 */
	netdev = mdev_get_netdev(mdev);
	netmdev = mdev_get_drvdata(mdev);

	if (vma->vm_end <= vma->vm_start)
		return -EINVAL;
	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;

	max = netmdev->vdev.bus_regions + netmdev->vdev.extra_regions;

	index = VFIO_PCI_OFFSET_TO_INDEX(offset);
	if (index > max)
		return -EINVAL;

	region = region_from_index(mdev, index);
	if (!region)
		return -EINVAL;

	pfn = nr_pages = 0;
	if (!region->caps.nr_areas) {
		if (region->offset == offset) {
			pfn = region->pfn;
			nr_pages = region->nr_pages;
		}
	} else {
		int i;

		/* Lookup relevant area => (pfn, nr_pages) */
		for (i = 0; i < region->caps.nr_areas; i++) {
			struct mdev_net_sparse *area = &region->caps.sparse[i];

			if (area->offset == offset) {
				pfn = area->pfn;
				nr_pages = area->nr_pages;
			}
		}
	}

	req_len = vma->vm_end - vma->vm_start;
	if (req_len != (u64)nr_pages << PAGE_SHIFT)
		return -EINVAL;

	vma->vm_private_data = &netmdev->vdev;
	vma->vm_pgoff = pfn;

	return io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       req_len, vma->vm_page_prot);
}

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

	for (i = 0; i < netmdev_known_drivers_count; i++) {
		if (netmdev_known_drivers[i].driver == driver)
			return	netmdev_known_drivers[i].drv_ops;
	}
	printk(KERN_ERR "netmdev_get_driver_ops could not find driver\n");

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

int mdev_net_add_sparse(struct mdev_net_region *region, __u64 offset,
                        unsigned long pfn, unsigned long nr_pages)
{
	struct mdev_net_sparse *sparse;

	sparse =
	    kzalloc((region->caps.nr_areas + 1) * sizeof(*sparse), GFP_KERNEL);
	if (!sparse)
		return -ENOMEM;

	memcpy(sparse, region->caps.sparse,
	       region->caps.nr_areas * sizeof(*sparse));
	kfree(region->caps.sparse);

	region->caps.sparse = sparse;

	sparse += region->caps.nr_areas;
	sparse->offset = offset;
	sparse->pfn = pfn;
	sparse->nr_pages = nr_pages;

	region->caps.nr_areas++;

	return 0;
}
EXPORT_SYMBOL(mdev_net_add_sparse);

/*
 * All devices will need region/capabilities and mmap properties
 * some of them will additionally need sparse map information
 */
void mdev_net_add_essential(struct mdev_net_region *region, __u32 type,
			    __u32 subtype, __u64 offset, unsigned long pfn,
			    unsigned long nr_pages)
{
	const int flags =
	    VFIO_REGION_INFO_FLAG_READ | VFIO_REGION_INFO_FLAG_WRITE |
	    VFIO_REGION_INFO_FLAG_MMAP | VFIO_REGION_INFO_FLAG_CAPS;

	region->caps.type = type;
	region->caps.subtype = subtype;
	region->flags = flags;
	region->offset = offset;
	region->pfn = pfn;
	region->nr_pages = nr_pages;
	region->caps.nr_areas = 0;
	region->caps.sparse = NULL;
}
EXPORT_SYMBOL(mdev_net_add_essential);

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
