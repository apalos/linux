#include <linux/module.h>
#include <linux/mii.h>
#include <linux/pci.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>
#include <uapi/linux/net_mdev.h>
#include <linux/netdevice.h>

#include "r8169_private.h"
//#include "mdev_private.h"

void r8169_mdev_prepare(struct net_device *dev);
int r8169_mdev_destroy(struct net_device *dev);

/* BAR2, RX/TX queues */
#define RTL_USED_REGIONS 3

#define to_netdev_queue(obj) container_of(obj, struct netdev_queue, kobj)

#define MDEV_NET_ATTR_RO(_name) \
	struct kobj_attribute mdev_attr_##_name = __ATTR_RO(_name)

static ssize_t doorbell_offset_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	struct netdev_queue *queue = to_netdev_queue(kobj);

	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}
MDEV_NET_ATTR_RO(doorbell_offset);

static const struct attribute *r8169_mdev_attrs[] = {
	&mdev_attr_doorbell_offset.attr,
};

static int r8169_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct rtl8169_private *tp;
	struct mdev_net_regions *info;
	struct pci_dev *pdev;
	int mmio_flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP;
	int cap_flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP |
		VFIO_REGION_INFO_FLAG_CAPS;
	phys_addr_t start;
	u64 len;

	tp = netdev_priv(netdev);
	if (!tp)
		return  -EFAULT;

	pdev = tp->pci_dev;

	netmdev->vdev = kzalloc(sizeof(netmdev->vdev), GFP_KERNEL);
	if (!netmdev->vdev)
		return -ENOMEM;

	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev->extra_regions = VFIO_NET_MDEV_NUM_REGIONS;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	netmdev->vdev->vdev_regions =
		kzalloc(RTL_USED_REGIONS *
			sizeof(*netmdev->vdev->vdev_regions), GFP_KERNEL);
	if (!netmdev->vdev->vdev_regions) {
		kfree(netmdev->vdev);
		return -ENOMEM;
	}

	/* BAR MMIO */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = pci_resource_start(pdev, VFIO_PCI_BAR2_REGION_INDEX);
	len = pci_resource_len(pdev, VFIO_PCI_BAR2_REGION_INDEX);
	mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR2_REGION_INDEX),
			    len, mmio_flags);
	mdev_net_add_mmap(&info, start, len);

	/* Rx */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = virt_to_phys(tp->RxDescArray);
	len = R8169_RX_RING_BYTES;
	mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_RX_REGION_INDEX +
			    netmdev->vdev->bus_regions), len, cap_flags);
	mdev_net_add_cap(&info, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_RX);
	mdev_net_add_mmap(&info, start, len);

	/* Tx */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = virt_to_phys(tp->TxDescArray);
	len = R8169_TX_RING_BYTES;
	mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_TX_REGION_INDEX +
			    netmdev->vdev->bus_regions), len, cap_flags);
	mdev_net_add_cap(&info, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_TX);
	mdev_net_add_mmap(&info, start, len);

	BUG_ON(netmdev->vdev->used_regions != RTL_USED_REGIONS);

	return 0;
}

void r8169_destroy_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	if (netmdev->vdev) {
		if (netmdev->vdev->vdev_regions)
			kfree(netmdev->vdev->vdev_regions);
		kfree(netmdev->vdev);
	}
}

static int r8169_transition_start(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	int ret;

	r8169_mdev_prepare(netdev);
	ret = r8169_init_vdev(mdev);
	if (ret)
		return -EINVAL;

	return 0;
}

static int r8169_transition_back(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	int ret;

	r8169_destroy_vdev(mdev);
	ret = r8169_mdev_destroy(netdev);

	return ret;
}

struct netmdev_driver_ops rtl8169_netmdev_driver_ops =
{
	.transition_start = r8169_transition_start,
	.transition_back = r8169_transition_back,
};

void r8169_register_netmdev(struct device *dev)
{
	int (*register_device)(struct device *d , struct netmdev_driver_ops *ops);
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct netdev_queue *queue = &ndev->_tx[0];

	register_device  = symbol_get(netmdev_register_device);
	if (!register_device)
		return;
	if (register_device(dev, &rtl8169_netmdev_driver_ops) < 0)
		dev_err(dev, "Could not register device\n");
	else
		dev_info(dev, "Successfully registered net_mdev device\n");
	symbol_put(netmdev_register_device);

	//netdev_queue_default_attrs
	sysfs_create_files(&queue->kobj, r8169_mdev_attrs);
}

void r8169_unregister_netmdev(struct device *dev)
{
	int (*unregister_device)(struct device*);
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct netdev_queue *queue = &ndev->_tx[0];

	sysfs_remove_files(&queue->kobj, r8169_mdev_attrs);

	unregister_device = symbol_get(netmdev_unregister_device);
	if (!unregister_device)
		return;
	if (unregister_device(dev) < 0)
		dev_err(dev, "Could not unregister device\n");
	else
		dev_info(dev, "Successfully unregistered net_mdev device\n");
	symbol_put(netmdev_unregister_device);
}
