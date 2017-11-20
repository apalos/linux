#include <linux/module.h>
#include <linux/mii.h>
#include <linux/pci.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>
#include <uapi/linux/net_mdev.h>
#include <linux/netdevice.h>

#include "r8169_private.h"

void r8169_mdev_prepare(struct net_device *dev);
int r8169_mdev_destroy(struct net_device *dev);

/* BAR2, RX/TX queues */
#define RTL_USED_REGIONS 3

static int r8169_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct rtl8169_private *tp;
	struct mdev_net_region *region;
	struct pci_dev *pdev;
	phys_addr_t start;
	u64 size, offset;

	tp = netdev_priv(netdev);
	if (!tp)
		return -EFAULT;

	pdev = tp->pci_dev;

	netmdev->vdev = kzalloc(sizeof(netmdev->vdev), GFP_KERNEL);
	if (!netmdev->vdev)
		return -ENOMEM;

	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev->extra_regions = VFIO_NET_MDEV_NUM_REGIONS;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	netmdev->vdev->regions =
	    kzalloc(RTL_USED_REGIONS *
		    sizeof(*netmdev->vdev->regions), GFP_KERNEL);
	if (!netmdev->vdev->regions) {
		kfree(netmdev->vdev);
		return -ENOMEM;
	}

	/* BAR MMIO */
	region = &netmdev->vdev->regions[netmdev->vdev->used_regions++];
	start = pci_resource_start(pdev, VFIO_PCI_BAR2_REGION_INDEX);
	size = pci_resource_len(pdev, VFIO_PCI_BAR2_REGION_INDEX);
	offset = VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR2_REGION_INDEX);
	mdev_net_add_essential(region, VFIO_NET_MMIO, VFIO_NET_MDEV_BARS,
			       offset, start >> PAGE_SHIFT, size >> PAGE_SHIFT);

	/* Rx */
	region = &netmdev->vdev->regions[netmdev->vdev->used_regions++];
	start = virt_to_phys(tp->RxDescArray);
	size = PAGE_ALIGN(R8169_RX_RING_BYTES);
	offset = VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_RX_REGION_INDEX +
					  netmdev->vdev->bus_regions);
	mdev_net_add_essential(region, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_RX,
			       offset, start >> PAGE_SHIFT, size >> PAGE_SHIFT);

	/* Tx */
	region = &netmdev->vdev->regions[netmdev->vdev->used_regions++];
	start = virt_to_phys(tp->TxDescArray);
	size = PAGE_ALIGN(R8169_TX_RING_BYTES);
	offset = VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_TX_REGION_INDEX +
					  netmdev->vdev->bus_regions);
	mdev_net_add_essential(region, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_TX,
			       offset, start >> PAGE_SHIFT, size >> PAGE_SHIFT);

	BUG_ON(netmdev->vdev->used_regions != RTL_USED_REGIONS);

	return 0;
}

void r8169_destroy_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	if (netmdev->vdev) {
		if (netmdev->vdev->regions)
			kfree(netmdev->vdev->regions);
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

	register_device  = symbol_get(netmdev_register_device);
	if (!register_device)
		return;
	if (register_device(dev, &rtl8169_netmdev_driver_ops) < 0)
		dev_err(dev, "Could not register device\n");
	else
		dev_info(dev, "Successfully registered net_mdev device\n");
	symbol_put(netmdev_register_device);

}

void r8169_unregister_netmdev(struct device *dev)
{
	int (*unregister_device)(struct device*);

	unregister_device = symbol_get(netmdev_unregister_device);
	if (!unregister_device)
		return;
	if (unregister_device(dev) < 0)
		dev_err(dev, "Could not unregister device\n");
	else
		dev_info(dev, "Successfully unregistered net_mdev device\n");
	symbol_put(netmdev_unregister_device);
}
