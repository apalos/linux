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
	struct mdev_net_regions *info;
	struct pci_dev *pdev;
	int mmio_flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP;
	int cap_flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP |
		VFIO_REGION_INFO_FLAG_CAPS;

	tp = netdev_priv(netdev);
	if (!tp)
		return  -EFAULT;

	pdev = tp->pci_dev;

	netmdev->vdev = kzalloc(sizeof(netmdev->vdev), GFP_KERNEL);
	if (!netmdev->vdev)
		return -ENOMEM;

	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev->extra_regions = VFIO_NET_MDEV_NUM_REGIONS;
	netmdev->vdev->used_regions = RTL_USED_REGIONS;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	netmdev->vdev->vdev_regions =
		kzalloc(netmdev->vdev->used_regions * sizeof(*netmdev->vdev->vdev_regions),
			GFP_KERNEL);
	/* BAR MMIO */
	info = &netmdev->vdev->vdev_regions[0];
	mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR2_REGION_INDEX),
			pci_resource_len(pdev, VFIO_PCI_BAR2_REGION_INDEX),
			mmio_flags);
	/* Rx */
	info = &netmdev->vdev->vdev_regions[1];
	mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_RX_REGION_INDEX +
			    netmdev->vdev->bus_regions), R8169_RX_RING_BYTES,
			    cap_flags);
	mdev_net_add_cap(&info, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_RX);
	/* Tx */
	info = &netmdev->vdev->vdev_regions[2];
	mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_TX_REGION_INDEX +
			    netmdev->vdev->bus_regions), R8169_TX_RING_BYTES,
			    cap_flags);
	mdev_net_add_cap(&info, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_TX);

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

static int r8169_get_mmap(struct mdev_device *mdev, u32 index,
			  unsigned long *pfn, unsigned long *nr_pages)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct rtl8169_private *tp = netdev_priv(netdev);
	phys_addr_t start = 0;
	u64 len = 0;

	switch (index) {
	case VFIO_PCI_BAR2_REGION_INDEX:
		start = pci_resource_start(tp->pci_dev, index);
		len = pci_resource_len(tp->pci_dev, index);
		break;

	case VFIO_PCI_NUM_REGIONS + VFIO_NET_MDEV_RX_REGION_INDEX:
		/* RX descriptor ring */
		start = virt_to_phys(tp->RxDescArray);
		len = R8169_RX_RING_BYTES;
		break;

	case VFIO_PCI_NUM_REGIONS + VFIO_NET_MDEV_TX_REGION_INDEX:
		/* TX descriptor ring */
		start = virt_to_phys(tp->TxDescArray);
		len = R8169_TX_RING_BYTES;
		break;

	default:
		return -EINVAL;
	}

	*pfn = start >> PAGE_SHIFT;
	*nr_pages = PAGE_ALIGN(len) >> PAGE_SHIFT;

	return 0;
}

static int r8169_get_extra_regions(struct mdev_device *mdev, u32 region,
				   struct vfio_region_info_cap_type *cap_type,
				   struct vfio_region_info *info, int *nr_areas)
{
	if (region >= VFIO_NET_MDEV_NUM_REGIONS)
		return -EINVAL;

	switch (region) {
	case VFIO_NET_MDEV_SHADOW_REGION_INDEX:
		cap_type->type = VFIO_NET_MDEV_SHADOW;
		cap_type->subtype = VFIO_NET_MDEV_STATS;
		info->size = 0;
		break;

	case VFIO_NET_MDEV_RX_REGION_INDEX:
		cap_type->type = VFIO_NET_DESCRIPTORS;
		cap_type->subtype = VFIO_NET_MDEV_RX;
		info->size = R8169_RX_RING_BYTES;
		break;

	case VFIO_NET_MDEV_TX_REGION_INDEX:
		cap_type->type = VFIO_NET_DESCRIPTORS;
		cap_type->subtype = VFIO_NET_MDEV_TX;
		info->size = R8169_TX_RING_BYTES;
		break;

	default:
		return -EINVAL;
	}
	*nr_areas = 0;

	info->offset = VFIO_PCI_INDEX_TO_OFFSET(region + VFIO_PCI_NUM_REGIONS);
	info->flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP |
		VFIO_REGION_INFO_FLAG_CAPS;

	return 0;
}

struct netmdev_driver_ops rtl8169_netmdev_driver_ops =
{
	.transition_start = r8169_transition_start,
	.transition_back = r8169_transition_back,
	.get_cap_info = r8169_get_extra_regions,
	.get_mmap_info = r8169_get_mmap,
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
