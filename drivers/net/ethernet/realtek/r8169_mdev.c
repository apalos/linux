#include <linux/module.h>
#include <linux/mii.h>
#include <linux/pci.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>
#include <uapi/linux/net_mdev.h>
#include <linux/netdevice.h>

#include "r8169_private.h"
extern void rtl_set_rx_tx_desc_registers(struct rtl8169_private *tp,
					 void __iomem *ioaddr);
extern void rtl_reset_work(struct rtl8169_private *tp);
extern int rtl8169_init_ring(struct net_device *dev);

void r8169_mdev_close(struct net_device *dev);
void r8169_mdev_prepare(struct net_device *dev);
int r8169_mdev_destroy(struct net_device *dev);

static int r8169_transition_start(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	void __iomem *ioaddr;
	struct rtl8169_private *tp;

	tp = netdev_priv(netdev);
	if (!tp)
		return -EINVAL;

	ioaddr = tp->mmio_addr;

	r8169_mdev_prepare(netdev);

	return 0;
}

static int r8169_transition_back(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	void __iomem *ioaddr;
	struct rtl8169_private *tp;

	tp = netdev_priv(netdev);
	if (!tp)
		return -EINVAL;

	ioaddr = tp->mmio_addr;

	return r8169_mdev_destroy(netdev);
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

static int r8169_get_region(struct mdev_device *mdev, struct vfio_region_info *info)

{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct rtl8169_private *tp;
	struct pci_dev *pdev;

	tp = netdev_priv(netdev);
	if (!tp)
		return  -EFAULT;

	pdev = tp->pci_dev;

	switch (info->index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
	case VFIO_PCI_BAR1_REGION_INDEX:
	case VFIO_PCI_BAR3_REGION_INDEX ... VFIO_PCI_NUM_REGIONS:
		info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
		info->size = 0;
		info->flags = 0;
		break;
	case VFIO_PCI_BAR2_REGION_INDEX:
		info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
		info->size = pci_resource_len(pdev, info->index);
		if (!info->size) {
			info->flags = 0;
			break;
		}
		info->flags = VFIO_REGION_INFO_FLAG_READ |
			VFIO_REGION_INFO_FLAG_WRITE |
			VFIO_REGION_INFO_FLAG_MMAP;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int r8169_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev->extra_regions = VFIO_NET_MDEV_NUM_REGIONS;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	return 0;
}

struct netmdev_driver_ops rtl8169_netmdev_driver_ops =
{
	.transition_start = r8169_transition_start,
	.transition_back = r8169_transition_back,
	.get_region_info = r8169_get_region,
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
