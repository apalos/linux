#include <linux/module.h>
#include <linux/mii.h>
#include <linux/pci.h>
#include <linux/net_mdev.h>
#include <uapi/linux/net_mdev.h>
#include <linux/netdevice.h>

#include "r8169_private.h"
extern void rtl8169_rx_clear(struct rtl8169_private *tp);
extern void rtl_set_rx_tx_desc_registers(struct rtl8169_private *tp,
					 void __iomem *ioaddr);
extern void rtl_hw_start(struct net_device *dev);
extern int rtl8169_init_ring(struct net_device *dev);

static int r8169_transition_start(struct net_device* netdev)
{
	void __iomem *ioaddr;
	struct rtl8169_private *tp;

	tp = netdev_priv(netdev);
	if (!tp)
		return -EINVAL;

	ioaddr = tp->mmio_addr;
	RTL_W8(ChipCmd, RTL_R8(ChipCmd) & ~(CmdTxEnb | CmdRxEnb));
	/* deallocate kernel buffers from ring */
	rtl8169_rx_clear(netdev_priv(netdev));
	rtl_set_rx_tx_desc_registers(tp, ioaddr);
	return 0;
}

static int r8169_transition_complete(struct net_device* netdev)
{
	void __iomem *ioaddr;
	struct rtl8169_private *tp;

	tp = netdev_priv(netdev);
	if (!tp)
		return -EINVAL;

	ioaddr = tp->mmio_addr;
	rtl_hw_start(netdev);
	RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);
	return 0;
}

static int r8169_transition_back(struct net_device* netdev)
{
	void __iomem *ioaddr;
	struct rtl8169_private *tp;
	tp = netdev_priv(netdev);
	if (!tp)
		return -EINVAL;

	ioaddr = tp->mmio_addr;
	rtl8169_init_ring(netdev);
	rtl_hw_start(netdev);
	RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);
	return 0;
}

static int r8169_get_mmap(struct net_device *netdev, u32 index,
			  unsigned long *pfn, unsigned long *nr_pages)
{
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

static int r8169_get_extra_regions(struct net_device *ndev, u32 region,
				   struct vfio_region_info_cap_type *cap_type,
				   struct vfio_region_info *info)
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

	info->offset = VFIO_PCI_INDEX_TO_OFFSET(region + VFIO_PCI_NUM_REGIONS);
	info->flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP |
		VFIO_REGION_INFO_FLAG_CAPS;

	return 0;
}

static int r8169_get_region(struct net_device *netdev, struct vfio_region_info *info)

{
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

static int r8169_bus_info(struct net_device *netdev, struct net_mdev_bus_info *bus_info)
{
	bus_info->bus_max = VFIO_PCI_NUM_REGIONS;
	bus_info->extra = VFIO_NET_MDEV_NUM_REGIONS;

	return 0;
}

static int r8169_get_dev(struct net_device *netdev, struct vfio_device_info *info)
{
	struct net_mdev_bus_info bus_info;

	r8169_bus_info(netdev, &bus_info);

	info->flags = VFIO_DEVICE_FLAGS_PCI;
	info->num_regions = bus_info.bus_max + bus_info.extra;
	info->num_irqs = 1;

	return 0;
}

static int r8169_get_irq(struct net_device *netdev, struct vfio_irq_info *info)
{
	info->flags = VFIO_IRQ_INFO_EVENTFD | VFIO_IRQ_INFO_MASKABLE |
			VFIO_IRQ_INFO_AUTOMASKED;
	info->count = 1;

	return 0;
}

struct netmdev_driver_ops rtl8169_netmdev_driver_ops =
{
	.transition_start = r8169_transition_start,
	.transition_complete = r8169_transition_complete,
	.transition_back = r8169_transition_back,
	.get_region_info = r8169_get_region,
	.get_cap_info = r8169_get_extra_regions,
	.get_mmap_info = r8169_get_mmap,
	.get_device_info = r8169_get_dev,
	.get_irq_info = r8169_get_irq,
	.get_bus_info = r8169_bus_info,
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
