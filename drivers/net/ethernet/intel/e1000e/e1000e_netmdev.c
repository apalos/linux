/* Intel PRO/1000 Linux driver
 * Copyright (c) 2017, Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mm.h>
#include <linux/vfio.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>

#include "e1000.h"
#include "e1000e_netmdev.h"

static int e1000e_get_bus_info(struct net_device *netdev,
			       struct net_mdev_bus_info *bus_info);

static int e1000e_transition_start(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	dev_hold(netdev);

	adapter->irq_mask =
	    E1000_IMS_RXT0 | E1000_IMS_TXDW |
	    E1000_IMS_RXDMT0 | E1000_IMS_RXSEQ;

	if (netif_running(netdev))
		e1000e_reinit_locked(adapter);
	else
		e1000e_reset(adapter);

	e1000e_trigger_lsc(adapter);

	return 0;
}

static int e1000e_transition_back(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	adapter->irq_mask = 0;

	if (netif_running(adapter->netdev))
		e1000e_reinit_locked(adapter);
	else
		e1000e_reset(adapter);

	dev_put(adapter->netdev);

	return 0;
}

static int e1000e_get_device_info(struct net_device *netdev,
				  struct vfio_device_info *info)
{
	struct net_mdev_bus_info bus_info;

	e1000e_get_bus_info(netdev, &bus_info);

	info->flags = VFIO_DEVICE_FLAGS_PCI;
	info->num_regions = bus_info.bus_max + bus_info.extra;
	info->num_irqs = 1;

	return 0;
}

static int e1000e_get_bus_info(struct net_device *netdev,
			       struct net_mdev_bus_info *bus_info)
{
	bus_info->bus_max = VFIO_PCI_NUM_REGIONS;
        bus_info->extra = VFIO_NET_MDEV_NUM_REGIONS;

	return 0;
}

static int e1000e_get_region_info(struct net_device *netdev,
				  struct vfio_region_info *info)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	switch (info->index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		/* PCI resource 0 */
		info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
		info->size = pci_resource_len(adapter->pdev, info->index);
		info->flags = VFIO_REGION_INFO_FLAG_MMAP;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int e1000e_get_cap_info(struct net_device *netdev, u32 region,
			       struct vfio_region_info_cap_type *cap_type,
			       struct vfio_region_info *info)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

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
		info->size = adapter->rx_ring[0].size;
		break;

	case VFIO_NET_MDEV_TX_REGION_INDEX:
		cap_type->type = VFIO_NET_DESCRIPTORS;
		cap_type->subtype = VFIO_NET_MDEV_TX;
		info->size = adapter->tx_ring[0].size;
		break;

	default:
		return -EINVAL;
	}

	info->offset =
	    VFIO_PCI_INDEX_TO_OFFSET(region + VFIO_PCI_NUM_REGIONS);
	info->flags =
	    VFIO_REGION_INFO_FLAG_READ | VFIO_REGION_INFO_FLAG_WRITE |
	    VFIO_REGION_INFO_FLAG_MMAP | VFIO_REGION_INFO_FLAG_CAPS;

	return 0;
}

static int e1000e_get_mmap_info(struct net_device *netdev, u32 index,
				unsigned long *pfn, unsigned long *nr_pages)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	phys_addr_t start = 0;
	u64 len = 0;

	switch (index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		start = pci_resource_start(adapter->pdev, index);
		len = pci_resource_len(adapter->pdev, index);
		break;

	case VFIO_PCI_NUM_REGIONS + VFIO_NET_MDEV_RX_REGION_INDEX:
		/* RX descriptor ring */
		start = virt_to_phys(adapter->rx_ring[0].desc);
		len = adapter->rx_ring[0].size;
		break;

	case VFIO_PCI_NUM_REGIONS + VFIO_NET_MDEV_TX_REGION_INDEX:
		/* TX descriptor ring */
		start = virt_to_phys(adapter->tx_ring[0].desc);
		len = adapter->tx_ring[0].size;
		break;

	default:
		return -EINVAL;
	}

	*pfn = start >> PAGE_SHIFT;
	*nr_pages = PAGE_ALIGN(len) >> PAGE_SHIFT;

	return 0;
}

static int e1000e_get_irq_info(struct net_device *netdev,
			       struct vfio_irq_info *info)
{
	info->flags = VFIO_IRQ_INFO_EVENTFD | VFIO_IRQ_INFO_MASKABLE |
	    VFIO_IRQ_INFO_AUTOMASKED;
	info->count = 1;

	return 0;
}

static struct netmdev_driver_ops e1000e_netmdev_driver_ops = {
	.transition_start = e1000e_transition_start,
	.transition_back = e1000e_transition_back,
	.get_device_info = e1000e_get_device_info,
	.get_bus_info = e1000e_get_bus_info,
	.get_region_info = e1000e_get_region_info,
	.get_cap_info = e1000e_get_cap_info,
	.get_mmap_info = e1000e_get_mmap_info,
	.get_irq_info = e1000e_get_irq_info,
};

void e1000e_register_netmdev(struct device *dev)
{
	int (*register_device) (struct device * d,
				struct netmdev_driver_ops * ops);

	register_device = symbol_get(netmdev_register_device);
	if (!register_device)
		return;

	if (register_device(dev, &e1000e_netmdev_driver_ops) < 0)
		dev_err(dev, "Could not register device\n");
	else
		dev_info(dev, "Successfully registered net_mdev device\n");

	symbol_put(netmdev_register_device);
}

void e1000e_unregister_netmdev(struct device *dev)
{
	int (*unregister_device) (struct device *);

	unregister_device = symbol_get(netmdev_unregister_device);
	if (!unregister_device)
		return;

	if (unregister_device(dev) < 0)
		dev_err(dev, "Could not unregister device\n");
	else
		dev_info(dev,
			 "Successfully unregistered net_mdev device\n");

	symbol_put(netmdev_unregister_device);
}
