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

	return 0;
}

static int e1000e_transition_complete(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

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

static int e1000e_get_region(struct net_device *netdev,
			     struct vfio_region_info *info)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	int ret = 0;

	switch (info->index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		/* PCI resource 0 */
		info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
		info->size = pci_resource_len(adapter->pdev, info->index);
		info->flags = VFIO_REGION_INFO_FLAG_MMAP;
		break;

	case VFIO_PCI_NUM_REGIONS + 2:
		/* RX descriptor rings */
		info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
		info->size = adapter->rx_ring[0].size;
		info->flags = VFIO_REGION_INFO_FLAG_MMAP;
		break;

	case VFIO_PCI_NUM_REGIONS + 3:
		/* TX descriptor ring */
		info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
		info->size = adapter->tx_ring[0].size;
		info->flags = VFIO_REGION_INFO_FLAG_MMAP;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int e1000e_get_mmap(struct vm_area_struct *vma,
			   struct net_device *netdev, unsigned long *pfn,
			   u64 * size)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	unsigned long phys_pfn = 0;
	u64 phys_len = 0;
	unsigned int index;
	int ret = 0;

	index = vma->vm_pgoff >> (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT);

	switch (index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		/* PCI resource 0 */
		phys_pfn =
		    pci_resource_start(adapter->pdev, index) >> PAGE_SHIFT;
		phys_len = pci_resource_len(adapter->pdev, index);
		break;

	case VFIO_PCI_NUM_REGIONS + 2:
		/* RX descriptor rings */
		phys_pfn =
		    (u64) virt_to_phys(adapter->rx_ring[0].
				       desc) >> PAGE_SHIFT;
		phys_len = adapter->rx_ring[0].size;
		break;

	case VFIO_PCI_NUM_REGIONS + 3:
		/* TX descriptor rings */
		phys_pfn =
		    (u64) virt_to_phys(adapter->tx_ring[0].
				       desc) >> PAGE_SHIFT;
		phys_len = adapter->tx_ring[0].size;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	*pfn = phys_pfn;
	*size = PAGE_ALIGN(phys_len);

	return ret;
}

static int e1000e_get_dev(struct net_device *netdev,
			  struct vfio_device_info *info)
{
	info->flags = VFIO_DEVICE_FLAGS_PCI;
	info->num_regions = VFIO_PCI_NUM_REGIONS + 4;
	info->num_irqs = 1;

	return 0;
}

static int e1000e_get_irq(struct net_device *netdev,
			  struct vfio_irq_info *info)
{
	info->flags = VFIO_IRQ_INFO_EVENTFD | VFIO_IRQ_INFO_MASKABLE |
	    VFIO_IRQ_INFO_AUTOMASKED;
	info->count = 1;

	return 0;
}

static struct netmdev_driver_ops e1000e_netmdev_driver_ops = {
	.transition_start = e1000e_transition_start,
	.transition_complete = e1000e_transition_complete,
	.transition_back = e1000e_transition_back,
	.get_region_info = e1000e_get_region,
	.get_mmap_info = e1000e_get_mmap,
	.get_device_info = e1000e_get_dev,
	.get_irq_info = e1000e_get_irq,
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
