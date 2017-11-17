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

#define E1000E_MDEV_USED_REGIONS 3

static int e1000e_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct mdev_net_regions *info;
	struct pci_dev *pdev;
	phys_addr_t start;
	u64 size, idx;

	pdev = adapter->pdev;

	netmdev->vdev = kzalloc(sizeof(netmdev->vdev), GFP_KERNEL);
	if (!netmdev->vdev)
		return -ENOMEM;

	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev->extra_regions = VFIO_NET_MDEV_NUM_REGIONS;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	netmdev->vdev->vdev_regions =
		kzalloc(E1000E_MDEV_USED_REGIONS *
			sizeof(*netmdev->vdev->vdev_regions), GFP_KERNEL);
	if (!netmdev->vdev->vdev_regions) {
		kfree(netmdev->vdev);
		return -ENOMEM;
	}

	/* BAR MMIO */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = pci_resource_start(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	size = pci_resource_len(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	idx = VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR0_REGION_INDEX);
	mdev_net_add_essential(info, idx, size, VFIO_NET_MMIO, VFIO_NET_MDEV_BARS,
			       start);

	/* Rx */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = virt_to_phys(adapter->rx_ring[0].desc);
	size = adapter->rx_ring[0].size;
	idx = VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_RX_REGION_INDEX +
			netmdev->vdev->bus_regions);
	mdev_net_add_essential(info, idx, size, VFIO_NET_DESCRIPTORS,
			       VFIO_NET_MDEV_RX, start);

	/* Tx */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = virt_to_phys(adapter->tx_ring[0].desc);
	size = adapter->tx_ring[0].size;
	idx = VFIO_PCI_INDEX_TO_OFFSET(VFIO_NET_MDEV_TX_REGION_INDEX +
			netmdev->vdev->bus_regions);
	mdev_net_add_essential(info, idx, size, VFIO_NET_DESCRIPTORS,
			       VFIO_NET_MDEV_TX, start);

	BUG_ON(netmdev->vdev->used_regions != E1000E_MDEV_USED_REGIONS);

	return 0;
}

void e1000e_destroy_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	if (netmdev->vdev) {
		if (netmdev->vdev->vdev_regions)
			kfree(netmdev->vdev->vdev_regions);
		kfree(netmdev->vdev);
	}
}


static int e1000e_transition_start(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	int ret;

	dev_hold(netdev);

	adapter->irq_mask =
	    E1000_IMS_RXT0 | E1000_IMS_TXDW |
	    E1000_IMS_RXDMT0 | E1000_IMS_RXSEQ;

	netif_carrier_off(netdev);

	ret = e1000e_init_vdev(mdev);
	if (ret)
		return -EINVAL;

	if (netif_running(netdev))
		e1000e_reinit_locked(adapter);
	else
		e1000e_reset(adapter);

	return 0;
}

static int e1000e_transition_back(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	adapter->irq_mask = 0;

	if (netif_running(adapter->netdev))
		e1000e_reinit_locked(adapter);
	else
		e1000e_reset(adapter);

	dev_put(adapter->netdev);
	e1000e_destroy_vdev(mdev);

	return 0;
}

static struct netmdev_driver_ops e1000e_netmdev_driver_ops = {
	.transition_start = e1000e_transition_start,
	.transition_back = e1000e_transition_back,
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
