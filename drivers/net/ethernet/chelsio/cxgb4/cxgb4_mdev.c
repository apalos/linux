/*
 * This file is part of the Chelsio T4 Ethernet driver for Linux.
 *
 * Copyright (c) 2017, Linaro Limited
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mm.h>
#include <linux/vfio.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>

#include "cxgb4.h"

struct net_device *mdev_get_netdev(struct mdev_device *mdev);

static int cxgb4_transition_start(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);

	dev_hold(netdev);
	netif_carrier_off(netdev);

	return 0;
}

static int cxgb4_transition_back(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);

	dev_put(netdev);

	return 0;
}

static int cxgb4_init_vdev(struct mdev_device *mdev)

{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);


	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	/* FIXME find #define for that */
	netmdev->vdev->extra_regions = 64;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	return 0;
}

static int cxgb4_get_sparse_info(struct mdev_device *mdev, u64 *offset, u64 *size,
				 int region, int area)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);

	if (region >= VFIO_NET_MDEV_NUM_REGIONS)
		return -EINVAL;
	switch (region) {
	case VFIO_NET_MDEV_RX_REGION_INDEX:
		break;

	case VFIO_NET_MDEV_TX_REGION_INDEX:
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int cxgb4_get_region_info(struct mdev_device *mdev,
				 struct vfio_region_info *info)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);

	switch (info->index) {
	case VFIO_PCI_BAR2_REGION_INDEX:
		/* PCI resource 2 */
		info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
		info->size = pci_resource_len(pi->adapter->pdev, info->index);
		info->flags = VFIO_REGION_INFO_FLAG_MMAP;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int cxgb4_get_cap_info(struct mdev_device *mdev, u32 region,
			      struct vfio_region_info_cap_type *cap_type,
			      struct vfio_region_info *info, int *nr_areas)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	int num_desc;

	num_desc = netmdev->vdev->bus_regions + netmdev->vdev->extra_regions;
	if (region >= num_desc)
		return -EINVAL;

	if (region == VFIO_NET_MDEV_SHADOW_REGION_INDEX) {
		cap_type->type = VFIO_NET_MDEV_SHADOW;
		cap_type->subtype = VFIO_NET_MDEV_STATS;
		info->size = 0;
	} else if (region > VFIO_NET_MDEV_SHADOW_REGION_INDEX && region < num_desc) {
		cap_type->type = VFIO_NET_DESCRIPTORS;
		if (region % VFIO_NET_MDEV_TX_REGION_INDEX)
			cap_type->subtype = VFIO_NET_MDEV_TX;
		else
			cap_type->subtype = VFIO_NET_MDEV_RX;
		info->size = 128;
	} else {
		return -EINVAL;
	}
	*nr_areas = 15;

	info->offset = VFIO_PCI_INDEX_TO_OFFSET(region + VFIO_PCI_NUM_REGIONS);
	info->flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP |
		VFIO_REGION_INFO_FLAG_CAPS;

	return 0;
}

static int cxgb4_get_mmap_info(struct mdev_device *mdev, u32 index,
			       unsigned long *pfn, unsigned long *nr_pages)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct sge *s = &pi->adapter->sge;
	struct sge_eth_rxq *rxq = &s->ethrxq[pi->first_qset];
	struct sge_eth_txq *txq = &s->ethtxq[pi->first_qset];
	phys_addr_t start = 0;
	u64 len = 0;

	switch (index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		//start = pci_resource_start(pi->adapter->pdev, index);
		//len = pci_resource_len(pi->adapter->pdev, index);
		break;

	case VFIO_PCI_NUM_REGIONS + VFIO_NET_MDEV_RX_REGION_INDEX:
		/* RX descriptor ring */
		//start = virt_to_phys(adapter->rx_ring[0].desc);
		//len = adapter->rx_ring[0].size;
		break;

	case VFIO_PCI_NUM_REGIONS + VFIO_NET_MDEV_TX_REGION_INDEX:
		/* TX descriptor ring */
		//start = virt_to_phys(adapter->tx_ring[0].desc);
		//len = adapter->tx_ring[0].size;
		break;

	default:
		return -EINVAL;
	}

	*pfn = start >> PAGE_SHIFT;
	*nr_pages = PAGE_ALIGN(len) >> PAGE_SHIFT;

	return 0;
}

static struct netmdev_driver_ops cxgb4_netmdev_driver_ops = {
	.transition_start = cxgb4_transition_start,
	.transition_back = cxgb4_transition_back,
	.get_region_info = cxgb4_get_region_info,
	.get_cap_info = cxgb4_get_cap_info,
	.get_sparse_info = cxgb4_get_sparse_info,
	.get_mmap_info = cxgb4_get_mmap_info,
};

void cxgb4_register_netmdev(struct device *dev)
{
	int (*register_device)(struct device *d,
			       struct netmdev_driver_ops *ops);

	register_device = symbol_get(netmdev_register_device);
	if (!register_device)
		return;

	if (register_device(dev, &cxgb4_netmdev_driver_ops) < 0)
		dev_err(dev, "Could not register device\n");
	else
		dev_info(dev, "Successfully registered net_mdev device\n");

	symbol_put(netmdev_register_device);
}

void cxgb4_unregister_netmdev(struct device *dev)
{
	int (*unregister_device)(struct device *d);

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
