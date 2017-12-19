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
static void cxgb4_destroy_vdev(struct mdev_device *mdev);

static int cxgb4_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct pci_dev *pdev = pi->adapter->pdev;
	struct mdev_net_region *region;
	int i;
	int alloc_regions = 0;
	phys_addr_t start;
	u64 size, offset;
	int offset_cnt;
	int bar_index;

	netmdev->vdev.bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev.extra_regions = 2 * pi->nqsets;
	alloc_regions = netmdev->vdev.extra_regions + 1;
	netmdev->vdev.used_regions = 0;

	netmdev->vdev.bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev.num_irqs = 1;

	netmdev->vdev.regions =
		kcalloc(alloc_regions,
			sizeof(*netmdev->vdev.regions), GFP_KERNEL);
	if (!netmdev->vdev.regions)
		goto err;

	region = netmdev->vdev.regions;

	bar_index = is_t4(pi->adapter->params.chip) ? VFIO_PCI_BAR0_REGION_INDEX :
		VFIO_PCI_BAR2_REGION_INDEX;
	start = pci_resource_start(pdev, bar_index);
	size = pci_resource_len(pdev, bar_index);
	offset = VFIO_PCI_INDEX_TO_OFFSET(bar_index);
	mdev_net_add_essential(region++, VFIO_NET_MMIO, VFIO_NET_MDEV_BARS,
			       offset, start >> PAGE_SHIFT, size >> PAGE_SHIFT);
	offset_cnt = netmdev->vdev.bus_regions;

	/* Rx + Rx free list */
	for (i = 0; i < pi->nqsets; i++) {
		struct sge_rspq *iq = &pi->adapter->sge.ethrxq[i].rspq;
		struct sge_fl *fl = &pi->adapter->sge.ethrxq[i].fl;
		struct sge *s = &pi->adapter->sge;

		offset = VFIO_PCI_INDEX_TO_OFFSET(offset_cnt++);
		mdev_net_add_essential(region, VFIO_NET_DESCRIPTORS,
				       VFIO_NET_MDEV_RX, offset, 0, 0);

		start = virt_to_phys(iq->desc);
		size = PAGE_ALIGN(iq->size * iq->iqe_len);
		mdev_net_add_sparse(region, offset, start >> PAGE_SHIFT,
				    size >> PAGE_SHIFT);

		offset += size + PAGE_SIZE;

		start = virt_to_phys(fl->desc);
		size = PAGE_ALIGN(fl->size * sizeof(*fl->desc) + s->stat_len);
		mdev_net_add_sparse(region, offset, start >> PAGE_SHIFT,
				    size >> PAGE_SHIFT);

		region->nr_pages = (offset + size) >> PAGE_SHIFT;

		region++;
	}

	/* Tx */
	for (i = 0; i < pi->nqsets; i++) {
		struct sge_txq *q = &pi->adapter->sge.ethtxq[i].q;
		struct sge *s = &pi->adapter->sge;

		start = virt_to_phys(q->desc);
		size = PAGE_ALIGN(q->size * sizeof(*q->desc) + s->stat_len);
		offset = VFIO_PCI_INDEX_TO_OFFSET(offset_cnt++);
		mdev_net_add_essential(region++, VFIO_NET_DESCRIPTORS,
				       VFIO_NET_MDEV_TX, offset,
				       start >> PAGE_SHIFT, size >> PAGE_SHIFT);
	}

	netmdev->vdev.used_regions = region - netmdev->vdev.regions;
	BUG_ON(netmdev->vdev.used_regions != alloc_regions);

	return 0;

err:
	cxgb4_destroy_vdev(mdev);
	return -ENOMEM;
}

static void cxgb4_destroy_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	int i;

	for (i = 0; i < netmdev->vdev.used_regions; i++) {
		struct mdev_net_region *region = &netmdev->vdev.regions[i];

		kfree(region->caps.sparse);
	}

	kfree(netmdev->vdev.regions);
}

static int cxgb4_transition_start(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct adapter *adapter = pi->adapter;

	dev_hold(netdev);

	if (cxgb4_init_vdev(mdev)) {
		dev_put(netdev);
		return -EINVAL;
	}

	t4_intr_disable(adapter);
	/* XXX Check if we have to free queues to save resources */
	t4_sge_stop(adapter);

	return 0;
}

static int cxgb4_transition_back(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct adapter *adapter = pi->adapter;

	t4_sge_start(adapter);
	t4_intr_enable(adapter);

	if (t4_update_port_info(pi) < 0) {
		dev_put(netdev);
		return -EINVAL;
	}

	cxgb4_destroy_vdev(mdev);

	dev_put(netdev);

	return 0;
}

static struct netmdev_driver_ops cxgb4_netmdev_driver_ops = {
	.transition_start = cxgb4_transition_start,
	.transition_back = cxgb4_transition_back,
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
