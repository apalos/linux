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

static int cxgb4_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct mdev_net_regions *info;
	struct mdev_net_sparse *sparse = NULL;
	struct pci_dev *pdev;
	int i;
	int cnt = 0;
	int nr_areas = 1;
	phys_addr_t start;
	u64 size, idx;

	pdev = pi->adapter->pdev;

	netmdev->vdev = kzalloc(sizeof(netmdev->vdev), GFP_KERNEL);
	if (!netmdev->vdev)
		goto alloc_fail;

	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev->extra_regions = VFIO_NET_MDEV_NUM_REGIONS;
	netmdev->vdev->used_regions = pi->nqsets + 1;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	netmdev->vdev->vdev_regions =
		kzalloc(netmdev->vdev->used_regions *
			sizeof(*netmdev->vdev->vdev_regions), GFP_KERNEL);
	if (!netmdev->vdev->vdev_regions)
		goto alloc_fail;

	/* BAR MMIO */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = pci_resource_start(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	size = pci_resource_len(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	idx = VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR0_REGION_INDEX);
	mdev_net_add_essential(info, idx, size, VFIO_NET_MMIO, VFIO_NET_MDEV_BARS,
			       start);

	/* Rx + Rx free list */
	sparse = kzalloc(pi->nqsets * nr_areas * sizeof(*info->caps.sparse),
			 GFP_KERNEL);
	for (i = 0; i < pi->nqsets; i++) {
		struct sge_rspq *iq = &pi->adapter->sge.ethrxq[i].rspq;
		struct sge_fl *fl = &pi->adapter->sge.ethrxq[i].fl;
		struct sge *s = &pi->adapter->sge;

		start = virt_to_phys(iq->desc);
		size = iq->size * iq->iqe_len;

		info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
		idx = VFIO_PCI_INDEX_TO_OFFSET(cnt + netmdev->vdev->bus_regions);
		mdev_net_add_essential(info, idx, size, VFIO_NET_DESCRIPTORS,
				       VFIO_NET_MDEV_RX, start);
		cnt++;
		/* free list as sparse map */
		start = virt_to_phys(fl->desc);
		size = fl->size * sizeof(*fl->desc) + s->stat_len;

		info->caps.sparse = &sparse[i];
		mdev_net_add_sparse(info, nr_areas, &start, &size);
	}

	/* Tx */
	for (i = 0; i < pi->nqsets; i++) {
		struct sge_txq *q = &pi->adapter->sge.ethtxq[i].q;
		struct sge *s = &pi->adapter->sge;

		info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
		start = virt_to_phys(q->desc);
		size = q->size * sizeof(*q->desc) + s->stat_len;
		idx = VFIO_PCI_INDEX_TO_OFFSET(cnt + netmdev->vdev->bus_regions);
		mdev_net_add_essential(info, idx, size, VFIO_NET_DESCRIPTORS,
				       VFIO_NET_MDEV_TX, start);
		cnt++;
	}

	return 0;

alloc_fail:
	if (sparse)
		kfree(sparse);
	if (netmdev->vdev->vdev_regions)
		kfree(netmdev->vdev->vdev_regions);
	if (netmdev->vdev)
		kfree(netmdev->vdev);

	return -ENOMEM;
}

void cxgb4_destroy_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	if (netmdev->vdev) {
		if (netmdev->vdev->vdev_regions->caps.sparse)
			kfree(netmdev->vdev->vdev_regions->caps.sparse);
		if (netmdev->vdev->vdev_regions)
			kfree(netmdev->vdev->vdev_regions);
		kfree(netmdev->vdev);
	}
}

static int cxgb4_transition_start(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct adapter *adapter = pi->adapter;
	int ret;

	t4_intr_disable(adapter);
	dev_hold(netdev);

	ret = t4_update_port_info(pi);
	if (ret < 0)
		return ret;
	/* XXX Check if we have to free queues to save resources */
	t4_sge_stop(adapter);

	ret = cxgb4_init_vdev(mdev);
	if (ret)
		return -EINVAL;

	return 0;
}

static int cxgb4_transition_back(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct adapter *adapter = pi->adapter;

	dev_put(netdev);
	cxgb4_destroy_vdev(mdev);

	/* Do we need this ??
	 * enable_rx(adap);
	 */
	t4_sge_start(adapter);
	t4_intr_enable(adapter);

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
