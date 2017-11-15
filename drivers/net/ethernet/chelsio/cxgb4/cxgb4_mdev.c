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

/* per queue */
static ssize_t doorbell_offset_show(struct netdev_queue *queue,
				    struct netdev_queue_attribute *attribute,
				    char *buf)
{
	struct net_device *ndev = queue->dev;
	unsigned int i;

	i = queue - ndev->_tx;
	BUG_ON(i >= ndev->num_tx_queues);

	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}
MDEV_NET_ATTR_RO(doorbell_offset);

static const struct attribute *cxgb4_mdev_attrs[] = {
	&mdev_attr_doorbell_offset.attr,
	NULL,
};

/* per device */
static ssize_t fl_size_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}
DEVICE_ATTR_RO(fl_size);

static const struct attribute *cxgb4_net_device_attrs[] = {
	&dev_attr_fl_size.attr,
	NULL,
};


static int cxgb4_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct port_info *pi = netdev_priv(netdev);
	struct mdev_net_regions *info;
	struct pci_dev *pdev;
	int mmio_flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP;
	int cap_flags = VFIO_REGION_INFO_FLAG_READ |
		VFIO_REGION_INFO_FLAG_WRITE |
		VFIO_REGION_INFO_FLAG_MMAP |
		VFIO_REGION_INFO_FLAG_CAPS;
	phys_addr_t start;
	u64 len;
	int i;
	int cnt = 0;

	pdev = pi->adapter->pdev;

	netmdev->vdev = kzalloc(sizeof(netmdev->vdev), GFP_KERNEL);
	if (!netmdev->vdev)
		return -ENOMEM;

	netmdev->vdev->bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev->extra_regions = VFIO_NET_MDEV_NUM_REGIONS;
	netmdev->vdev->used_regions = 0;

	netmdev->vdev->bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev->num_irqs = 1;

	netmdev->vdev->vdev_regions =
		kzalloc(netmdev->vdev->used_regions *
			sizeof(*netmdev->vdev->vdev_regions), GFP_KERNEL);
	if (!netmdev->vdev->vdev_regions) {
		kfree(netmdev->vdev);
		return -ENOMEM;
	}
	netmdev->vdev->vdev_regions->caps.sparse = kzalloc(pi->nqsets *
			sizeof(*netmdev->vdev->vdev_regions->caps.sparse), GFP_KERNEL);

	/* BAR MMIO */
	info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
	start = pci_resource_start(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	len = pci_resource_len(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR0_REGION_INDEX),
			    len, mmio_flags);
	mdev_net_add_mmap(&info, start, len);

	/* Rx + Rx free list */
	for (i = 0; i < pi->nqsets; i++) {
		struct sge_rspq *iq = &pi->adapter->sge.ethrxq[i].rspq;
		struct sge_fl *fl = &pi->adapter->sge.ethrxq[i].fl;
		struct sge *s = &pi->adapter->sge;

		start = virt_to_phys(iq->desc);
		len = iq->size * iq->iqe_len;

		info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
		mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(cnt +
				    netmdev->vdev->bus_regions), len,
				    cap_flags);
		cnt++;
		mdev_net_add_cap(&info, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_RX);
		mdev_net_add_mmap(&info, start, len);
		/* free list as sparse map */
		start = virt_to_phys(fl->desc);
		len = (fl->size * sizeof(*fl->desc)) + s->stat_len;
		mdev_net_add_sparse(&info, 1, &start, &len);
	}
	/* Tx */
	for (i = 0; i < pi->nqsets; i++) {
		/* Tx */
		struct sge_txq *q = &pi->adapter->sge.ethtxq[i].q;
		struct sge *s = &pi->adapter->sge;

		info = &netmdev->vdev->vdev_regions[netmdev->vdev->used_regions++];
		start = virt_to_phys(q->desc);
		len = q->size * sizeof(*q->desc) + s->stat_len;
		mdev_net_add_region(&info, VFIO_PCI_INDEX_TO_OFFSET(cnt +
				    netmdev->vdev->bus_regions), len,
				    cap_flags);
		cnt++;
		mdev_net_add_cap(&info, VFIO_NET_DESCRIPTORS, VFIO_NET_MDEV_TX);
		mdev_net_add_mmap(&info, start, len);
	}

	return 0;
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

	dev_hold(netdev);

	ret = t4_update_port_info(pi);
	if (ret < 0)
		return ret;
	/* XXX Check if we have to free queues to save resources */
	t4_intr_disable(adapter);
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

	/* XXX Check if we have to free queues to save resources */
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
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct netdev_queue *queue = NULL;
	int ret, i;

	register_device = symbol_get(netmdev_register_device);
	if (!register_device)
		return;

	if (register_device(dev, &cxgb4_netmdev_driver_ops) < 0)
		dev_err(dev, "Could not register device\n");
	else
		dev_info(dev, "Successfully registered net_mdev device\n");

	for (i = 0; i < ndev->num_tx_queues; i++) {
		queue = &ndev->_tx[i];
		ret = sysfs_create_files(&queue->kobj, cxgb4_mdev_attrs);
		if (ret)
			return;
	}
	sysfs_create_files(&ndev->dev.kobj, cxgb4_net_device_attrs);

	symbol_put(netmdev_register_device);
}

void cxgb4_unregister_netmdev(struct device *dev)
{
	int (*unregister_device)(struct device *d);
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = pci_get_drvdata(pdev);
	struct netdev_queue *queue = &ndev->_tx[0];
	int i;

	sysfs_remove_files(&ndev->dev.kobj, cxgb4_net_device_attrs);
	for (i = 0; i < ndev->num_tx_queues; i++) {
		queue = &ndev->_tx[i];
		sysfs_remove_files(&queue->kobj, cxgb4_mdev_attrs);
	}
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
