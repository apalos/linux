/*******************************************************************************
 *
 * Intel Ethernet Controller XL710 Family Linux Driver
 * Copyright(c) 2017 Linaro Limited.
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
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 ******************************************************************************/

#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mm.h>
#include <linux/vfio.h>
#include <linux/mdev.h>
#include <linux/net_mdev.h>

#include "i40e.h"
#include "i40e_mdev.h"

static void i40e_destroy_vdev(struct mdev_device *mdev);

static int i40e_init_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct i40e_netdev_priv *np = netdev_priv(netdev);
	struct i40e_vsi *vsi = np->vsi;
	struct pci_dev *pdev = vsi->back->pdev;
	struct mdev_net_region *region;
	int i;
	int alloc_regions;
	phys_addr_t start;
	u64 size, offset;
	int offset_cnt;

	netmdev->vdev.bus_regions = VFIO_PCI_NUM_REGIONS;
	netmdev->vdev.extra_regions = 2 * vsi->num_queue_pairs;
	alloc_regions = 2 * vsi->num_queue_pairs + 1;

	netmdev->vdev.bus_flags = VFIO_DEVICE_FLAGS_PCI;
	netmdev->vdev.num_irqs = 1;

	netmdev->vdev.regions = kcalloc(alloc_regions,
					sizeof(*netmdev->vdev.regions), GFP_KERNEL);
	if (!netmdev->vdev.regions)
		goto err;

	region = netmdev->vdev.regions;

	/* MMIO */
	start = pci_resource_start(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	size = pci_resource_len(pdev, VFIO_PCI_BAR0_REGION_INDEX);
	offset = VFIO_PCI_INDEX_TO_OFFSET(VFIO_PCI_BAR0_REGION_INDEX);
	mdev_net_add_essential(region++, VFIO_NET_MMIO, VFIO_NET_MDEV_BARS,
			       offset, start >> PAGE_SHIFT, size >> PAGE_SHIFT);

	offset_cnt = netmdev->vdev.bus_regions;

	/* Rx */
	for (i = 0; i < vsi->num_queue_pairs; i++) {
		struct i40e_ring *rxq = vsi->rx_rings[i];

		start = virt_to_phys(rxq->desc);
		size = PAGE_ALIGN(rxq->size);
		offset = VFIO_PCI_INDEX_TO_OFFSET(offset_cnt++);
		mdev_net_add_essential(region++, VFIO_NET_DESCRIPTORS,
				       VFIO_NET_MDEV_RX, offset,
				       start >> PAGE_SHIFT, size >> PAGE_SHIFT);
	}

	/* Tx */
	for (i = 0; i < vsi->num_queue_pairs; i++) {
		struct i40e_ring *txq = vsi->tx_rings[i];

		start = virt_to_phys(txq->desc);
		size = PAGE_ALIGN(txq->size);
		offset = VFIO_PCI_INDEX_TO_OFFSET(offset_cnt++);
		mdev_net_add_essential(region++, VFIO_NET_DESCRIPTORS,
				       VFIO_NET_MDEV_TX, offset,
				       start >> PAGE_SHIFT, size >> PAGE_SHIFT);
	}

	netmdev->vdev.used_regions = region - netmdev->vdev.regions;
	BUG_ON(netmdev->vdev.used_regions != alloc_regions);

	return 0;

err:
	i40e_destroy_vdev(mdev);
	return -EFAULT;
}

static void i40e_destroy_vdev(struct mdev_device *mdev)
{
	struct netmdev *netmdev = mdev_get_drvdata(mdev);

	kfree(netmdev->vdev.regions);
}

static int i40e_transition_start(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct i40e_netdev_priv *np = netdev_priv(netdev);
	struct i40e_vsi *vsi = np->vsi;

	dev_hold(netdev);

#if 0
	if (netif_running(netdev)) {
		set_bit(__I40E_VSI_REINIT_REQUESTED, vsi->state);
		i40e_do_reset(vsi->back, BIT_ULL(__I40E_REINIT_REQUESTED),
			      false);
	}
#endif

	if (i40e_init_vdev(mdev)) {
		dev_put(netdev);
		return -EINVAL;
	}

	return 0;
}

static int i40e_transition_back(struct mdev_device *mdev)
{
	struct net_device *netdev = mdev_get_netdev(mdev);
	struct i40e_netdev_priv *np = netdev_priv(netdev);
	struct i40e_vsi *vsi = np->vsi;

	i40e_destroy_vdev(mdev);

#if 0
	if (netif_running(netdev)) {
		set_bit(__I40E_VSI_REINIT_REQUESTED, vsi->state);
		i40e_do_reset(vsi->back, BIT_ULL(__I40E_REINIT_REQUESTED),
			      false);
	}
#endif

	dev_put(netdev);

	return 0;
}

static struct netmdev_driver_ops i40e_netmdev_driver_ops = {
	.transition_start = i40e_transition_start,
	.transition_back = i40e_transition_back,
};

void i40e_register_netmdev(struct device *dev)
{
	int (*register_device)(struct device *d,
			       struct netmdev_driver_ops *ops);

	register_device = symbol_get(netmdev_register_device);
	if (!register_device)
		return;

	if (register_device(dev, &i40e_netmdev_driver_ops) < 0)
		dev_err(dev, "Could not register device\n");
	else
		dev_info(dev, "Successfully registered net_mdev device\n");

	symbol_put(netmdev_register_device);
}

void i40e_unregister_netmdev(struct device *dev)
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
