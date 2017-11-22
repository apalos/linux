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

#ifndef _I40E_MDEV_H_
#define _I40E_MDEV_H_

#include <linux/device.h>
#include <linux/kconfig.h>

/* TODO: use IF_ENABLED */
#define CONFIG_VFIO_MDEV_NET_DEVICE

#ifdef CONFIG_VFIO_MDEV_NET_DEVICE
void i40e_register_netmdev(struct device *dev);
void i40e_unregister_netmdev(struct device *dev);
#else
#define i40e_register_netmdev(dev)
#define i40e_unregister_netmdev(dev)
#endif /* CONFIG_VFIO_MDEV_NET_DEVICE */

#endif /* _I40E_MDEV_H_ */
