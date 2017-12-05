/*
 * VFIO API definition
 *
 * Copyright (C) 2012 Red Hat, Inc.  All rights reserved.
 *     Author: Alex Williamson <alex.williamson@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _UAPI__LINUX_NET_MDEV_H
#define _UAPI__LINUX_NET_MDEV_H
#include <linux/types.h>
#include <linux/ioctl.h>

/* find a way to define this IOCTL properly */
#define VFIO_NETMDEV_TRANSITION_COMPLETE	500

enum net_mdev_types {
	VFIO_NET_MDEV_SHADOW,
	VFIO_NET_DESCRIPTORS,
	VFIO_NET_MMIO,
};

/* FIXME split subtypes properly */
enum net_mdev_subtypes {
	VFIO_NET_MDEV_STATS,
	VFIO_NET_MDEV_RX,
	VFIO_NET_MDEV_TX,
	VFIO_NET_MDEV_BARS,
};

enum vfio_net_mdev_regions {
	VFIO_NET_MDEV_SHADOW_REGION_INDEX,
	VFIO_NET_MDEV_RX_REGION_INDEX,
	VFIO_NET_MDEV_TX_REGION_INDEX,
	VFIO_NET_MDEV_NUM_REGIONS,
};

/* We might need access to some of these
netdev_features_t	features;
netdev_features_t	hw_features;
netdev_features_t	wanted_features;
netdev_features_t	vlan_features;
netdev_features_t	hw_enc_features;
netdev_features_t	mpls_features;
netdev_features_t	gso_partial_features;
atomic_long_t		rx_dropped;
atomic_long_t		tx_dropped;
atomic_long_t		rx_nohandler;
struct net_device_stats	stats;
*/

#endif /* _UAPI__LINUX_NET_MDEV_H */
