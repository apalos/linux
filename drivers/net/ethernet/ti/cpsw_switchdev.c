/*
 * Texas Instruments switchdev Driver
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <net/switchdev.h>
#include "cpsw.h"
#include "cpsw_ale.h"

static inline void slave_write(struct cpsw_slave *slave, u32 val, u32 offset)
{
	writel_relaxed(val, slave->regs + offset);
}

static u32 cpsw_switchdev_get_ver(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	struct cpsw_common *cpsw = priv->cpsw;

	return cpsw->version;
}


static int cpsw_port_attr_get(struct net_device *dev,
			      struct switchdev_attr *attr)
{
	int err = 0;
	uint32_t cpsw_ver;

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_PARENT_ID:
		cpsw_ver = cpsw_switchdev_get_ver(dev);
		attr->u.ppid.id_len = sizeof(cpsw_ver);
		memcpy(&attr->u.ppid.id, &cpsw_ver, attr->u.ppid.id_len);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}

static int cpsw_port_vlan_add(struct cpsw_priv *priv, u16 vid, bool untagged,
			      bool pvid)
{
	int ret = 0;
	int unreg_mcast_mask = 0;
	struct cpsw_common *cpsw = priv->cpsw;
	void __iomem *port_vlan_reg;
	int port_mask, port_vlan;
	int untag_mask = 0;
	int reg = CPSW2_PORT_VLAN;

	port_mask = BIT(priv->emac_port + 1) | ALE_PORT_HOST;
	/* add ports that already participate */
	port_mask |= cpsw_ale_read_vlan_members(cpsw->ale, vid);

	if (priv->ndev->flags & IFF_ALLMULTI)
		unreg_mcast_mask = port_mask;

	if (untagged) {
		untag_mask = port_mask & ~ALE_PORT_HOST;
		/* add ports that are arelady untagged */
		untag_mask |= cpsw_ale_read_untagged(cpsw->ale, vid);
	} else {
		untag_mask = cpsw_ale_read_untagged(cpsw->ale, vid);
		untag_mask &= ~BIT(priv->emac_port + 1);
	}
	dev_dbg(priv->dev, "port mask 0x%x untagged 0x%x\n", port_mask,
		untag_mask);
	ret = cpsw_ale_add_vlan(cpsw->ale, vid, port_mask, untag_mask,
				port_mask, unreg_mcast_mask);
	if (ret) {
		dev_err(priv->dev, "Unable to add vlan\n");
		return ret;
	}

	if (!pvid)
		return ret;

	/* add proper CFI and CoS */
	port_vlan = vid;


	if (cpsw->version == CPSW_VERSION_1)
		reg = CPSW1_PORT_VLAN;
	/* no barrier ?? */
	slave_write(cpsw->slaves + priv->emac_port, port_vlan, reg);
	/* CPU port */
	port_vlan_reg = &cpsw->host_port_regs->port_vlan;
	writel(port_vlan, port_vlan_reg);

	dev_info(priv->dev, "VID: %u dev: %s port: %u\n", vid,
		 priv->ndev->name, priv->emac_port + 1);

	return ret;
}

static int cpsw_port_vlan_del(struct cpsw_priv *priv, u16 vid)
{
	int ret = 0;
	struct cpsw_common *cpsw = priv->cpsw;
	int vlan_members, port_mask;

	port_mask = BIT(priv->emac_port + 1);
	vlan_members = cpsw_ale_read_vlan_members(cpsw->ale, vid);

	/* keep CPU port if we have members patcicipating in that VLAN */
	if (vlan_members == (port_mask | ALE_PORT_HOST))
		port_mask |= ALE_PORT_HOST;

	port_mask = vlan_members & ~port_mask;

	dev_dbg(priv->dev, "port mask 0x%x\n", port_mask);

	ret = cpsw_ale_del_vlan(cpsw->ale, vid, port_mask);
	if (ret != 0)
		return ret;

	ret = cpsw_ale_del_ucast(cpsw->ale, priv->mac_addr,
				 HOST_PORT_NUM, ALE_VLAN, vid);
	if (ret != 0)
		return ret;

	ret = cpsw_ale_del_mcast(cpsw->ale, priv->ndev->broadcast,
				 port_mask, ALE_VLAN, vid);

	return ret;

}

static int cpsw_port_vlans_add(struct cpsw_priv *priv,
			       const struct switchdev_obj_port_vlan *vlan,
			       struct switchdev_trans *trans)
{
	u16 vid;
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;

	if (switchdev_trans_ph_prepare(trans))
		return 0;

	for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {
		int err;

		err = cpsw_port_vlan_add(priv, vid, untagged, pvid);
		if (err)
			return err;
	}

	return 0;
}

static int cpsw_port_vlans_del(struct cpsw_priv *priv,
			       const struct switchdev_obj_port_vlan *vlan)
{
	u16 vid;

	for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {
		int err;

		err = cpsw_port_vlan_del(priv, vid);
		if (err)
			return err;
	}

	return 0;
}

static int cpsw_port_mdb_add(struct cpsw_priv *priv,
			     struct switchdev_obj_port_mdb *mdb,
			     struct switchdev_trans *trans)
{
	int err;
	struct cpsw_common *cpsw = priv->cpsw;
	int port_mask;

	if (switchdev_trans_ph_prepare(trans))
		return 0;

	port_mask = BIT(priv->emac_port + 1) | ALE_PORT_HOST;
	/* add ports that already participate */
	port_mask |= cpsw_ale_read_maddr(cpsw->ale, mdb->addr, ALE_VLAN,
					 mdb->vid);
	err = cpsw_ale_add_mcast(cpsw->ale, mdb->addr,
				 port_mask, ALE_VLAN, mdb->vid, 0);

	return err;
}

static int cpsw_port_mdb_del(struct cpsw_priv *priv,
			     struct switchdev_obj_port_mdb *mdb)

{
	int err;
	struct cpsw_common *cpsw = priv->cpsw;
	int port_mask, del_mask;

	del_mask = BIT(priv->emac_port + 1);
	/* add ports that already participate */
	port_mask = cpsw_ale_read_maddr(cpsw->ale, mdb->addr, ALE_VLAN,
					mdb->vid);

	if (port_mask == (del_mask | ALE_PORT_HOST))
		del_mask |= ALE_PORT_HOST;
	port_mask &= ~del_mask;

	err = cpsw_ale_del_mcast(cpsw->ale, mdb->addr,
				 port_mask, ALE_VLAN, mdb->vid);

	return err;
}

static int cpsw_port_obj_add(struct net_device *ndev,
			     const struct switchdev_obj *obj,
			     struct switchdev_trans *trans)
{
	int err = 0;
	struct switchdev_obj_port_vlan *vlan = SWITCHDEV_OBJ_PORT_VLAN(obj);
	struct switchdev_obj_port_mdb *mdb = SWITCHDEV_OBJ_PORT_MDB(obj);
	struct cpsw_priv *priv = netdev_priv(ndev);

	switch (obj->id) {
	case SWITCHDEV_OBJ_ID_PORT_VLAN:
		cpsw_port_vlans_add(priv, vlan, trans);
		break;
	case SWITCHDEV_OBJ_ID_PORT_MDB:
		err = cpsw_port_mdb_add(priv, mdb, trans);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	return err;
}

static int cpsw_port_obj_del(struct net_device *ndev,
			     const struct switchdev_obj *obj)

{
	int err = 0;
	struct switchdev_obj_port_vlan *vlan = SWITCHDEV_OBJ_PORT_VLAN(obj);
	struct cpsw_priv *priv = netdev_priv(ndev);

	switch (obj->id) {
	case SWITCHDEV_OBJ_ID_PORT_VLAN:
		cpsw_port_vlans_del(priv, vlan);
		break;
	case SWITCHDEV_OBJ_ID_PORT_MDB:
		err = cpsw_port_mdb_del(priv, SWITCHDEV_OBJ_PORT_MDB(obj));
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	return err;
}

static const struct switchdev_ops cpsw_port_switchdev_ops = {
	.switchdev_port_attr_get	= cpsw_port_attr_get,
	.switchdev_port_attr_set	= NULL,
	.switchdev_port_obj_add		= cpsw_port_obj_add,
	.switchdev_port_obj_del		= cpsw_port_obj_del,
};

void cpsw_port_switchdev_init(struct net_device *ndev)
{
	ndev->switchdev_ops = &cpsw_port_switchdev_ops;
}
