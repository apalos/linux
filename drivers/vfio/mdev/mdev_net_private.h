#ifndef MDEV_NET_PRIVATE_H
#define MDEV_NET_PRIVATE_H


struct netmdev_driver {
	struct device_driver *driver;
	struct netmdev_driver_ops *drv_ops;
};

struct iovamap {
	struct list_head list;
	u64 iova;
	void *cookie;
	struct device *dev;
	u32 size; /* maximum of 32MB */
	enum dma_data_direction direction;
};

struct netmdev {
	union {
		/* kernel visibility only, not part of UAPI*/
		char private[4096 * 2];
		struct {
			struct net_device *netdev;
			struct netmdev_driver_ops drv_ops;
			struct list_head mapping_list_head;
		};
	};
	union {
		/* shadow features & statistics page */
		/* part of UAPI */
		char uapioffset[4096];
		struct netmdev_uapi uapi;
	};
} ;

#endif
