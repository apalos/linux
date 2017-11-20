#ifndef NET_MDEV_H
#define NET_MDEV_H

#include <linux/vfio.h>
#include <linux/netdevice.h>
#include <uapi/linux/net_mdev.h>

/* helper macros copied from vfio-pci */
#define VFIO_PCI_OFFSET_SHIFT   40
#define VFIO_PCI_OFFSET_TO_INDEX(off)   (off >> VFIO_PCI_OFFSET_SHIFT)
#define VFIO_PCI_INDEX_TO_OFFSET(index) ((u64)(index) << VFIO_PCI_OFFSET_SHIFT)
#define VFIO_PCI_OFFSET_MASK    (((u64)(1) << VFIO_PCI_OFFSET_SHIFT) - 1)

/* TODO: add net_device->sysfs_tx_queue_group and cleanup this */
struct netdev_queue_attribute {
	struct attribute attr;
	ssize_t (*show)(struct netdev_queue *queue,
	    struct netdev_queue_attribute *attr, char *buf);
	ssize_t (*store)(struct netdev_queue *queue,
	    struct netdev_queue_attribute *attr, const char *buf, size_t len);
};

struct mdev_device;
struct device;

struct mdev_net_sparse {
	__u64 offset;
	unsigned long pfn;
	unsigned long nr_pages;
};

struct mdev_net_caps {
	__u32 type;     /* global per bus driver */
	__u32 subtype;  /* type specific */
	__u32 nr_areas; /* number of sparse areas */
	struct mdev_net_sparse *sparse;
};

struct mdev_net_region {
	__u32	flags;
	__u64	offset;		/* Region offset */
	unsigned long pfn;
	unsigned long nr_pages;
	struct mdev_net_caps caps;
};

struct mdev_net_vdev {
	__u8 bus_regions;	/* Bus specific */
	__u8 extra_regions;	/* extra regions */
	__u8 used_regions;	/* Used regions/caps */
	__u32 bus_flags;	/* vfio_device_info flags */
	__u16 num_irqs;		/* Max IRQ index + 1 */
	struct mdev_net_region *regions;
};

/**
 * struct netmdev_driver_ops - Structure to be registered for each mdev net
 * device.
 *
 * register the device to mdev module
 * @transition_start: called on mediated device init
 * @transition_complete: called when mediated device is ready
 * @reset_dev: reset the device
 **/

struct netmdev_driver_ops {
	int (*transition_start)(struct mdev_device *mdev);
	int (*transition_back)(struct mdev_device *mdev);
	int (*reset_dev)(struct mdev_device *mdev);
};

struct netmdev_driver {
	struct device_driver *driver;
	struct netmdev_driver_ops *drv_ops;
};

struct netmdev {
	struct mdev_net_vdev *vdev;
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

int netmdev_register_device(struct device* dev, struct netmdev_driver_ops *ops);
int netmdev_unregister_device(struct device* dev);
struct net_device *mdev_get_netdev(struct mdev_device *mdev);
int mdev_net_add_sparse(struct mdev_net_region *region, __u64 offset,
			unsigned long pfn, unsigned long nr_pages);
void mdev_net_add_essential(struct mdev_net_region *region, __u32 type,
			    __u32 subtype, __u64 offset, unsigned long pfn,
			    unsigned long nr_pages);

#endif /* MDEV_H */
