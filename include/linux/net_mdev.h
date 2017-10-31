#ifndef NET_MDEV_H
#define NET_MDEV_H

#include <linux/vfio.h>
#include <uapi/linux/net_mdev.h>
/* helper macros copied from vfio-pci */
#define VFIO_PCI_OFFSET_SHIFT   40
#define VFIO_PCI_OFFSET_TO_INDEX(off)   (off >> VFIO_PCI_OFFSET_SHIFT)
#define VFIO_PCI_INDEX_TO_OFFSET(index) ((u64)(index) << VFIO_PCI_OFFSET_SHIFT)
#define VFIO_PCI_OFFSET_MASK    (((u64)(1) << VFIO_PCI_OFFSET_SHIFT) - 1)

struct mdev_device;
struct device;

/**
 * struct netmdev_driver_ops - Structure to be registered for each mdev net
 * device.
 *
 * register the device to mdev module
 * @transition_start: called on mediated device init
 * @transition_complete: called when mediated device is ready
 * @transition_back: called when mediated device control back to kernel
 * @get_region_info: get region information
 **/
struct net_mdev_bus_info {
	__u32 bus_max;
	__u32 extra;
};

struct netmdev_driver_ops {
	int (*transition_start)(struct net_device *ndev);
	int (*transition_back)(struct net_device *ndev);
	int (*get_mmap_info)(struct net_device* netdev, u32 index,
			     unsigned long *pfn, unsigned long *nr_pages);
	int (*get_region_info)(struct net_device *ndev,
			       struct vfio_region_info *info);
	int (*get_cap_info)(struct net_device *ndev, u32 region,
			    struct vfio_region_info_cap_type *cap_type,
			    struct vfio_region_info *info);
	int (*get_device_info)(struct net_device *ndev,
			       struct vfio_device_info *info);
	int (*get_irq_info)(struct net_device *ndev,
			       struct vfio_irq_info *info);
	int (*reset_dev)(struct net_device *ndev);
	int (*get_bus_info)(struct net_device *ndev,
			       struct net_mdev_bus_info *bus_info);
};

int netmdev_register_device(struct device* dev, struct netmdev_driver_ops *ops);
int netmdev_unregister_device(struct device* dev);

#endif /* MDEV_H */
