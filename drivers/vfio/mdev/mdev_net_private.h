#ifndef MDEV_NET_PRIVATE_H
#define MDEV_NET_PRIVATE_H

struct iovamap {
	struct list_head list;
	u64 iova;
	void *cookie;
	struct device *dev;
	u32 size; /* maximum of 32MB */
	enum dma_data_direction direction;
};

#endif
