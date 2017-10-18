#include <linux/init.h>
#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/pci.h>
#include <linux/dmar.h>
#include <linux/dma-mapping.h>
#include <linux/mempool.h>
#include <linux/memory.h>
#include <linux/cpu.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/iova.h>
#include <linux/iommu.h>
#include <linux/intel-iommu.h>
#include <linux/syscore_ops.h>
#include <linux/tboot.h>
#include <linux/dmi.h>
#include <linux/pci-ats.h>
#include <linux/memblock.h>
#include <linux/dma-contiguous.h>
#include <linux/crash_dump.h>

struct mdev_domain {
	/* generic domain data structure for iommu core */
	struct iommu_domain domain;
};

/* Convert generic 'struct iommu_domain to private struct dmar_domain */
static struct mdev_domain *to_mdev_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct mdev_domain, domain);
}

static bool mdev_iommu_capable(enum iommu_cap cap)
{
	if (cap == IOMMU_CAP_CACHE_COHERENCY)
		return true;
	if (cap == IOMMU_CAP_INTR_REMAP)
		return true;

	return false;
}

static struct iommu_domain *mdev_iommu_domain_alloc(unsigned type)
{
	struct mdev_domain *mdev_domain;
	struct iommu_domain *domain;

	pr_info("%s\n", __func__);
	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	mdev_domain = kmalloc(sizeof(struct mdev_domain), GFP_KERNEL);
	if (!mdev_domain) {
		pr_err("Can't allocate mdev_domain\n");
		return NULL;
	}

	domain = &mdev_domain->domain;
	domain->geometry.aperture_start = 0;
	domain->geometry.aperture_end = (1ULL << 49) - 1 ;
	domain->geometry.force_aperture = true;
	pr_info("%s (%d): aperture end=%llx\n ", __func__,
		type, domain->geometry.aperture_end);

	return domain;
}

static void mdev_iommu_domain_free(struct iommu_domain *domain)
{
	struct mdev_domain *mdev_domain;
	mdev_domain = to_mdev_domain(domain);
	/* do some cleanup */
	kfree(mdev_domain);
}

static int mdev_iommu_attach_device(struct iommu_domain *domain,
	struct device *dev)
{
	return 0;
}

static void mdev_iommu_detach_device(struct iommu_domain *domain,
	struct device *dev)
{
	dmar_remove_one_dev_info(to_dmar_domain(domain), dev);
}

const struct iommu_ops mdev_iommu_ops = {
	.capable = mdev_iommu_capable,
	.domain_alloc = mdev_iommu_domain_alloc,
	.domain_free = mdev_iommu_domain_free,
	.attach_dev = mdev_iommu_attach_device,
	.detach_dev = mdev_iommu_detach_device,
	/*
	.map = mdev_iommu_map,
	.unmap = mdev_iommu_unmap,
	.map_sg = default_iommu_map_sg,
	.iova_to_phys = mdev_iommu_iova_to_phys,
	.add_device = mdev_iommu_add_device,
	.remove_device = mdev_iommu_remove_device,
	.get_resv_regions = mdev_iommu_get_resv_regions,
	.put_resv_regions = mdev_iommu_put_resv_regions,
	.device_group = pci_device_group,
	.pgsize_bitmap = mdev_IOMMU_PGSIZES,
	*/
};
