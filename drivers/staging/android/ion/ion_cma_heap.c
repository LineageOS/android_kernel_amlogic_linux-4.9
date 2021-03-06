/*
 * drivers/staging/android/ion/ion_cma_heap.c
 *
 * Copyright (C) Linaro 2012
 * Author: <benjamin.gaignard@linaro.org> for ST-Ericsson.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#ifdef CONFIG_AMLOGIC_MODIFY
#include <linux/cma.h>
#include <linux/of.h>
#include <linux/highmem.h>
#endif

#include "ion.h"
#include "ion_priv.h"

#define ION_CMA_ALLOCATE_FAILED -1

struct ion_cma_heap {
	struct ion_heap heap;
	struct device *dev;
};

#define to_cma_heap(x) container_of(x, struct ion_cma_heap, heap)

struct ion_cma_buffer_info {
	void *cpu_addr;
	dma_addr_t handle;
	struct sg_table *table;
#ifdef CONFIG_AMLOGIC_MODIFY
	struct page *pages;
#endif
};

#ifdef CONFIG_AMLOGIC_MODIFY
static bool ion_cma_has_kernel_nomapping(struct ion_heap *heap)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);
	struct device *dev = cma_heap->dev;
	struct device_node *mem_region;

	mem_region = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem_region)
		return false;

	return of_property_read_bool(mem_region, "no-kernel-map");
}

/* ION CMA heap operations functions */
static int ion_cma_nomap_allocate(struct ion_heap *heap,
				  struct ion_buffer *buffer,
				  unsigned long len, unsigned long align,
				  unsigned long flags)
{
	int ret;
	struct sg_table *table;
	struct cma *cma = NULL;
	struct page *pages = NULL;
	unsigned long size = PAGE_ALIGN(len);
	unsigned long nr_pages = size >> PAGE_SHIFT;
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info;

	if (buffer->flags & ION_FLAG_CACHED)
		return -EINVAL;

	if (align > PAGE_SIZE)
		return -EINVAL;

	if (!dev->cma_area)
		return -EINVAL;

	cma = dev->cma_area;
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ION_CMA_ALLOCATE_FAILED;

	pages = cma_alloc(cma, nr_pages, align);

	if (!pages)
		goto err;

	if (PageHighMem(pages)) {
		unsigned long nr_clear_pages = nr_pages;
		struct page *page = pages;

		while (nr_clear_pages > 0) {
			void *vaddr = kmap_atomic(page);

			memset(vaddr, 0, PAGE_SIZE);
			kunmap_atomic(vaddr);
			page++;
			nr_clear_pages--;
		}
	} else {
		memset(page_address(pages), 0, size);
	}

	ion_pages_sync_for_device(dev, pages, size, DMA_BIDIRECTIONAL);

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		goto free_mem;

	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret)
		goto free_table;

	sg_set_page(table->sgl, pages, size, 0);

	/* keep this for memory release */
	info->table = table;
	info->pages = pages;
	buffer->priv_virt = info;
	buffer->sg_table = table;
	return 0;

free_table:
	kfree(table);
free_mem:
	cma_release(cma, pages, nr_pages);
err:
	kfree(info);
	return ION_CMA_ALLOCATE_FAILED;
}

static void ion_cma_nomap_free(struct ion_buffer *buffer)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;
	struct page *pages = info->pages;
	unsigned long nr_pages = PAGE_ALIGN(buffer->size) >> PAGE_SHIFT;

	/* release memory */
	cma_release(dev->cma_area, pages, nr_pages);
	/* release sg table */
	sg_free_table(info->table);
	kfree(info->table);
	kfree(info);
}

static struct ion_heap_ops ion_cma_nomap_ops = {
	.allocate = ion_cma_nomap_allocate,
	.free = ion_cma_nomap_free,
	.map_user = ion_heap_map_user,
	.map_kernel = ion_heap_map_kernel,
	.unmap_kernel = ion_heap_unmap_kernel,
};

#endif

/* ION CMA heap operations functions */
static int ion_cma_allocate(struct ion_heap *heap, struct ion_buffer *buffer,
			    unsigned long len, unsigned long align,
			    unsigned long flags)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info;

	if (buffer->flags & ION_FLAG_CACHED)
		return -EINVAL;

	if (align > PAGE_SIZE)
		return -EINVAL;

	info = kzalloc(sizeof(struct ion_cma_buffer_info), GFP_KERNEL);
	if (!info)
		return ION_CMA_ALLOCATE_FAILED;

	info->cpu_addr = dma_alloc_coherent(dev, len, &(info->handle),
						GFP_HIGHUSER | __GFP_ZERO);

	if (!info->cpu_addr) {
		dev_err(dev, "Fail to allocate buffer\n");
		goto err;
	}

	info->table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!info->table)
		goto free_mem;

	if (dma_get_sgtable(dev, info->table, info->cpu_addr, info->handle,
			    len))
		goto free_table;
	/* keep this for memory release */
	buffer->priv_virt = info;
	buffer->sg_table = info->table;
	return 0;

free_table:
	kfree(info->table);
free_mem:
	dma_free_coherent(dev, len, info->cpu_addr, info->handle);
err:
	kfree(info);
	return ION_CMA_ALLOCATE_FAILED;
}

static void ion_cma_free(struct ion_buffer *buffer)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	/* release memory */
	dma_free_coherent(dev, buffer->size, info->cpu_addr, info->handle);
	/* release sg table */
	sg_free_table(info->table);
	kfree(info->table);
	kfree(info);
}

static int ion_cma_mmap(struct ion_heap *mapper, struct ion_buffer *buffer,
			struct vm_area_struct *vma)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(buffer->heap);
	struct device *dev = cma_heap->dev;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	return dma_mmap_coherent(dev, vma, info->cpu_addr, info->handle,
				 buffer->size);
}

static void *ion_cma_map_kernel(struct ion_heap *heap,
				struct ion_buffer *buffer)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;
	/* kernel memory mapping has been done at allocation time */
	return info->cpu_addr;
}

static void ion_cma_unmap_kernel(struct ion_heap *heap,
				 struct ion_buffer *buffer)
{
}

static struct ion_heap_ops ion_cma_ops = {
	.allocate = ion_cma_allocate,
	.free = ion_cma_free,
	.map_user = ion_cma_mmap,
	.map_kernel = ion_cma_map_kernel,
	.unmap_kernel = ion_cma_unmap_kernel,
};

struct ion_heap *ion_cma_heap_create(struct ion_platform_heap *data)
{
	struct ion_cma_heap *cma_heap;

	cma_heap = kzalloc(sizeof(struct ion_cma_heap), GFP_KERNEL);

	if (!cma_heap)
		return ERR_PTR(-ENOMEM);

	cma_heap->heap.ops = &ion_cma_ops;
	/*
	 * get device from private heaps data, later it will be
	 * used to make the link with reserved CMA memory
	 */
	cma_heap->dev = data->priv;
	cma_heap->heap.type = ION_HEAP_TYPE_DMA;
#ifdef CONFIG_AMLOGIC_MODIFY
	if (ion_cma_has_kernel_nomapping(&cma_heap->heap))
		cma_heap->heap.ops = &ion_cma_nomap_ops;
#endif
	return &cma_heap->heap;
}

void ion_cma_heap_destroy(struct ion_heap *heap)
{
	struct ion_cma_heap *cma_heap = to_cma_heap(heap);

	kfree(cma_heap);
}
