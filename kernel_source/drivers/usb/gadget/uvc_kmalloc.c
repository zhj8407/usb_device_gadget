/*
 * uvc_static_malloc.c - static memory allocator for uvc
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <pawel@osciak.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-memops.h>

#include "uvc_kmalloc.h"

unsigned int en_non_cache_map = 1;

#define MAX_BUFFER_NUM 1024

struct uvc_mem_pool {
	void **buffer_virt_addr_list;
	unsigned int buffer_in_used[MAX_BUFFER_NUM];
	unsigned int buffer_num;
	unsigned int available_buffer_size;
	struct mutex lock;
};

struct uvc_mem_pool_ctx {
	struct uvc_mem_pool  *mem_pool;
	atomic_t	bInitialMemPool;
};

static int  pool_create(struct uvc_kmalloc_conf *conf)
{
	struct uvc_mem_pool_ctx *pool_ctx = NULL;
	struct uvc_mem_pool *p = NULL;
	unsigned int i  = 0;

	if (conf == NULL) return  -1;

	pool_ctx = (struct uvc_mem_pool_ctx *)conf->context;

	if (pool_ctx == NULL) return  -1;

	p = pool_ctx->mem_pool;

	if (p == NULL) return  -1;

	mutex_init(&p->lock);

	p->buffer_virt_addr_list = (void **)conf->buffer_virt_addr_list;
	p->buffer_num = conf->buffer_num;
	p->available_buffer_size = conf->available_buffer_size;

	if (p->buffer_num > MAX_BUFFER_NUM) {
		printk(KERN_INFO"[uvc_kmalloc](%d) Requested buffer num %u > %u !!\n", __LINE__, (unsigned int)p->buffer_num, (unsigned int)MAX_BUFFER_NUM);
		return -1;
	}

	mutex_lock(&p->lock);

	for (i = 0 ; i  < p->buffer_num; i++) {
		p->buffer_in_used[i] = 0;
	}

	mutex_unlock(&p->lock);

	return 0;
}

static void pool_destroy(struct uvc_kmalloc_conf *conf)
{
	struct uvc_mem_pool_ctx *pool_ctx = (struct uvc_mem_pool_ctx *)conf->context;
	struct uvc_mem_pool *p = pool_ctx->mem_pool;

	if (!p) return;

	mutex_destroy(&p->lock);
	return;
}

static void * pool_alloc(unsigned long *size,
	phys_addr_t *phys_addr,
	unsigned int *index,
	struct uvc_kmalloc_conf *conf)
{
	int i = 0;
	struct uvc_mem_pool_ctx *pool_ctx = (struct uvc_mem_pool_ctx *)conf->context;
	struct uvc_mem_pool *p = pool_ctx->mem_pool;
	void *mem = NULL;
	unsigned int padd_size = (*size + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1)); //PAGE_ALIGN( *size + PAGE_SIZE);

	*index  = (unsigned int) - 1;

	if (conf->available_buffer_size < padd_size) {
		printk(KERN_INFO"[uvc_kmalloc](%d) call pool_alloc failed!! (available size: %u < need size: %u)\n", __LINE__, conf->available_buffer_size, padd_size);
		return NULL;
	}

	*size = padd_size;

	if (conf->is_always_get_first_memory) {
		mem = (void *)p->buffer_virt_addr_list[0];
		*index = 0;
	} else {
		mutex_lock(&p->lock);

		for (i = 0; i < p->buffer_num; i++) {
			if (p->buffer_in_used[i] == 0) {
				mem = (void *)p->buffer_virt_addr_list[i];
				*phys_addr = virt_to_phys(mem);
				*index = i;
				p->buffer_in_used[i] = 1;
				mutex_unlock(&p->lock);
				goto exit;
			}
		}

		mutex_unlock(&p->lock);
	}

exit:
	return mem;
}

struct vb2_kmalloc_buf {
	void				*vaddr;
	phys_addr_t			phys_addr;
	unsigned long			size;
	atomic_t			refcount;
	struct vb2_vmarea_handler	handler;
	unsigned int index;
	unsigned int available_buffer_size;
	struct uvc_mem_pool  *pool;
	unsigned int en_non_cache_map;
};

static void vb2_kmalloc_put(void *buf_priv);

static void *vb2_kmalloc_alloc(void *alloc_ctx, unsigned long size, gfp_t gfp_flags)
{

	struct uvc_kmalloc_conf *conf = alloc_ctx;
	struct vb2_kmalloc_buf *buf = NULL;
	struct uvc_mem_pool_ctx *pool_ctx = (struct uvc_mem_pool_ctx *)conf->context;

	buf = kzalloc(sizeof(struct vb2_kmalloc_buf), GFP_KERNEL | gfp_flags);

	if (!buf) {
		return NULL;
	}

	if (atomic_inc_and_test(&(pool_ctx->bInitialMemPool))) {
		if (pool_create(conf) != 0) {
			printk(KERN_INFO"[uvc_kmalloc](%d)call pool_create failed!!\n", __LINE__);
			return NULL;
		}

		//printk(KERN_INFO"[uvc_kmalloc](%d)\n",__LINE__);
	}

	buf->en_non_cache_map = conf->en_non_cache_map;
	buf->available_buffer_size = conf->available_buffer_size;
	buf->size = size;

	buf->vaddr = pool_alloc(&buf->size, &buf->phys_addr, &buf->index, conf);

	if ((buf->vaddr == NULL) || (buf->index == -1)) {
		printk(KERN_ERR "[uvc_kmalloc]xxx failed to allocate memory"
			"throug the memory pool!! (vaddr=%p, phys_addr=%p, index=%d)\n",
			(void *)buf->vaddr, (void *)buf->phys_addr, (int)buf->index);

		if (buf) {
			kfree(buf);
			buf = NULL;
		}

		return  NULL;
	}

	atomic_set(&(buf->refcount), 0);
	buf->handler.refcount = &buf->refcount;
	buf->handler.put = vb2_kmalloc_put;
	buf->handler.arg = buf;
	buf->pool = pool_ctx->mem_pool;
	atomic_inc(&buf->refcount);
	return buf;
}

static void vb2_kmalloc_put(void *buf_priv)
{
	struct vb2_kmalloc_buf *buf = buf_priv;
	struct uvc_mem_pool *p = NULL;

	if (!buf) {
		return;
	}

	p = buf->pool;

	if (!p) {
		return;
	}

	if (atomic_dec_and_test(&buf->refcount)) {
		mutex_lock(&p->lock);
		p->buffer_in_used[buf->index] = 0;

		if (buf) {
			kfree(buf);
			buf = NULL;
		}

		mutex_unlock(&p->lock);
	}
}


static void *vb2_kmalloc_get_userptr(void *alloc_ctx, unsigned long vaddr,
									 unsigned long size, int write)
{
	/* TODO. Add implementation of get_userptr callback. */
	return NULL;
}

static void vb2_kmalloc_put_userptr(void *buf_priv)
{
	/* TODO. Add implementation of put_userptr callback. */
	return;
}

static void *vb2_kmalloc_vaddr(void *buf_priv)
{

	struct vb2_kmalloc_buf *buf = buf_priv;

	if (!buf) 	 {
		printk(KERN_INFO"[uvc_kmalloc](%d) %p is NULL!!\n", __LINE__, buf);
		return NULL;
	}

	if (!buf->vaddr) {
		printk(KERN_INFO"[uvc_kmalloc](%d) Address of an unallocated plane requested "
			   "or cannot map user pointer\n", __LINE__);
		return NULL;
	}

	return buf->vaddr;
}

static unsigned int vb2_kmalloc_num_users(void *buf_priv)
{
	struct vb2_kmalloc_buf *buf = buf_priv;

	if (!buf) 	 {
		printk(KERN_INFO"[uvc_kmalloc](%d) %p is NULL!!\n", __LINE__, buf);
		return 0;
	}

	return atomic_read(&buf->refcount);
}


static int vb2_kmalloc_mmap(void *buf_priv, struct vm_area_struct *vma)
{

	struct vb2_kmalloc_buf *buf = buf_priv;
	/* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
	unsigned long	region_offset = /* vma->vm_pgoff */ 0 * PAGE_SIZE;
	unsigned long	region_length = 0;
	unsigned long	physical_addr = 0;
	unsigned long	user_virtaddr = 0;

	if (!buf || !vma) 	 {
		printk(KERN_INFO"[uvc_kmalloc](%d) buf(%p) is NULL or vma(%p) is NULL!!\n", __LINE__, buf, vma);
		return -EAGAIN;
	}

	region_length = vma->vm_end - vma->vm_start;
	user_virtaddr = vma->vm_start;
    physical_addr = (unsigned long)(buf->phys_addr + region_offset);

	if (en_non_cache_map || buf->en_non_cache_map) {
		printk(KERN_INFO"[uvc_kmalloc](%d) map  the vma pages as non cached!!\n", __LINE__);
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	}

	// invoke kernel function that sets up the page-table entries
	if (remap_pfn_range(vma,
						user_virtaddr,
						physical_addr >> PAGE_SHIFT,
						region_length,
						vma->vm_page_prot))
		return -EAGAIN;

	/*
	 * Use common vm_area operations to track buffer refcount.
	 */
	vma->vm_private_data = &buf->handler;
	vma->vm_ops = &vb2_common_vm_ops;

	vma->vm_ops->open(vma);

	return 0;
}

/*********************************************/
/*       callbacks for DMABUF buffers        */
/*********************************************/

static int vb2_kmalloc_map_dmabuf(void *mem_priv)
{
	return  -EFAULT;
}

static void vb2_kmalloc_unmap_dmabuf(void *mem_priv)
{
	return;
}

static void vb2_kmalloc_detach_dmabuf(void *mem_priv)
{
	return;
}

static void *vb2_kmalloc_attach_dmabuf(void *alloc_ctx, struct dma_buf *dbuf,
									   unsigned long size, int write)
{
	return NULL;
}

/*
V4L2_MEMORY_MMAP
V4L2_MEMORY_USERPTR
V4L2_MEMORY_DMABUF
 */
const struct vb2_mem_ops uvc_kmalloc_memops = {
	.alloc		= vb2_kmalloc_alloc,
	.put		= vb2_kmalloc_put,
	.get_userptr	= vb2_kmalloc_get_userptr,
	.put_userptr	= vb2_kmalloc_put_userptr,
	.map_dmabuf	= vb2_kmalloc_map_dmabuf,
	.unmap_dmabuf	= vb2_kmalloc_unmap_dmabuf,
	.attach_dmabuf	= vb2_kmalloc_attach_dmabuf,
	.detach_dmabuf	= vb2_kmalloc_detach_dmabuf,
	.vaddr		= vb2_kmalloc_vaddr,
	.mmap		= vb2_kmalloc_mmap,
	.num_users	= vb2_kmalloc_num_users,
};

void *uvc_kmalloc_init_ctx(uvc_kmalloc_conf_t *ctx)
{
	struct uvc_mem_pool_ctx *pool_ctx = NULL;
	struct uvc_kmalloc_conf *conf = NULL;

	if (!ctx) {
		printk(KERN_INFO"[uvc_kmalloc](%d) ctx (%p)  is NULL!!\n", __LINE__, ctx);
		goto exit;
	}

	pool_ctx = kzalloc(sizeof(struct uvc_mem_pool_ctx), GFP_KERNEL);

	if (!pool_ctx) {
		printk(KERN_INFO"[uvc_kmalloc](%d) pool_ctx (%p)  is NULL!!\n", __LINE__, pool_ctx);
		goto exit;
	}

	pool_ctx->mem_pool = kzalloc(sizeof(struct uvc_mem_pool), GFP_KERNEL);

	if (!pool_ctx->mem_pool) {
		printk(KERN_INFO"[uvc_kmalloc](%d) conf (%p) or mem_pool(%p)  is NULL!!\n", __LINE__, conf, pool_ctx->mem_pool);
		goto exit;
	}

	conf = kzalloc(sizeof(struct uvc_kmalloc_conf), GFP_KERNEL);

	if (!conf) {
		printk(KERN_INFO"[uvc_kmalloc](%d) conf (%p) is NULL \n", __LINE__, conf);
		goto exit;
	}

	conf->is_always_get_first_memory = ctx->is_always_get_first_memory;
	conf->buffer_virt_addr_list = ctx->buffer_virt_addr_list;
	conf->buffer_num = ctx->buffer_num;
	conf->available_buffer_size = ctx->available_buffer_size;
	conf->context = (void *)pool_ctx;
	conf->en_non_cache_map = ctx->en_non_cache_map;

	//printk(KERN_INFO"[uvc_kmalloc]jefff  memory pool address : 0x%p\n", pool_ctx->mem_pool);

	atomic_set(&(pool_ctx->bInitialMemPool), -1);

	return conf;

exit:

	if (pool_ctx) {
		if (pool_ctx->mem_pool) {
			kfree(pool_ctx->mem_pool);
			pool_ctx->mem_pool  = NULL;
		}

		kfree(pool_ctx);
		pool_ctx = NULL;
	}

	if (conf) {
		kfree(conf);
		conf = NULL;
	}

	return ERR_PTR(-ENOMEM);

}

void uvc_kmalloc_cleanup_ctx(void *alloc_ctx)
{
	struct uvc_kmalloc_conf *conf = (struct uvc_kmalloc_conf *)alloc_ctx;
	struct uvc_mem_pool_ctx *pool_ctx = NULL;

	if (!conf) return;

	pool_ctx = (struct uvc_mem_pool_ctx *)conf->context;

	if (pool_ctx) {
		atomic_dec(&(pool_ctx->bInitialMemPool));

		if (pool_ctx->mem_pool) {
			pool_destroy(conf);
			kfree(pool_ctx->mem_pool);
			pool_ctx->mem_pool  = NULL;
		}

		kfree(pool_ctx);
		pool_ctx = NULL;
	}

	if (conf) {
		kfree(conf);
		conf = NULL;
	}
}

