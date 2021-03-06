/*
 * videobuf2-vmalloc.h - vmalloc memory allocator for videobuf2
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <pawel@osciak.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#ifndef _ZYNQ_MALLOC_H
#define _ZYNQ_MALLOC_H

#include <media/videobuf2-core.h>
#include <linux/dma-mapping.h>

struct zynq_malloc_conf {
	unsigned int is_always_get_first_memory; //0: increase getting, 1: always get the first memory
	void *pool_start_address;
	unsigned int pool_size;
};

typedef struct zynq_malloc_conf  zynq_malloc_conf_t; 

extern const struct vb2_mem_ops zynq_malloc_memops;

void *zynq_malloc_init_ctx(zynq_malloc_conf_t *ctx);

void zynq_malloc_cleanup_ctx(void *alloc_ctx);

dma_addr_t  zynq_malloc_plane_dma_addr(struct vb2_buffer *vb, unsigned int plane_no);

#endif
