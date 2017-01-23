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

#ifndef _UVC_KMALLOC_H
#define _UVC_KMALLOC_H

#include <media/videobuf2-core.h>
#include <linux/dma-mapping.h>

struct uvc_kmalloc_conf {
	unsigned int is_always_get_first_memory; //0: increase getting, 1: always get the first memory
	void *buffer_virt_addr_list;
	unsigned int buffer_num;
	unsigned int available_buffer_size;
	void *context;
	unsigned int en_non_cache_map;
};

typedef struct uvc_kmalloc_conf  uvc_kmalloc_conf_t;

extern const struct vb2_mem_ops uvc_kmalloc_memops;

void *uvc_kmalloc_init_ctx(uvc_kmalloc_conf_t *ctx);

void uvc_kmalloc_cleanup_ctx(void *alloc_ctx);

#endif
