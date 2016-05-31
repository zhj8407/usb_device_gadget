/*
 * zynq_malloc.c - malloc memory allocator for zynq capturing
 *
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-memops.h>
#include "zynq_malloc.h"

extern unsigned int en_use_dev_mem_map;
extern unsigned int en_non_cache_map;
//static struct vm_area_struct  g_vma;

///////////////////////////////////////////////////////////////////////


#define MAX_BUFFER_NUM 1024

struct zynq_mem_pool {
	void **buffer_virt_addr_list;
	dma_addr_t  *buffer_phy_addr_list;
	unsigned int buffer_in_used[MAX_BUFFER_NUM];
	unsigned int buffer_num;
	unsigned int available_buffer_size;
    struct mutex lock;
};

struct zynq_mem_pool_ctx {
	struct zynq_mem_pool  *mem_pool;
	atomic_t	bInitialMemPool;
};
#define MAX_CHAN_NUM 1024 
//static struct zynq_mem_pool  mem_pool[MAX_CHAN_NUM];
//static atomic_t	bInitialMemPool[MAX_CHAN_NUM];

static int  pool_create( struct zynq_malloc_conf *conf)
{
	struct zynq_mem_pool_ctx *pool_ctx = (struct zynq_mem_pool_ctx *)conf->context; 
    struct zynq_mem_pool *p = pool_ctx->mem_pool;
    unsigned int i  = 0;
	if (conf == NULL) return  -1;
    
	mutex_init(&p->lock);
 
	p->buffer_virt_addr_list = (void **)conf->buffer_virt_addr_list;
	p->buffer_phy_addr_list = (dma_addr_t *)conf->buffer_phy_addr_list;
	p->buffer_num = conf->buffer_num;
	p->available_buffer_size = conf->available_buffer_size;
	if (p->buffer_num > MAX_BUFFER_NUM) return -1;
	
	for (i = 0 ; i  < p->buffer_num; i++) {
		p->buffer_in_used[i] = 0;
	}
    return 0;
}

static void pool_destroy(struct zynq_malloc_conf *conf) {
	struct zynq_mem_pool_ctx *pool_ctx = (struct zynq_mem_pool_ctx *)conf->context; 
    struct zynq_mem_pool *p = pool_ctx->mem_pool;
	if (!p) return;
	mutex_destroy(&p->lock);
    return;
}

static void * pool_alloc(unsigned long *size, dma_addr_t *dma_addr,  unsigned int *index,struct zynq_malloc_conf *conf)
{
	int i = 0;
	struct zynq_mem_pool_ctx *pool_ctx = (struct zynq_mem_pool_ctx *)conf->context; 
    struct zynq_mem_pool *p = pool_ctx->mem_pool;
    void *mem = NULL;
    unsigned int padd_size =   (*size+ (PAGE_SIZE -1)) & (~(PAGE_SIZE -1)); //PAGE_ALIGN( *size + PAGE_SIZE);
    
    *dma_addr = (dma_addr_t)0;
	*index  = (unsigned int)-1;

    if (conf->available_buffer_size < padd_size) {
		printk(KERN_INFO"[zynq_malloc]call pool_alloc failed!! (available size: %u < need size: %u)\n", conf->available_buffer_size, padd_size);
		return NULL;
	}
    
    //printk(KERN_INFO"[zynq_malloc]>>>>>>>>>>>>>>> channel_id = %d\n", conf->channel_id);
		
    *size = padd_size;
	
    if (conf->is_always_get_first_memory) {
        mem = (void *)p->buffer_virt_addr_list[0];
		*index = 0;
    } else {
     	for (i = 0; i < p->buffer_num; i++) {
			if (p->buffer_in_used[i] == 0) {
				mem = (void *)p->buffer_virt_addr_list[i];
				*dma_addr = (dma_addr_t)p->buffer_phy_addr_list[i];
				*index = i;
				p->buffer_in_used[i] = 1;
				goto exit;
			}
		}
    }
exit:    
    return mem;
}

//////////////////////////////////////////////////////////////////////

struct vb2_vmalloc_buf {
    void				*vaddr;
    dma_addr_t			dma_addr;
    struct page			**pages;
    struct vm_area_struct		*vma;
    int				write;
    unsigned long			size;
    unsigned int			n_pages;
    atomic_t			refcount;
    struct vb2_vmarea_handler	handler;
    struct dma_buf			*dbuf;
	unsigned int index;
	unsigned int channel_id;
	unsigned int available_buffer_size;
	 struct zynq_mem_pool  *pool;
};

static void vb2_vmalloc_put(void *buf_priv);

static void *vb2_vmalloc_alloc(void *alloc_ctx, unsigned long size, gfp_t gfp_flags)
{

    struct zynq_malloc_conf *conf = alloc_ctx;
    struct vb2_vmalloc_buf *buf = NULL;
	struct zynq_mem_pool_ctx *pool_ctx = (struct zynq_mem_pool_ctx *)conf->context; 
    
	buf = kzalloc(sizeof(*buf), GFP_KERNEL | gfp_flags);
    if (!buf)
        return NULL;

    if (atomic_inc_and_test(&(pool_ctx->bInitialMemPool))) {
        //printk(KERN_INFO"[zynq_malloc](%d)\n",__LINE__);
        if (pool_create(conf) != 0) {
            printk(KERN_INFO"[zynq_malloc]call pool_create failed!!\n");
            return NULL;
        }
        //printk(KERN_INFO"[zynq_malloc](%d)\n",__LINE__);
    }
    
    buf->channel_id = conf->channel_id;
	buf->available_buffer_size = conf->available_buffer_size;
    buf->size = size;
    //buf->vaddr = vmalloc_user(buf->size);
    buf->vaddr = pool_alloc(&buf->size, &buf->dma_addr, &buf->index, conf);
	
    if ((buf->vaddr == NULL) || ((void *)buf->dma_addr == NULL)) {
        printk(KERN_INFO"[zynq_malloc]xxx failed to allocate memory throug the memory pool!! (vaddr=%p, dma_addr=%p)(channel_id = %d)\n", (void *)buf->vaddr, (void *)buf->dma_addr, conf->channel_id);
        return  NULL;
    }

    buf->handler.refcount = &buf->refcount;
    buf->handler.put = vb2_vmalloc_put;
    buf->handler.arg = buf;

    if (!buf->vaddr) {
        pr_debug("[zynq_malloc]malloc of size %ld failed\n", buf->size);
        kfree(buf);
        return NULL;
    }
	
	buf->pool = pool_ctx->mem_pool;
    atomic_inc(&buf->refcount);
    return buf;
}

static void vb2_vmalloc_put(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;
	struct zynq_mem_pool *p = buf->pool;
	
    if (atomic_dec_and_test(&buf->refcount)) {
       p->buffer_in_used[buf->index] = 0;
       kfree(buf);
    }
}

static void *vb2_vmalloc_cookie(void *buf_priv) {
    struct vb2_vmalloc_buf *buf = buf_priv;

    return &buf->dma_addr;
}

static void *vb2_vmalloc_get_userptr(void *alloc_ctx, unsigned long vaddr,
                                     unsigned long size, int write) {
    return NULL;
}

static void vb2_vmalloc_put_userptr(void *buf_priv) {
	return;
}

static void *vb2_vmalloc_vaddr(void *buf_priv) {
	
	struct vb2_vmalloc_buf *buf = buf_priv;

	if (!buf->vaddr) {
		printk(KERN_INFO"[zynq_malloc] Address of an unallocated plane requested "
		       "or cannot map user pointer\n");
		return NULL;
	}

	return buf->vaddr;
}

static unsigned int vb2_vmalloc_num_users(void *buf_priv) {
    struct vb2_vmalloc_buf *buf = buf_priv;
    return atomic_read(&buf->refcount);
}


static int vb2_vmalloc_mmap(void *buf_priv, struct vm_area_struct *vma) { 
  
	struct vb2_vmalloc_buf *buf = buf_priv;
  /* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
  unsigned long	region_origin = /*vma->vm_pgoff*/ 0 * PAGE_SIZE;
  unsigned long	region_length = vma->vm_end - vma->vm_start;
  unsigned long	physical_addr = buf->dma_addr + region_origin;	
  unsigned long	user_virtaddr = vma->vm_start;
  
  // sanity check: mapped region cannot expend past end of vram
  //if ( region_origin + region_length > dev->ctrlreg_size ) return -EINVAL;
  
  // let the kernel know not to try swapping out this region
	//vma->vm_flags =  VM_IO;
#if 0
    if (en_use_dev_mem_map) {
      	  		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			}else {
				vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
			}
#endif	
  
  if (en_non_cache_map) {
	printk(KERN_INFO"[zynq_malloc] map  the vma pages as non cached!!\n");
  	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
  }
  // invoke kernel function that sets up the page-table entries
  if ( remap_pfn_range( vma, 
			user_virtaddr, 
			physical_addr >> PAGE_SHIFT,
			region_length, 
			vma->vm_page_prot ) ) 
    return -EAGAIN;
  
  
	printk(KERN_INFO"[zynq_malloc] kmalloc %s: mapped dma addr 0x%x   at 0x%x, size 0x%x (use device memory type = %u)(off = %u)\n",
           __func__, (unsigned int)buf->dma_addr, (unsigned int)vma->vm_start, (unsigned int)buf->size, en_use_dev_mem_map, (unsigned int)vma->vm_pgoff);
  
  return 0;
	
}


/*********************************************/
/*       callbacks for DMABUF buffers        */
/*********************************************/

static int vb2_vmalloc_map_dmabuf(void *mem_priv) {
    return  -EFAULT;
}

static void vb2_vmalloc_unmap_dmabuf(void *mem_priv) {
	return;
}

static void vb2_vmalloc_detach_dmabuf(void *mem_priv) {
	return;
}

static void *vb2_vmalloc_attach_dmabuf(void *alloc_ctx, struct dma_buf *dbuf,
                                       unsigned long size, int write) {
	return NULL;
}

/*
V4L2_MEMORY_MMAP
V4L2_MEMORY_USERPTR
V4L2_MEMORY_DMABUF
 */
const struct vb2_mem_ops zynq_malloc_memops = {
    .alloc		= vb2_vmalloc_alloc,
    .put		= vb2_vmalloc_put,
    .cookie		= vb2_vmalloc_cookie,
    .get_userptr	= vb2_vmalloc_get_userptr,
    .put_userptr	= vb2_vmalloc_put_userptr,
    .map_dmabuf	= vb2_vmalloc_map_dmabuf,
    .unmap_dmabuf	= vb2_vmalloc_unmap_dmabuf,
    .attach_dmabuf	= vb2_vmalloc_attach_dmabuf,
    .detach_dmabuf	= vb2_vmalloc_detach_dmabuf,
    .vaddr		= vb2_vmalloc_vaddr,
    .mmap		= vb2_vmalloc_mmap,
    .num_users	= vb2_vmalloc_num_users,
};

void *zynq_malloc_init_ctx(zynq_malloc_conf_t *ctx)
{
    struct zynq_malloc_conf *conf;
//	int i = 0;
	struct zynq_mem_pool_ctx *pool_ctx = (struct zynq_mem_pool_ctx *)vmalloc(sizeof(struct zynq_mem_pool_ctx));
    
	conf = kzalloc(sizeof *conf, GFP_KERNEL);
	
	if (!pool_ctx) return ERR_PTR(-ENOMEM);
	
	pool_ctx->mem_pool = (struct zynq_mem_pool *)vmalloc(sizeof(struct zynq_mem_pool));
	
    if (!conf || !pool_ctx->mem_pool) {
		return ERR_PTR(-ENOMEM);
	}
	
    conf->is_always_get_first_memory = ctx->is_always_get_first_memory;
	conf->buffer_virt_addr_list = ctx->buffer_virt_addr_list;
	conf->buffer_phy_addr_list = ctx->buffer_phy_addr_list;
	conf->buffer_num = ctx->buffer_num;
	conf->channel_id = ctx->channel_id;
	conf->available_buffer_size = ctx->available_buffer_size;
	conf->context = (void *)pool_ctx;
	
	printk(KERN_INFO"[zynq_malloc]jefff  memory pool address : 0x%p\n", pool_ctx->mem_pool);
	
	atomic_set(&(pool_ctx->bInitialMemPool), -1);

    return conf;
}


void zynq_malloc_cleanup_ctx(void *alloc_ctx)
{
	struct zynq_malloc_conf *conf = (struct zynq_malloc_conf *)alloc_ctx;
	struct zynq_mem_pool_ctx *pool_ctx = (struct zynq_mem_pool_ctx *)conf->context; 
	
	if (conf != NULL) {
		pool_destroy(conf);
    	kfree(conf);
		atomic_dec(&(pool_ctx->bInitialMemPool));
	}

	if (pool_ctx->mem_pool != NULL) {
		vfree(pool_ctx->mem_pool);
		pool_ctx->mem_pool = NULL;
	}
	
	if (pool_ctx  != NULL) {
		 vfree(pool_ctx);
		 pool_ctx= NULL;
	}
}

dma_addr_t  zynq_malloc_plane_dma_addr(struct vb2_buffer *vb, unsigned int plane_no)
{
    dma_addr_t *addr = vb2_plane_cookie(vb, plane_no);
    return *addr;
}

MODULE_DESCRIPTION(" handling routine for a large revserved memory ");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");
MODULE_LICENSE("GPL");

