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

static u8 g_is_always_get_first_memory  =0;
//static struct vm_area_struct  g_vma;
static u32 start_dma_address = 0;
static u32 start_address = 0;

///////////////////////////////////////////////////////////////////////
static atomic_t	bInitialMemorPool = {-1};

struct zynq_mem_pool {
	u32 phy_pool_start_addr;
    unsigned  char *pool_start_addr;
    unsigned char * next;
    unsigned char * end;
    struct mutex lock;
};

static struct zynq_mem_pool  gMemPool ;


static int  pool_create(size_t size, void *pool_start_addr, u32 phy_pool_start_addr)
{
    struct zynq_mem_pool *p = &gMemPool;
    if (pool_start_addr == NULL) return  -1;
    mutex_init(&p->lock);
    p->pool_start_addr = pool_start_addr;
    p->next =   p->pool_start_addr;
    p->end = p->next + size;
	p->phy_pool_start_addr = phy_pool_start_addr;
    return 0;
}

static void pool_destroy(void)
{
    struct zynq_mem_pool *p = &gMemPool;
    mutex_destroy(&p->lock);
    return;
}
#if 0
static size_t pool_available(void)
{
    struct zynq_mem_pool *p = &gMemPool;
    return (size_t)(p->end - p->next);
}
#endif

static void * pool_alloc(unsigned long *size, unsigned int is_always_get_first_memory)
{

    struct zynq_mem_pool *p = &gMemPool;
    void *mem = NULL;
    unsigned long padd_size =   (*size+ (PAGE_SIZE -1)) & (~(PAGE_SIZE -1)); //PAGE_ALIGN( *size + PAGE_SIZE);
    *size = padd_size;
	
	start_address = (u32)p->pool_start_addr;
	start_dma_address = (u32)p->phy_pool_start_addr;
	
	if (((void *)start_dma_address == NULL) || ((void *)start_address == NULL)){
		printk(KERN_INFO"[zynq_malloc]There is error because (pool_start_addr ==  %p) or (phy_start_address == %p)\n",(void *)p->pool_start_addr,  (void *)p->phy_pool_start_addr);	
		return  NULL;
	}
		
    //if( pool_available() < padd_size ) return NULL;
    if (is_always_get_first_memory) {
        mem = (void *)p->pool_start_addr;
    } else {
        mem = (void*)p->next;
        mutex_lock(&p->lock);
        p->next += padd_size;
        mutex_unlock(&p->lock);
    }
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
};




static void vb2_vmalloc_put(void *buf_priv);

static void *vb2_vmalloc_alloc(void *alloc_ctx, unsigned long size, gfp_t gfp_flags)
{

    struct zynq_malloc_conf *conf = alloc_ctx;

    struct vb2_vmalloc_buf *buf = NULL;

    buf = kzalloc(sizeof(*buf), GFP_KERNEL | gfp_flags);
    if (!buf)
        return NULL;

    if (atomic_inc_and_test(&bInitialMemorPool)) {
        //printk(KERN_INFO"[zynq_malloc](%d)\n",__LINE__);
        if (pool_create(conf->pool_size, conf->pool_start_address, conf->phy_pool_start_address) != 0) {
            printk(KERN_INFO"[zynq_malloc]call pool_create failed!!\n");
            return NULL;
        }
        //printk(KERN_INFO"[zynq_malloc](%d)\n",__LINE__);
    }

    buf->size = size;
    //buf->vaddr = vmalloc_user(buf->size);
    buf->vaddr = pool_alloc(&buf->size, conf->is_always_get_first_memory);
    buf->dma_addr= (dma_addr_t)(start_dma_address + ( buf->vaddr - start_address));	//(dma_addr_t) virt_to_phys((void *)buf->vaddr);

    if ((buf->vaddr == NULL) || ((void *)buf->dma_addr == NULL)) {
        printk(KERN_INFO"[zynq_malloc]xxx failed to allocate memory throug the memory pool!! (vaddr = %p, dma_addr = %p)\n", (void *)buf->vaddr, (void *)buf->dma_addr);
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

    atomic_inc(&buf->refcount);
    return buf;
}

static void vb2_vmalloc_put(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;

    if (atomic_dec_and_test(&buf->refcount)) {
        //vfree(buf->vaddr);
     /*   if (pool_available() == 0)*/ {
            pool_destroy();
            atomic_dec(&bInitialMemorPool);
        }
        kfree(buf);
    }
}


static void *vb2_vmalloc_cookie(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;

    return &buf->dma_addr;
}

static void *vb2_vmalloc_get_userptr(void *alloc_ctx, unsigned long vaddr,
                                     unsigned long size, int write)
{
    struct vb2_vmalloc_buf *buf;
    unsigned long first, last;
    int n_pages, offset;
    struct vm_area_struct *vma;
    dma_addr_t physp;

    buf = kzalloc(sizeof(*buf), GFP_KERNEL);
    if (!buf)
        return NULL;

    buf->write = write;
    offset = vaddr & ~PAGE_MASK;
    buf->size = size;


    vma = find_vma(current->mm, vaddr);
    if (vma && (vma->vm_flags & VM_PFNMAP) && (vma->vm_pgoff)) {
        if (vb2_get_contig_userptr(vaddr, size, &vma, &physp))
            goto fail_pages_array_alloc;
        buf->vma = vma;
        buf->vaddr = ioremap_nocache(physp, size);
        if (!buf->vaddr)
            goto fail_pages_array_alloc;
    } else {
        first = vaddr >> PAGE_SHIFT;
        last  = (vaddr + size - 1) >> PAGE_SHIFT;
        buf->n_pages = last - first + 1;
        buf->pages = kzalloc(buf->n_pages * sizeof(struct page *),
                             GFP_KERNEL);
        if (!buf->pages)
            goto fail_pages_array_alloc;

        /* current->mm->mmap_sem is taken by videobuf2 core */
        n_pages = get_user_pages(current, current->mm,
                                 vaddr & PAGE_MASK, buf->n_pages,
                                 write, 1, /* force */
                                 buf->pages, NULL);
        if (n_pages != buf->n_pages)
            goto fail_get_user_pages;

        buf->vaddr = vm_map_ram(buf->pages, buf->n_pages, -1,
                                PAGE_KERNEL);
        if (!buf->vaddr)
            goto fail_get_user_pages;
    }

    buf->vaddr += offset;
    return buf;

fail_get_user_pages:
    pr_debug("[zynq_malloc]get_user_pages requested/got: %d/%d]\n", n_pages,
             buf->n_pages);
    while (--n_pages >= 0)
        put_page(buf->pages[n_pages]);
    kfree(buf->pages);

fail_pages_array_alloc:
    kfree(buf);

    return NULL;
}

static void vb2_vmalloc_put_userptr(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;
    unsigned long vaddr = (unsigned long)buf->vaddr & PAGE_MASK;
    unsigned int i;

    if (buf->pages) {
        if (vaddr)
            vm_unmap_ram((void *)vaddr, buf->n_pages);
        for (i = 0; i < buf->n_pages; ++i) {
            if (buf->write)
                set_page_dirty_lock(buf->pages[i]);
            put_page(buf->pages[i]);
        }
        kfree(buf->pages);
    } else {
        if (buf->vma)
            vb2_put_vma(buf->vma);
        iounmap(buf->vaddr);
    }
    kfree(buf);
}

static void *vb2_vmalloc_vaddr(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;

    if (!buf->vaddr) {
        pr_err("[zynq_malloc]Address of an unallocated plane requested "
               "or cannot map user pointer\n");
        return NULL;
    }

    return buf->vaddr;
}

static unsigned int vb2_vmalloc_num_users(void *buf_priv)
{
    struct vb2_vmalloc_buf *buf = buf_priv;
    return atomic_read(&buf->refcount);
}

#if 1
static int vb2_vmalloc_mmap(void *buf_priv, struct vm_area_struct *vma) 
{
	
	struct vb2_vmalloc_buf *buf = buf_priv;
	unsigned long start = 0;
	//unsigned long mmio_pgoff = 0;
	u32 len = 0;
	int ret = 0;
	
	start = buf->dma_addr;
	len = buf->size;
	
	vma->vm_pgoff = 0;
#if 0	
	mmio_pgoff = PAGE_ALIGN((start & ~PAGE_MASK) + len) >> PAGE_SHIFT;
	printk(KERN_INFO"[zynq_malloc] mmio_pgoff = %lu, vm_pgoff = %lu\n", mmio_pgoff, vma->vm_pgoff);
	if (vma->vm_pgoff >= mmio_pgoff) {
		vma->vm_pgoff -= mmio_pgoff;
	}
#endif
	
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	
	if (en_use_dev_mem_map) {
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	}else {
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	}
	
	vma->vm_private_data	= &buf->handler;
	vma->vm_ops		= &vb2_common_vm_ops;
	
	ret = vm_iomap_memory(vma, start, len);
	if (ret)
		goto error;
	
	vma->vm_ops->open(vma);
	
	printk(KERN_INFO"[zynq_malloc]mapped dma addr 0x%08lx  at 0x%08lx, size 0x%08lx (use device memory type = %u)(off = %u)\n", 
		   (unsigned long) buf->dma_addr, (unsigned long)vma->vm_start, (unsigned long)len, (unsigned int)en_use_dev_mem_map, (unsigned int)vma->vm_pgoff);
	
	return 0;
	
error:
	return ret;
}
#endif

#if 0
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
  
  // invoke kernel function that sets up the page-table entries
  if ( remap_pfn_range( vma, 
			user_virtaddr, 
			physical_addr >> PAGE_SHIFT,
			region_length, 
			vma->vm_page_prot ) ) 
    return -EAGAIN;
     
  
	printk(KERN_INFO"[zynq_malloc] xxxffeeggg %s: mapped dma addr 0x%08lx   at 0x%08lx, size 0x%08lx (use device memory type = %u)(off = %u)\n",
           __func__, (unsigned long) buf->dma_addr, vma->vm_start,
           buf->size, en_use_dev_mem_map, vma->vm_pgoff);
  
  return 0;
	
}
#endif
#if 0
static int vb2_vmalloc_mmap(void *buf_priv, struct vm_area_struct *vma) {
	
	  	  struct vb2_vmalloc_buf *buf = buf_priv;
		  unsigned long user_count = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
		  unsigned long count = PAGE_ALIGN(buf->size) >> PAGE_SHIFT;
		  unsigned long pfn =  page_to_pfn(virt_to_page(buf->dma_addr));// page_to_pfn(virt_to_page(buf->vaddr));
		  unsigned long off = 0;
		  int ret = -ENXIO;
		  
		vma->vm_flags		|= VM_DONTEXPAND | VM_DONTDUMP | VM_IO | VM_PFNMAP ;
    	vma->vm_private_data	= &buf->handler;
    	vma->vm_ops		= &vb2_common_vm_ops;
		vma->vm_pgoff = 0;
		
		off = vma->vm_pgoff;
		   if (en_use_dev_mem_map) {
      	  		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			}else {
				vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
			}
			

		 if (off < count && user_count <= (count - off)) {
		            ret = remap_pfn_range(vma, vma->vm_start,
	                                 pfn + off,
	                                 user_count << PAGE_SHIFT,
	                                 vma->vm_page_prot);
						printk(KERN_INFO"[zynq_malloc] xxxfff %s: mapped dma addr 0x%08lx   at 0x%08lx, size 0x%08lx (use device memory type = %u)(off = %u)(user_count=0x%08lx)(count=0x%08lx)\n",
           __func__, (unsigned long) buf->dma_addr, vma->vm_start,
           buf->size, en_use_dev_mem_map, off, user_count << PAGE_SHIFT, count << PAGE_SHIFT);
		}
		
		vma->vm_ops->open(vma);
	
		return ret;
}
#endif

#if 0
static int vb2_vmalloc_mmap(void *buf_priv, struct vm_area_struct *vma)
{
    struct vb2_vmalloc_buf *buf = buf_priv;
	//static unsigned int isFirstRun = 1;
	
    int ret;
    if (!buf) {
        printk(KERN_INFO"[zynq_malloc]No memory to map\n");
        return -EINVAL;
    }
#if 0
    if ((isFirstRun == 0)  && g_is_always_get_first_memory) {
		memcpy(vma, &g_vma, sizeof(struct vm_area_struct ));
		goto exit1;
	}
#endif
    vma->vm_pgoff = 0;
	vma->vm_flags		|= VM_DONTEXPAND | VM_DONTDUMP | VM_IO | VM_PFNMAP ;
    vma->vm_private_data	= &buf->handler;
    vma->vm_ops		= &vb2_common_vm_ops;
	
exit1:

	if (en_use_dev_mem_map) {
		ret = dma_mmap_coherent(NULL, vma, buf->vaddr,
                            buf->dma_addr, buf->size);
		
	} else {
		ret = dma_mmap_writecombine(NULL, vma, buf->vaddr,
      	                    buf->dma_addr, buf->size);
	}

    if (ret) {
        printk(KERN_INFO"[zynq_malloc]Remapping memory failed, error: %d\n", ret);
        return ret;
    }
    
    vma->vm_ops->open(vma);
#if  0
	if (isFirstRun == 1) {
		isFirstRun = 0;
		memcpy(&g_vma, vma, sizeof(struct vm_area_struct ));
	}
#endif	

    printk(KERN_INFO"[zynq_malloc] jeff12345678 %s: mapped dma addr 0x%08lx at 0x%08lx, size 0x%08lx (use device memory type = %u)\n",
           __func__, (unsigned long)buf->dma_addr, vma->vm_start,
           buf->size, en_use_dev_mem_map);

    return 0;
	
	
}
#endif

/*********************************************/
/*       callbacks for DMABUF buffers        */
/*********************************************/

static int vb2_vmalloc_map_dmabuf(void *mem_priv)
{
    struct vb2_vmalloc_buf *buf = mem_priv;

    buf->vaddr = dma_buf_vmap(buf->dbuf);

    return buf->vaddr ? 0 : -EFAULT;
}

static void vb2_vmalloc_unmap_dmabuf(void *mem_priv)
{
    struct vb2_vmalloc_buf *buf = mem_priv;

    dma_buf_vunmap(buf->dbuf, buf->vaddr);
    buf->vaddr = NULL;
}

static void vb2_vmalloc_detach_dmabuf(void *mem_priv)
{
    struct vb2_vmalloc_buf *buf = mem_priv;

    if (buf->vaddr)
        dma_buf_vunmap(buf->dbuf, buf->vaddr);

    kfree(buf);
}

static void *vb2_vmalloc_attach_dmabuf(void *alloc_ctx, struct dma_buf *dbuf,
                                       unsigned long size, int write)
{
    struct vb2_vmalloc_buf *buf;

    if (dbuf->size < size)
        return ERR_PTR(-EFAULT);

    buf = kzalloc(sizeof(*buf), GFP_KERNEL);
    if (!buf)
        return ERR_PTR(-ENOMEM);

    buf->dbuf = dbuf;
    buf->write = write;
    buf->size = size;

    return buf;
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

    conf = kzalloc(sizeof *conf, GFP_KERNEL);
    if (!conf)
        return ERR_PTR(-ENOMEM);

	g_is_always_get_first_memory  = ctx->is_always_get_first_memory;
    conf->is_always_get_first_memory = ctx->is_always_get_first_memory;
    conf->pool_start_address = ctx->pool_start_address;
    conf->pool_size= ctx->pool_size;
	conf->phy_pool_start_address = ctx->phy_pool_start_address;
    return conf;
}


void zynq_malloc_cleanup_ctx(void *alloc_ctx)
{
    kfree(alloc_ctx);
    pool_destroy();
    atomic_dec(&bInitialMemorPool);
}

dma_addr_t  zynq_malloc_plane_dma_addr(struct vb2_buffer *vb, unsigned int plane_no)
{
    dma_addr_t *addr = vb2_plane_cookie(vb, plane_no);
    return *addr;
}

MODULE_DESCRIPTION(" handling routine for a large revserved memory ");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");
MODULE_LICENSE("GPL");

