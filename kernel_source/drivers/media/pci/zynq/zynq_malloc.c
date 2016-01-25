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

///////////////////////////////////////////////////////////////////////
static atomic_t	bInitialMemorPool = {-1};

struct zynq_mem_pool
{
  unsigned  char *pool_start_addr;
  unsigned char * next;
  unsigned char * end;
  struct mutex lock;
};

static struct zynq_mem_pool  gMemPool ;


static int  pool_create(size_t size, void *pool_start_addr) 
{
	struct zynq_mem_pool *p = &gMemPool;
	if (pool_start_addr == NULL) return  -1;
	mutex_init(&p->lock);
	p->pool_start_addr = pool_start_addr; 
	p->next =   p->pool_start_addr;
	p->end = p->next + size; 
    return 0;
}

static void pool_destroy(void)
{
	struct zynq_mem_pool *p = &gMemPool;
	mutex_destroy(&p->lock);
	return;
}

static size_t pool_available(void) {
	struct zynq_mem_pool *p = &gMemPool;
    return (size_t)(p->end - p->next);
}

static void * pool_alloc(unsigned long *size, unsigned int is_always_get_first_memory) {
	
	struct zynq_mem_pool *p = &gMemPool;
	void *mem = NULL;
	unsigned long padd_size = PAGE_ALIGN( *size + PAGE_SIZE);
	*size = padd_size;
	
	if( pool_available() < padd_size ) return NULL;
	if (is_always_get_first_memory)
	{
			mem = (void *)p->pool_start_addr;
	}
	else
	{
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

	if (atomic_inc_and_test(&bInitialMemorPool))
	{
		//printk(KERN_INFO"[zynq_malloc](%d)\n",__LINE__);
		if (pool_create(conf->pool_size, conf->pool_start_address) != 0)
		{
			printk(KERN_INFO"call pool_create failed!!\n");
			return NULL;
		}
		//printk(KERN_INFO"[zynq_malloc](%d)\n",__LINE__);
	}

	buf->size = size;
	//buf->vaddr = vmalloc_user(buf->size);
	buf->vaddr = pool_alloc(&buf->size, conf->is_always_get_first_memory);
	buf->dma_addr= (dma_addr_t) virt_to_phys((void *)buf->vaddr);
	
	if ((buf->vaddr == NULL) || ((void *)buf->dma_addr == NULL)) 
	{	
		printk(KERN_INFO"failed to allocate memory throug the memory pool!!\n");
		return  NULL;
	}

	buf->handler.refcount = &buf->refcount;
	buf->handler.put = vb2_vmalloc_put;
	buf->handler.arg = buf;
	
	if (!buf->vaddr) {
		pr_debug("vmalloc of size %ld failed\n", buf->size);
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
		if (pool_available() == 0)
		{	
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
	pr_debug("get_user_pages requested/got: %d/%d]\n", n_pages,
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
		pr_err("Address of an unallocated plane requested "
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

static int vb2_vmalloc_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct vb2_vmalloc_buf *buf = buf_priv;
	int ret;
	
	if (!buf) {
		printk(KERN_INFO"No memory to map\n");
		return -EINVAL;
	}
	
/*
	 * dma_mmap_* uses vm_pgoff as in-buffer offset, but we want to
	 * map whole buffer
	 */
	vma->vm_pgoff = 0;
	ret = dma_mmap_coherent(NULL, vma, buf->vaddr,
		buf->dma_addr, buf->size);
	
	if (ret) {
		printk(KERN_INFO"Remapping memory failed, error: %d\n", ret);
		return ret;
	}
	
	vma->vm_flags		|= VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_private_data	= &buf->handler;
	vma->vm_ops		= &vb2_common_vm_ops;

	vma->vm_ops->open(vma);

	printk(KERN_INFO"%s: mapped dma addr 0x%08lx at 0x%08lx, size 0x%08lx\n",
		__func__, (unsigned long)buf->dma_addr, vma->vm_start,
		buf->size);
	
	return 0;
}

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
	
	conf->is_always_get_first_memory = ctx->is_always_get_first_memory;
	conf->pool_start_address = ctx->pool_start_address;
	conf->pool_size= ctx->pool_size;
	
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

