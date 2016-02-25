#include <linux/pci.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#define ZYNQ_VERSION "0.0.1"


#ifndef PCI_DEVICE_ID_ZYNQ
//#define PCI_DEVICE_ID_ZYNQ 0x0033
//#define PCI_DEVICE_ID_ZYNQ 0x7015
#define PCI_DEVICE_ID_ZYNQ 0x7022
//#define PCI_DEVICE_ID_ZYNQ 0x1533
#endif

#ifndef PCI_VENDOR_ID_ZYNQ
//#define PCI_VENDOR_ID_ZYNQ 0x168c
#define PCI_VENDOR_ID_ZYNQ 0x10ee
//#define PCI_DEVICE_ID_ZYNQ 0x8086
#endif

MODULE_DESCRIPTION("Fake pci driver  module");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(ZYNQ_VERSION);
#define PCIE_BAR_0	0x0
#define PCIE_BAR_1	0x1
#define FB_MEM 0x2

static int is_disable_pci = 0;
static void __iomem *pci_reg_base = NULL;

static   resource_size_t res_len = 0;
static   resource_size_t res_start = 0;

static  unsigned char  data_mem[1024 * 1024];

static char *kbuf =  NULL;
static dma_addr_t handle = (dma_addr_t)0;
static size_t size = (1000 * PAGE_SIZE);

static DEFINE_PCI_DEVICE_TABLE(zynq_pci_tbl) = {
    {PCI_DEVICE(PCI_VENDOR_ID_ZYNQ, PCI_DEVICE_ID_ZYNQ)},
    {0,}
};

static int  pci_probe_imp(struct pci_dev *pdev)
{
    int status = 0;
    int bar_num = PCIE_BAR_0;
    unsigned long flags = 0;
    unsigned int bar_type = 0;


    bar_type = pci_resource_flags(pdev, bar_num) & PCI_BASE_ADDRESS_SPACE; // 0 = memory, 1 = I/O
  	//f8500000-f96fffff : fbmem
	//f9700000-fdefffff : fbmem
	res_len = 0x1200000;
	res_start = 0xf8500000;

    dev_info(&pdev->dev,"[fake_zynq_pci]zynq bar type is %s (%lu)(%lu)\n", (bar_type  == 1)?"I/O":"memory", pci_resource_flags (pdev, bar_num) & IORESOURCE_IO, pci_resource_flags (pdev, bar_num) & IORESOURCE_MEM);

    if ((bar_type == 1) && (!res_start || ((pci_resource_flags (pdev, bar_num) & IORESOURCE_IO) == 0))) {
        dev_err(&pdev->dev, "[fake_zynq_pci]no I/O resource at PCI BAR #%u\n",  bar_num);
        status = -1;
        goto exit;
    }

    if ((bar_type  == 0) && (!res_start || ((pci_resource_flags (pdev, bar_num) & IORESOURCE_MEM) == 0))) {
        dev_err(&pdev->dev, "[fake_zynq_pci]no memory resource at PCI BAR #%u\n", bar_num);
        status = -1;
        goto exit;
    }

    flags = pci_resource_flags(pdev, bar_num);

    if (flags & IORESOURCE_MEM) {
        if (flags & IORESOURCE_CACHEABLE)
            dev_err(&pdev->dev, "[fake_zynq_pci] memory is cacheable.\n");
        else
            dev_err(&pdev->dev, "[fake_zynq_pci] memory is non-cacheable.\n");
    }

#if 0
    status = pci_request_regions(pdev, KBUILD_MODNAME);
    if (status)
        goto exit;

    pci_reg_base = pci_iomap(pdev,  bar_num , res_len);

    if (!pci_reg_base) {
        status = -EBUSY;
        goto release_regions_exit;
    }
#endif
	printk(KERN_INFO">>>>>>>>>>>>>>>>>>>res_len = 0x%08x, res_length= 0x%08x\n", res_len, res_start);
	if (!request_mem_region(res_start,res_len,
				"zynq fb")) {
		printk(KERN_ERR "[fake_zynq_pci] cannot reserve frame "
				"buffer memory\n");
		 status = -ENODEV;
		goto exit;
	}
/*
	32800000-33ffffff : PCI Bus 0000:01
    32800000-32803fff : 0000:01:00.0
    33000000-33ffffff : 0000:01:00.0
*/
	if (bar_num == PCIE_BAR_0)
		pci_reg_base =  ioremap_wc(res_start, res_len);
	else
		pci_reg_base = ioremap(res_start, res_len);
	
    if (!pci_reg_base) {
        status = -EBUSY;
        goto release_regions_exit;
    }
   

	if (bar_num == PCIE_BAR_1) {
    	dev_err(&pdev->dev, "[fake_zynq_pci]bar_num = 0x%lx , res_start = 0x%lx (virt : 0x%lx)\n",(unsigned long)bar_num,  (unsigned long)res_start, (unsigned long)phys_to_virt(res_start));
    	dev_err(&pdev->dev, "[fake_zynq_pci]Zynq Regisetr Base Address: 0x%lx  (phy : 0x%lx)\n", (unsigned long)pci_reg_base,  (unsigned long)virt_to_phys((void *)pci_reg_base));
    	dev_err(&pdev->dev, "[fake_zynq_pci] offset 0x00 value 0x%lx\n", (unsigned long) ioread32(pci_reg_base));
    	dev_err(&pdev->dev, "[fake_zynq_pci] offset 0x04 value 0x%lx\n",  (unsigned long)ioread32(pci_reg_base+0x4));
	}

	{
		  unsigned char  *ptr = (unsigned  char  *)((unsigned long)pci_reg_base + 0x3000);
		 unsigned int res_length = 0;
		 unsigned int i  =0;
		 
		 if (bar_num == PCIE_BAR_1) 
			 res_length = 8;
		 else
			 res_length = sizeof(data_mem);
		 
		memset(data_mem, 0x1a, sizeof(data_mem));
		printk(KERN_INFO"src: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", data_mem[0], data_mem[1], data_mem[2], data_mem[3], data_mem, res_length);
		*(unsigned char *)ptr  = (unsigned char)(*((unsigned char *)data_mem));
		/*if (bar_num == PCIE_BAR_1)*/printk(KERN_INFO"dest: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr, res_length);
		
		memset(data_mem, 0x1b, sizeof(data_mem));
		printk(KERN_INFO"src: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", data_mem[0], data_mem[1], data_mem[2], data_mem[3], data_mem, res_length);
        *(unsigned short *)ptr = (unsigned short)(*((unsigned short *)data_mem));
		/*if (bar_num == PCIE_BAR_1)*/ printk(KERN_INFO"dest: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr, res_length);

        memset(data_mem, 0x1c, sizeof(data_mem));
		printk(KERN_INFO"src: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", data_mem[0], data_mem[1], data_mem[2], data_mem[3], data_mem, res_length);
		*(unsigned int *)ptr   =  (unsigned int)(*((unsigned int *)data_mem));
		/*if (bar_num == PCIE_BAR_1)*/ printk(KERN_INFO"dest: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr, res_length);

		memset(data_mem, 0x1e, sizeof(data_mem));
		for (i = 0; i < res_length; i++){
					//ptr[i] = data_mem[i];
					 //wmb();
					 *((unsigned  char *)ptr + i)  = (unsigned char)(*((unsigned  char *)(data_mem + i))); 
		}
		printk(KERN_INFO"(without wmb)src: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", data_mem[0], data_mem[1], data_mem[2], data_mem[3], data_mem, res_length);
   		/*if (bar_num == PCIE_BAR_1)*/ printk(KERN_INFO"(without wmb)dest: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr, res_length);
		
		memset(data_mem, 0x2a, sizeof(data_mem));
		for (i = 0; i < res_length; i++){
					//ptr[i] = data_mem[i];
					 wmb();
					 *((unsigned  char *)ptr + i)  = (unsigned char)(*((unsigned  char *)(data_mem + i))); 
		}
		printk(KERN_INFO"(with wmb)src: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", data_mem[0], data_mem[1], data_mem[2], data_mem[3], data_mem, res_length);
   		/*if (bar_num == PCIE_BAR_1)*/ printk(KERN_INFO"(with wmb)dest: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr, res_length);
#if 0
		memset(data_mem, 0x2a, sizeof(data_mem));
		memcpy_toio((unsigned char *)ptr, (unsigned char *)data_mem, res_length);
		printk(KERN_INFO"src: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", data_mem[0], data_mem[1], data_mem[2], data_mem[3], data_mem, res_length);
   		/*if (bar_num == PCIE_BAR_1)*/ printk(KERN_INFO"dest: [0, 1, 2, 3] -> [%02x, %02x, %02x, %02x] (ptr, len)-->(%p,0x%08x)\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr, res_length);
#endif
	}

    kbuf = dma_alloc_coherent(NULL, size, &handle, GFP_KERNEL);
    dev_err(&pdev->dev, "[fake_zynq_pci] dma address :0x%lx, size :%u\n", (unsigned long)handle, (unsigned int)size);

    dev_info(&pdev->dev, "[fake_zynq_pci]zynq pci probe success\n");

    return 0;
//release_zynq_reg_base:
    if (pci_reg_base) {
		//pci_iounmap(pdev, pci_reg_base);
		iounmap(pci_reg_base);
	}
release_regions_exit:
  	//  pci_release_regions(pdev);
  	 release_mem_region(res_start,res_len);
exit:
    dev_info(&pdev->dev,"[fake_zynq_pci]Failed to request bar #%d!!\n", bar_num);
    return status;
}

static int pci_remove_imp(struct pci_dev *pdev)
{
    if (pci_reg_base != NULL) {

        dev_info(&pdev->dev,"[fake_zynq_pci]xxxfff Call pci_iounmap()\n");
        //pci_iounmap(pdev, pci_reg_base);
		iounmap(pci_reg_base);
    }

    if (kbuf != NULL)
        dma_free_coherent(NULL, size, kbuf, handle);

    //pci_release_regions(pdev);
	release_mem_region(res_start,res_len);
    return 0;
}


static int zynq_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{

    int ret = 0;
    u8  pci_rev =0 ,pci_lat = 0;

    if (!pdev || !ent) return -1;

    printk(KERN_INFO"[fake_zynq_pci] enter probe!\n");

    /* Enable PCI */
    ret = pci_enable_device(pdev);

    if (ret) return ret;

    pci_read_config_byte(pdev, PCI_CLASS_REVISION, &pci_rev);
    pci_read_config_byte(pdev, PCI_LATENCY_TIMER,  &pci_lat);

    printk(KERN_INFO"[fake_zynq_pci]%s: found at %s, bus: %d, rev: %d, irq: %d, "
           "latency: %d, mmio: 0x%llx\n", "zynq",
           pci_name(pdev), pdev->bus->number, pci_rev, pdev->irq, pci_lat,
           (unsigned long long)pci_resource_start(pdev, 0));

    pci_set_master(pdev);

    if (!pci_dma_supported(pdev, 0xffffffff)) {
        printk(KERN_INFO "[fake_zynq_pci]%s: Oops: no 32bit PCI DMA ???\n", "zynq");
        goto exit_disable_pci;

    }
#if 1
    ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
    if (ret) {
        printk(KERN_INFO"[zynq_driver]no suitable DMA available.\n");
        goto exit_disable_pci;
    }
#endif
    pci_probe_imp(pdev);

    return  0;

exit_disable_pci:
    pci_disable_device(pdev);
    is_disable_pci = 1;

    printk(KERN_INFO"[fake_zynq_pci] leave probe!\n");
    return  ret;
}


static void zynq_pci_remove(struct pci_dev *pdev)
{
    printk(KERN_INFO"[fake_zynq_pci] enter remove!\n");

    if (!pdev) return;

    pci_remove_imp(pdev);

    if (is_disable_pci == 0) pci_disable_device(pdev);

    printk(KERN_INFO"[fake_zynq_pci] leave remove!\n");
    return;
}


static struct pci_driver zynq_pci_driver = {
    .name = KBUILD_MODNAME,
    .probe = zynq_pci_probe,
    .remove = zynq_pci_remove,
    .id_table = zynq_pci_tbl
};


static int __init zynq_fake_pci_init(void)
{
    printk(KERN_INFO "[fake_zynq_pci] driver version %s loaded\n",
           ZYNQ_VERSION);

    return pci_register_driver(&zynq_pci_driver);
}

static void __exit zynq_fake_pci_fini(void)
{
    printk(KERN_INFO "[fake_zynq_pci] driver version %s unloaded\n",
           ZYNQ_VERSION);

    pci_unregister_driver(&zynq_pci_driver);

    return;
}

module_init(zynq_fake_pci_init);
module_exit(zynq_fake_pci_fini);