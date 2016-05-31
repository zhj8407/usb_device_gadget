#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>
#include <linux/v4l2-dv-timings.h>

//#include <mach/hardware.h>
#include "zynq_core.h"
#include "zynq_debug.h"
#include "zynq_board.h"
#include "zynq_control.h"
#include "zynq_fpga_verify.h"

#define VPIF_CH0_MAX_MODES	(22)
#define VPIF_CH1_MAX_MODES	(02)
#define VPIF_CH2_MAX_MODES	(15)
#define VPIF_CH3_MAX_MODES	(02)

#define PCIE_BAR_0	0x0
#define PCIE_BAR_1	0x1

unsigned int en_use_fb_mem = 0;
module_param(en_use_fb_mem, int, 0644);

unsigned int en_use_dev_mem_map = 0;
module_param(en_use_dev_mem_map, int, 0644);

unsigned int en_non_cache_map = 0;
module_param(en_non_cache_map, int, 0644);

spinlock_t vpif_lock;

void __iomem *zynq_reg_base = NULL;
EXPORT_SYMBOL_GPL(zynq_reg_base);

u32 zynq_reg_len = 0;
EXPORT_SYMBOL_GPL(zynq_reg_len);

u32 zynq_reg_base_dma_handle = 0;
EXPORT_SYMBOL_GPL(zynq_reg_base_dma_handle);

void __iomem *zynq_video_input_window_base = NULL;
EXPORT_SYMBOL_GPL(zynq_video_input_window_base);
u32 zynq_video_input_window_len = 0;
EXPORT_SYMBOL_GPL(zynq_video_input_window_len);
u32 zynq_video_input_window_base_dma_handle = 0;
EXPORT_SYMBOL_GPL(zynq_video_input_window_base_dma_handle);
static u8 is_sucessful_reseverve_video_input_window_mem = 0;

static u8 bIsReleasePCI = 0;

unsigned int g_video_cap_nr[VPIF_CAPTURE_NUM_CHANNELS] = {0, 1, 2, 3, 6, 7};
unsigned int g_video_cap_en[VPIF_CAPTURE_NUM_CHANNELS] = {1, 1, 1, 1, 1, 1};

unsigned int g_video_display_nr[VPIF_DISPLAY_NUM_CHANNELS] = {4};
unsigned int g_video_display_en[VPIF_DISPLAY_NUM_CHANNELS] = {1};
unsigned int g_video_control_nr[1] = {5};

const struct vpif_channel_config_params vpif_ch_params[] = {
    /* HDTV formats */
    {
        .name = "1080p30",
        .width = 1920,
        .height = 1080,
        .frm_fmt = 1,
        .eav2sav = 138-8,
        .sav2eav = 1920,
        .l1 = 1,
        .l3 = 42,
        .l5 = 1122,
        .vsize = 1125,
        .capture_format = 0,
        .vbi_supported = 0,
        .hd_sd = 1,
        .dv_timings = V4L2_DV_BT_CEA_1920X1080P30,
        .pixelformat =  V4L2_PIX_FMT_YUV420
    },
    {
        .name = "720p30",
        .width = 1280,
        .height = 720,
        .frm_fmt = 1,
        .eav2sav = 370 - 8,
        .sav2eav = 1280,
        .l1 = 1,
        .l3 = 26,
        .l5 = 746,
        .vsize = 750,
        .capture_format = 0,
        .vbi_supported = 0,
        .hd_sd = 1,
        .dv_timings = V4L2_DV_BT_CEA_1280X720P30,
        .pixelformat =  V4L2_PIX_FMT_YUV420
    },
    {
        .name = "720p60",
        .width = 1280,
        .height = 720,
        .frm_fmt = 1,
        .eav2sav = 370 - 8,
        .sav2eav = 1280,
        .l1 = 1,
        .l3 = 26,
        .l5 = 746,
        .vsize = 750,
        .capture_format = 0,
        .vbi_supported = 0,
        .hd_sd = 1,
        .dv_timings = V4L2_DV_BT_CEA_1280X720P60,
        .pixelformat =  V4L2_PIX_FMT_YUV420
    }
};

const unsigned int vpif_ch_params_count = ARRAY_SIZE(vpif_ch_params);

int vpif_set_video_params(struct vpif_params *vpifparams, u8 channel_id)
{
    int found = 1;

    return found;
}
#if 1
static int init_display_resource(struct pci_dev *pdev)
{
    int ret = 0;
    int bar_num = PCIE_BAR_0;
    unsigned int bar_type = 0;
    resource_size_t res_len = 0;
    resource_size_t res_start = 0;
	unsigned long flags = 0;
	  
    bar_type = pci_resource_flags(pdev, bar_num) & PCI_BASE_ADDRESS_SPACE; // 0 = memory, 1 = I/O
    
    if (en_use_fb_mem == 1) {
		//f8500000-f96fffff : fbmem
		//f9700000-fdefffff : fbmem
		res_len = 0x1200000;
		res_start = 0xf8500000;
	}else {
		res_len = pci_resource_len(pdev,  bar_num);
    	res_start = pci_resource_start(pdev, bar_num);
	}
	
    if (bar_type != 0){
		 zynq_printk(0,"[zynq_core]The bar #%d is not IORESOURCE_MEM!! \n", bar_num);
		return -1;
	}
	
	  flags = pci_resource_flags(pdev, bar_num);

    if (flags & IORESOURCE_MEM) {
        if (flags & IORESOURCE_CACHEABLE)
            zynq_printk(0,"[zynq_core] The bar %d memory is cacheable.\n", bar_num);
        else
            zynq_printk(0,"[zynq_core]The bar %d  memory is non-cacheable.\n", bar_num);
    }

    if (!request_mem_region(res_start,res_len, "zynq video input window")) {
			zynq_printk(0,"[zynq_core] Cannot reserve video input window memory!!\n");
		 ret = -ENODEV;
		goto exit;
	}
    
     is_sucessful_reseverve_video_input_window_mem = 1;
	
	if (en_use_dev_mem_map)
		zynq_video_input_window_base =  ioremap(res_start, res_len);	
	else	 
   		zynq_video_input_window_base =  ioremap_wc(res_start, res_len);
    
	zynq_printk(0, "[zynq_core] en_use_dev_mem_map = %d\n", en_use_dev_mem_map);
	
	if (!zynq_video_input_window_base) {
        zynq_printk(0,"[zynq_core]Failed to request bar #%d!!\n", bar_num);
        ret = -EBUSY;
        goto exit;
    }
    zynq_video_input_window_len = res_len;
    zynq_video_input_window_base_dma_handle = res_start;

    zynq_printk(1, "[zynq_core]bar_num = 0x%x , res_start = 0x%x, res_len = 0x%x\n",(unsigned int)bar_num,  (unsigned int)res_start, (unsigned int)res_len);
    zynq_printk(1, "[zynq_core]video Input Window Base Address: 0x%x\n", (unsigned int)zynq_video_input_window_base);
	
	return  ret;
	
exit:
	if ( is_sucessful_reseverve_video_input_window_mem) {
		 is_sucessful_reseverve_video_input_window_mem = 0;
		release_mem_region( zynq_video_input_window_base_dma_handle, zynq_video_input_window_len);
	}
    return ret;
}

static int release_display_resource(struct pci_dev *pdev)
{
    int ret = 0;

//    if (zynq_video_input_window_base != NULL) pci_iounmap(pdev, zynq_video_input_window_base);

	 if (zynq_video_input_window_base != NULL){ 
		 iounmap(zynq_video_input_window_base);
		 zynq_video_input_window_base = NULL;
	 }
	
	 if ( is_sucessful_reseverve_video_input_window_mem == 1)  {
		  is_sucessful_reseverve_video_input_window_mem = 0;
		 release_mem_region( zynq_video_input_window_base_dma_handle, zynq_video_input_window_len);
	 }
    return ret;

}
#endif

int vpif_pci_probe(struct pci_dev *pdev)
{
    int status = 0;
    int bar_num = PCIE_BAR_1;


    unsigned int bar_type = 0;
    resource_size_t res_len = 0;
    resource_size_t res_start = 0;

    bar_type = pci_resource_flags(pdev, bar_num) & PCI_BASE_ADDRESS_SPACE; // 0 = memory, 1 = I/O
    res_len = pci_resource_len(pdev,  bar_num);
    res_start = pci_resource_start(pdev, bar_num);

    //zynq_printk(1,"[zynq_core]zynq bar type is %s (%lu)(%lu)\n", (bar_type  == 1)?"I/O":"memory", pci_resource_flags (pdev, bar_num) & IORESOURCE_IO, pci_resource_flags (pdev, bar_num) & IORESOURCE_MEM);

    if ((bar_type == 1) && (!res_start || ((pci_resource_flags (pdev, bar_num) & IORESOURCE_IO) == 0))) {
        zynq_printk(0, "[zynq_core]no I/O resource at PCI BAR #%u\n",  bar_num);
        status = -1;
        goto exit;
    }

    if ((bar_type  == 0) && (!res_start || ((pci_resource_flags (pdev, bar_num) & IORESOURCE_MEM) == 0))) {
        zynq_printk(0, "[zynq_core]no memory resource at PCI BAR #%u\n", bar_num);
        status = -1;
        goto exit;
    }

    if (!request_mem_region(res_start, res_len, "zynq regisetr region")){
    	zynq_printk(0,"[zynq_core] Cannot reserve register region!!\n");
		status = -ENODEV;
        goto exit;
	}

    zynq_reg_base =  ioremap(res_start, res_len);
    if (!zynq_reg_base) {
        status = -EBUSY;
        goto release_regions_exit;
    }

    zynq_reg_len = res_len;
	zynq_reg_base_dma_handle = res_start;
	
    zynq_printk(1,"[zynq_core]bar_num = 0x%x , res_start = 0x%x, res_len = 0x%x\n",(unsigned int)bar_num,  (unsigned int)res_start, (unsigned int)res_len);
    //zynq_printk(1,"[zynq_core] Zynq Regisetr Base Address: 0x%x\n", (unsigned int)zynq_reg_base);
    zynq_printk(1,"[zynq_core]FPGA logical version is 0x%08x .\n", (unsigned int)fpga_reg_read(zynq_reg_base, FPGA_LOGIC_VERSION_REG));
    zynq_printk(1,"[zynq_core]FPGA compile time is 0x%08x .\n",  (unsigned int) fpga_reg_read(zynq_reg_base, FPGA_COMPILE_TIME_REG));
#if 1
    if (init_display_resource(pdev) != 0) {
        status = -EBUSY;
        goto release_zynq_reg_base;
    }
#endif
    spin_lock_init(&vpif_lock);

    vpif_control_init_pipeline(pdev);

    zynq_printk(1, "[zynq_core]PCI probe success !!\n");

    return 0;
release_zynq_reg_base:
    iounmap(zynq_reg_base);
    zynq_reg_base = NULL;
release_regions_exit:
    if (bIsReleasePCI == 0) {
        release_mem_region( zynq_reg_base_dma_handle, zynq_reg_len);
        bIsReleasePCI = 1;
    }
exit:
    zynq_printk(0, "[zynq_core]Failed to probe PCI !!\n");
    return status;
}

int vpif_pci_remove(struct pci_dev *pdev)
{
    if (zynq_reg_base != NULL) {
        vpif_control_release_pipeline(pdev);
        iounmap(zynq_reg_base);
		zynq_reg_base = NULL;
    }
#if 1
    release_display_resource(pdev);
#endif
    if (bIsReleasePCI == 0) {
         release_mem_region(zynq_reg_base_dma_handle, zynq_reg_len);
        bIsReleasePCI = 1;
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_PM_SLEEP
int vpif_pci_suspend(struct pci_dev *pdev)
{
    if (!pdev) return -1;
    pci_disable_device(pdev);
    pci_save_state(pdev);
    pci_set_power_state(pdev, PCI_D3hot);
    return 0;
}
int vpif_pci_resume(struct pci_dev *pdev)
{
    if(!pdev) return -1;
    pci_set_power_state(pdev, PCI_D0);
    pci_restore_state(pdev);
    if (pci_enable_device(pdev) < 0) {
        zynq_printk(0, "[zynq_core] pci_enable_device failed!!\n");
        //snd_card_disconnect(card);
        return -EIO;
    }
    pci_set_master(pdev);
    return 0;
}
#endif
