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

spinlock_t vpif_lock;

void __iomem *zynq_reg_base = NULL;
EXPORT_SYMBOL_GPL(zynq_reg_base);

u32 zynq_reg_len = 0;
EXPORT_SYMBOL_GPL(zynq_reg_len);

void __iomem *zynq_video_input_window_base = NULL;
EXPORT_SYMBOL_GPL(zynq_video_input_window_base);
u32 zynq_video_input_window_len = 0;
EXPORT_SYMBOL_GPL(zynq_video_input_window_len);
u32 zynq_video_input_window_base_dma_handle = 0;
EXPORT_SYMBOL_GPL(zynq_video_input_window_base_dma_handle);

static u8 bIsReleasePCI = 0;

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
	
	bar_type = pci_resource_flags(pdev, bar_num) & PCI_BASE_ADDRESS_SPACE; // 0 = memory, 1 = I/O
	res_len = pci_resource_len(pdev,  bar_num);
	res_start = pci_resource_start(pdev, bar_num);


	if (bar_type != 0) return -1;
	
	zynq_video_input_window_base = pci_iomap(pdev,  bar_num , res_len);
	if (!zynq_reg_base) {
		dev_info(&pdev->dev,"[zynq_core]Failed to request bar #%d!!\n", bar_num); 
		ret = -EBUSY;
		goto exit;
	}
	zynq_video_input_window_len = res_len;
	zynq_video_input_window_base_dma_handle = res_start;
	
	dev_err(&pdev->dev, "[zynq_core]bar_num = 0x%x , res_start = 0x%x, res_len = 0x%x\n",(unsigned int)bar_num,  (unsigned int)res_start, (unsigned int)res_len);
	dev_err(&pdev->dev, "[zynq_core] Zynq Video Input Window Base Address: 0x%x\n", (unsigned int)zynq_video_input_window_base);
exit:	
	return ret;
}

static int release_display_resource(struct pci_dev *pdev)
{
	int ret = 0;
	
	if (zynq_video_input_window_base != NULL) pci_iounmap(pdev, zynq_video_input_window_base);
	
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

	 dev_info(&pdev->dev,"[zynq_core]zynq bar type is %s (%lu)(%lu)\n", (bar_type  == 1)?"I/O":"memory", pci_resource_flags (pdev, bar_num) & IORESOURCE_IO, pci_resource_flags (pdev, bar_num) & IORESOURCE_MEM);
	
	if ((bar_type == 1) && (!res_start || ((pci_resource_flags (pdev, bar_num) & IORESOURCE_IO) == 0))) 
	{
		dev_err(&pdev->dev, "[zynq_core]no I/O resource at PCI BAR #%u\n",  bar_num);
		status = -1;
        goto exit;
	}
	
	if ((bar_type  == 0) && (!res_start || ((pci_resource_flags (pdev, bar_num) & IORESOURCE_MEM) == 0))) 
	{
		dev_err(&pdev->dev, "[zynq_core]no memory resource at PCI BAR #%u\n", bar_num);
		status = -1;
		goto exit;
	}
	
	status = pci_request_regions(pdev, KBUILD_MODNAME);
	if (status)
		goto exit;
	
	zynq_reg_base = pci_iomap(pdev,  bar_num , res_len);
	if (!zynq_reg_base) {
		status = -EBUSY;
		goto release_regions_exit;
	}
	
	zynq_reg_len = res_len;
	
	dev_err(&pdev->dev, "[zynq_core]bar_num = 0x%x , res_start = 0x%x, res_len = 0x%x\n",(unsigned int)bar_num,  (unsigned int)res_start, (unsigned int)res_len);
	dev_err(&pdev->dev, "[zynq_core] Zynq Regisetr Base Address: 0x%x\n", (unsigned int)zynq_reg_base);
	dev_err(&pdev->dev, "[zynq_core] offset 0x00 value 0x%x (logic version)\n", (unsigned int)fpga_reg_read(zynq_reg_base, FPGA_LOGIC_VERSION_REG));
	dev_err(&pdev->dev, "[zynq_core] offset 0x04 value 0x%x (compile time)\n",  (unsigned int) fpga_reg_read(zynq_reg_base, FPGA_COMPILE_TIME_REG));	
#if 1	
	if (init_display_resource(pdev) != 0) 
	{
		status = -EBUSY;
		goto release_zynq_reg_base;
	}
#endif
	spin_lock_init(&vpif_lock);

	vpif_control_init_pipeline(pdev);
	
	dev_info(&pdev->dev, "[zynq_core]zynq pci probe success\n");
	
	return 0;
release_zynq_reg_base:
	pci_iounmap(pdev, zynq_reg_base);
	zynq_reg_base = NULL;
release_regions_exit: 
	if (bIsReleasePCI == 0) {
		pci_release_regions(pdev);
		bIsReleasePCI = 1;
	}
exit:
	dev_info(&pdev->dev,"[zynq_core]Failed to request bar #%d!!\n", bar_num); 
	return status;
}

int vpif_pci_remove(struct pci_dev *pdev)
{
	zynq_printk(0,"Enter vpif_pci_remove()\n");
	if (zynq_reg_base != NULL) {
		zynq_printk(0,"Call pci_iounmap()\n");
		vpif_control_release_pipeline(pdev);
		pci_iounmap(pdev, zynq_reg_base);
	}
#if 1
	release_display_resource(pdev);
#endif	
	zynq_printk(0,"Call pci_release_regions()\n");
	if (bIsReleasePCI == 0) {
		pci_release_regions(pdev);
		bIsReleasePCI = 1;
	}
	zynq_printk(0,"Leave vpif_pci_remove()\n");
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_PM_SLEEP	
int vpif_pci_suspend(struct pci_dev *pdev){
	zynq_printk(0,"[zynq_core]Enter vpif_pci_suspend().\n");
	if (!pdev) return -1;
	pci_disable_device(pdev);
	pci_save_state(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
	zynq_printk(0,"[zynq_core]Leave vpif_pci_suspend().\n");
	return 0;
}
int vpif_pci_resume(struct pci_dev *pdev){
	zynq_printk(0,"[zynq_core]Enter vpif_pci_resume().\n");
	if(!pdev) return -1;
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	if (pci_enable_device(pdev) < 0) {
		zynq_printk(0, "[zynq_core] pci_enable_device failed!!\n");
		//snd_card_disconnect(card);
		return -EIO;
	}
	pci_set_master(pdev);
	zynq_printk(0,"[zynq_capture]Leave vpif_pci_resume().\n");
	return 0;
}
#endif
