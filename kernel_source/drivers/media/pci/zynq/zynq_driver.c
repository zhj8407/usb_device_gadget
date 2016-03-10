#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#define ZYNQ_VERSION "0.0.1"

#include "zynq_fpga_verify.h"
#include "zynq_capture.h"
#include "zynq_display.h"
#include "zynq_control.h"
#include "zynq_audio.h"
#include "zynq_debug.h"
#include "zynq_uart.h"
#include "zynq_diagnostic.h"
#include "zynq_board.h"
/*NOTE:
 * (1) Because there is no ZYNQ FPGA, use the following PCI device for testing:
 * 		Network controller [0280]: Qualcomm Atheros AR9580 Wireless Network Adapter [168c:0033]
 * (2) The ventor ID will be defined in include/linux/pci_ids.h.
 * 03:00.0 Ethernet controller [0200]: Intel Corporation I210 Gigabit Network Connection [8086:1533] (rev 03)
*/
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

MODULE_DESCRIPTION("Driver for zynq based video/audio control module");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(ZYNQ_VERSION);

unsigned int en_video_capture = 1;
module_param(en_video_capture, int, 0644);

unsigned int en_video_display = 1;
module_param(en_video_display, int, 0644);

unsigned int en_audio = 1;
module_param(en_audio, int, 0644);

unsigned int en_uart = 1;
module_param(en_uart, int, 0644);

unsigned int en_video_control = 1;
module_param(en_video_control, int, 0644);

unsigned int en_diagnostic = 1;
module_param(en_diagnostic, int, 0644);

static DEFINE_PCI_DEVICE_TABLE(zynq_pci_tbl) = {
    {PCI_DEVICE(PCI_VENDOR_ID_ZYNQ, PCI_DEVICE_ID_ZYNQ)},
    {0,}
};

static int is_disable_pci = 0;

struct zynq_card {
    int (*capture_suspend_func)(void);
    int (*capture_resume_func)(void);
    int (*display_suspend_func)(void);
    int (*display_resume_func)(void);
    int (*audio_suspend_func)(void);
    int (*audio_resume_func)(void);
    int (*uart_suspend_func)(void);
    int (*uart_resume_func)(void);
    int (*pci_suspend_func)(struct pci_dev *pdev);
    int (*pci_resume_func)(struct pci_dev *pdev);
} ;


static	int capture_init_status = 0;
static 	int display_init_status = 0;
static 	int audio_init_status = 0;
static 	int uart_init_status = 0;
static	int control_init_status = 0;
static 	int diagnostic_init_status = 0;

static int zynq_pci_probe(struct pci_dev *pdev,
                          const struct pci_device_id *ent)
{
    int ret = 0;
    u8  pci_rev =0 ,pci_lat = 0;
#ifdef CONFIG_PM_SLEEP
    struct zynq_card *card = NULL;
#endif

    if (!pdev || !ent) return -1;

    /* Enable PCI */
    ret = pci_enable_device(pdev);

    if (ret) return ret;

    pci_read_config_byte(pdev, PCI_CLASS_REVISION, &pci_rev);
    pci_read_config_byte(pdev, PCI_LATENCY_TIMER,  &pci_lat);
#if 0
    zynq_printk(1, "[zynq_driver]%s: found at %s, bus: %d, rev: %d, irq: %d, "
                "latency: %d, mmio: 0x%llx\n", "zynq",
                pci_name(pdev), pdev->bus->number, pci_rev, pdev->irq, pci_lat,
                (unsigned long long)pci_resource_start(pdev, 0));
#endif
    pci_set_master(pdev);

    if (!pci_dma_supported(pdev, 0xffffffff)) {
        zynq_printk(0, "[zynq_driver]%s: Oops: no 32bit PCI DMA ???\n", "zynq");
        goto exit_disable_pci;

    }

    ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
    if (ret) {
        zynq_printk(0, "[zynq_driver]no suitable DMA available.\n");
        goto exit_disable_pci;
    }

    ret = vpif_pci_probe(pdev);
    if (ret) goto exit_disable_pci;

    zynq_setup_interrupt();

    zynq_setup_i2c_adapter() ;

    if (en_video_capture) {
        ret = vpif_capture_init(pdev);
        capture_init_status = ret;
		// if (ret) goto exit_disable_pci;
    }
    
    if (en_video_display) {
        ret = vpif_display_init(pdev);
		display_init_status = ret;
       	//if (ret) goto exit_disable_pci;
	}
	
    if (en_video_control) {
        ret =  vpif_control_init(pdev);
		control_init_status = ret;
        //if (ret) goto exit_disable_pci;
    }
    
    if (en_audio) {
        ret = zynq_audio_init(pdev);
		audio_init_status = ret;
       // if (ret) goto exit_disable_pci;
    }
    
    if (en_uart) {
        ret =zynq_uart_probe(pdev);
		uart_init_status = ret;
       // if (ret)  goto exit_disable_pci;
    }

    if (en_diagnostic) {
        ret = zynq_diagnostic_probe(pdev);
		diagnostic_init_status = ret;
       // if (ret)  goto exit_disable_pci;
    }

    
    if ((capture_init_status != 0) && (display_init_status != 0) && (audio_init_status != 0) && (uart_init_status != 0) && (control_init_status != 0) && (diagnostic_init_status != 0)){
		zynq_printk (0, "[zynq_driver] The init status for functions all are not zero:  (cap, disp, audio uart, control, diagnostic) -> (%d, %d, %d, %d, %d, %d).\n", capture_init_status, display_init_status, audio_init_status, uart_init_status, control_init_status, diagnostic_init_status);
		goto exit_disable_pci;
	}
    
#ifdef CONFIG_PM_SLEEP
    card = (struct zynq_card *) vmalloc(sizeof(struct zynq_card));

    if (card != NULL) {
		
		if (capture_init_status == 0) {
			card->capture_suspend_func = vpif_capture_suspend;
    		card->capture_resume_func = vpif_capture_resume;
		}
		if (display_init_status == 0) {
			card->display_suspend_func  = vpif_display_suspend;
    		card->display_resume_func = vpif_display_resume;
		}
	
		if (audio_init_status == 0) {
    		card->audio_suspend_func = zynq_audio_suspend;
    		card->audio_resume_func = zynq_audio_resume;
		}
		
		if (uart_init_status == 0) {
			card->uart_suspend_func = zynq_uart_suspend;
    		card->uart_resume_func = zynq_uart_resume;
		}
    	card->pci_suspend_func = vpif_pci_suspend;
    	card->pci_resume_func = vpif_pci_resume;
    	
    	pci_set_drvdata(pdev, card);
	}
#endif
	
	zynq_printk (0, "[zynq_driver] The init status for functions are (cap, disp, audio uart, control, diagnostic) -> (%d, %d, %d, %d, %d, %d).\n", capture_init_status, display_init_status, audio_init_status, uart_init_status, control_init_status, diagnostic_init_status);
    return 0;

exit_disable_pci:
    pci_disable_device(pdev);
    is_disable_pci = 1;

    if (pdev && (zynq_reg_base != NULL) ) {
        pci_iounmap(pdev, zynq_reg_base);
    }

    if (pdev) pci_release_regions(pdev);

    if (card != NULL) vfree(card);

    return ret;
}

static void zynq_pci_remove(struct pci_dev *pdev)
{
    struct zynq_card *card = NULL;

    if (!pdev) return;

    card =(struct zynq_card *) pci_get_drvdata(pdev);

    if (en_diagnostic && (diagnostic_init_status == 0)) zynq_diagnostic_remove(pdev);

    if (en_uart && (uart_init_status == 0)) zynq_uart_remove(pdev);

    if (en_audio && (audio_init_status == 0)) zynq_audio_release(pdev);

    if (en_video_control && (control_init_status == 0)) vpif_control_release(pdev);

    if (en_video_display && (display_init_status == 0)) vpif_display_release(pdev);

    if (en_video_capture && (capture_init_status == 0)) vpif_capture_release(pdev);

    vpif_pci_remove(pdev);

    if (is_disable_pci == 0) pci_disable_device(pdev);

    if (card != NULL) vfree(card);

    zynq_rls_i2c_adapter();

    return;
}

#ifdef CONFIG_PM_SLEEP

static int zynq_suspend(struct device *dev)
{
    struct pci_dev *pci = to_pci_dev(dev);
    struct zynq_card *card = (struct zynq_card *)pci_get_drvdata(pci);

    if (!card) goto exit;

    if (en_video_capture) card->capture_suspend_func();
    if (en_video_display) card->display_suspend_func();
    if (en_audio) card->audio_suspend_func();
    if (en_uart) card->uart_suspend_func();

    card->pci_suspend_func(pci);

exit:
    return 0;
}

static int zynq_resume(struct device *dev)
{
    struct pci_dev *pci = to_pci_dev(dev);
    struct zynq_card *card = (struct zynq_card *)pci_get_drvdata(pci);

    if (!card) goto exit;

    card->pci_resume_func(pci);

    if (en_video_capture) card->capture_resume_func();
    if (en_video_display) card->display_resume_func();
    if (en_audio) card->audio_resume_func();
    if (en_uart) card->uart_resume_func();

exit:
    return 0;
}

SIMPLE_DEV_PM_OPS(zynq_dev_pm_ops, zynq_suspend, zynq_resume);
#endif /* CONFIG_PM_SLEEP */

static struct pci_driver zynq_pci_driver = {
    .name = KBUILD_MODNAME,
    .probe = zynq_pci_probe,
    .remove = zynq_pci_remove,
    .id_table = zynq_pci_tbl,
#ifdef CONFIG_PM_SLEEP
    .driver = {
        .pm = &zynq_dev_pm_ops,
    },
#endif
};

static int __init zynq_init(void)
{

    zynq_printk(0,  "[zynq_driver]The driver version %s is loaded !!\n", ZYNQ_VERSION);

    if (en_uart) zynq_uart_init();

    if (en_diagnostic) zynq_diagnostic_init();

    return pci_register_driver(&zynq_pci_driver);
}

static void __exit zynq_fini(void)
{

    pci_unregister_driver(&zynq_pci_driver);

    if (en_diagnostic) zynq_diagnostic_exit();

    if (en_uart) zynq_uart_exit();

    zynq_printk(0,  "[zynq_driver]The driver version %s is unloaded!!\n", ZYNQ_VERSION);

    return;
}

module_init(zynq_init);
module_exit(zynq_fini);