#ifndef	_ZYNQ_AUDIO_H_
#define _ZYNQ_AUDIO_H_
#include <linux/pci.h>

typedef struct {
	void **data;
	unsigned int data_len;
} zynq_audio_card_info_t;

int zynq_audio_init (struct pci_dev *pdev);
int zynq_audio_release(struct pci_dev *pdev);

#ifdef CONFIG_PM_SLEEP
int zynq_audio_suspend(void);
int zynq_audio_resume(void);
#endif

#endif