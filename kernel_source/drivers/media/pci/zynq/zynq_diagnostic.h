#ifndef	_ZYNQ_DIAGNOSTIC_H_
#define _ZYNQ_DIAGNOSTIC_H_

#include <linux/pci.h>

int zynq_diagnostic_probe( struct pci_dev *pdev);
int zynq_diagnostic_remove(struct pci_dev *pdev);

int zynq_diagnostic_init(void);
void  zynq_diagnostic_exit(void);

#endif