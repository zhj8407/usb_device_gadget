#ifndef	_ZYNQ_UART_H_
#define _ZYNQ_UART_H_
#include <linux/pci.h>

int zynq_uart_probe( struct pci_dev *pdev);
int zynq_uart_remove(struct pci_dev *pdev);

int zynq_uart_init(void);
void  zynq_uart_exit(void);

#ifdef CONFIG_PM_SLEEP
int zynq_uart_suspend(void);
int zynq_uart_resume(void);
#endif

#endif