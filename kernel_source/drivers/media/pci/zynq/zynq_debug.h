#ifndef ZYNQ_DEBUG_H
#define ZYNQ_DEBUG_H
#include <linux/module.h>
extern  unsigned int debug_print;

#define zynq_printk(level, fmt, arg...)\
	do { if (debug_print >= level)\
		printk(KERN_INFO "[zynq]" fmt, ## arg);\
	} while (0)

void zynq_print_string(char *str);

#endif