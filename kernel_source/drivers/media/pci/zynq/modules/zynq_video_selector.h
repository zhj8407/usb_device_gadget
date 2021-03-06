#ifndef ZYNQ_VIDEO_SELECTOR_H
#define ZYNQ_VIDEO_SELECTOR_H
#include <linux/io.h>

typedef enum vselector_option_flags {
	VSELECTOR_OPTION_SET_VOUT_FRAME_SIZE = 1,
	VSELECTOR_OPTION_ENABLE = 2,
	VSELECTOR_OPTION_SET_VOUT0_FULL_SRC = 3,
	VSELECTOR_OPTION_SET_VOUT1_FULL_SRC = 4,
	VSELECTOR_OPTION_SET_VOUT0_1_16_SRC = 5,
	VSELECTOR_OPTION_SET_VOUT1_1_16_SRC = 6,
} EVSelectorOptionFlags;

typedef struct {
	unsigned int width;
	unsigned int height;
} vselector_vout_frame_size_t;

typedef struct {
	unsigned char  vin0;
	unsigned char vin1;
	unsigned char vin2;
	unsigned char cpu;
} vselector_source_t;

typedef struct{
	unsigned char value;
} vselector_enable_t;

typedef struct{
	u8 is_initialized;
	u8 is_started;
} vselector_status_t;

int vselector_initial(void __iomem *pci_base_addr);

int vselector_setoption(EVSelectorOptionFlags flag, void *userdata);

int vselector_reset(void);

int vselector_start(void);

int vselector_stop(void);

int vselector_release(void __iomem *pci_base_addr);

int vselector_enable_reg_update(void);

int  vselector_disable_reg_update(void);

void vselector_dump_registers(void);

void vselector_get_status(vselector_status_t *st);

int vselector_get_regs(void *regs_ptr);

int vselector_set_reg(u16 offset, u32 value);

#endif