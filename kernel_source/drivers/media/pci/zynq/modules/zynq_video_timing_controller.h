#ifndef ZYNQ_VIDEO_TIMING_CONTROLLER_H
#define ZYNQ_VIDEO_TIMING_CONTROLLER_H
#include <linux/io.h>

#define VIDEO_TIMING_CONTROLLER_MAX_NUM 2

typedef enum timing_option_flags {
	VTIMING_OPTION_RESET = 1,
	VTIMING_OPTION_ENABLE = 2,
	VTIMING_OPTION_UPDATE_CHANGE = 3,
	VTIMING_OPTION_SET_FORMAT =4,
	VTIMING_OPTION_SET_SIZE = 5
} EVTimingOptionFlags;

typedef enum timing_formats {
	VTIMING_FORMAT_YUV422 = 0,
	VTIMING_FORMAT_YUV444 = 1,
	VTIMING_FORMAT_RGB = 2,
	VTIMING_FORMAT_YUV420 = 3
} EVTimingFormats;

typedef enum timing_sizes {
	VTIMING_SIZE_1080P60 = 0,
	VTIMING_SIZE_720P60 = 1,
	VTIMING_SIZE_1080P50 = 2,
	VTIMING_SIZE_720P50 = 3,
	VTIMING_SIZE_NUM =4
} EVTimingSizes;

typedef struct {
	EVTimingFormats value;
}vtiming_fromat_t;

typedef struct {
	EVTimingSizes value;
}vtiming_size_t;

typedef struct{
	unsigned char value;
} vtiming_enable_t;

typedef struct{
	u8 is_initialized;
	u8 is_started;
} vtiming_status_t;

int  vtiming_initial(void __iomem *pci_base_addr);

int vtiming_initial_by_index(void __iomem *pci_base_addr, unsigned index);

int  vtiming_setoption(EVTimingOptionFlags  flag, void *userdata, unsigned int index);

int  vtiming_reset(unsigned int index);

int  vtiming_start(unsigned int index);

int  vtiming_stop(unsigned int index);

int vtiming_release(void __iomem *pci_base_addr);

void  vtiming_dump_registers(unsigned int index);

int vtiming_enable_reg_update(unsigned int index);

int  vtiming_disable_reg_update(unsigned int index);

void vtiming_get_status(vtiming_status_t *st, unsigned int index);

int vtiming_config_input_size(unsigned int index, unsigned int in_width, unsigned int in_height);

int vtiming_get_regs(unsigned int index, void *regs_ptr);

int vtiming_set_reg(unsigned int index, u16 offset, u32 value);
#endif