#ifndef ZYNQ_RESAMPLER_H
#define ZYNQ_RESAMPLER_H

#include <linux/io.h>

#define RESAMPLER_MAX_NUM 5

typedef enum resampler_option_flags {
    RESAMPLER_OPTION_RESET = 1,
    RESAMPLER_OPTION_UPDATE_CHANGE = 2,
    RESAMPLER_OPTION_BYPASS_ENABLE = 3,
    RESAMPLER_OPTION_ENABLE = 4,
    RESAMPLER_OPTION_SET_ACTIVE_SIZE = 5
} EResamplerOptionFlags;

typedef struct {
    unsigned int width;
    unsigned int height;
} resampler_size_t;

typedef struct {
    unsigned char value;
} resampler_enable_t;

typedef struct {
    u8 is_initialized;
    u8 is_started;
} resampler_status_t;

int resampler_initial(void __iomem *pci_base_addr);

int resampler_initial_by_index(void __iomem *pci_base_addr, unsigned index);

int resampler_setoption( EResamplerOptionFlags flag, void *userdata, unsigned  index);

int resampler_reset(unsigned index);

int resampler_start(unsigned index);

int resampler_stop(unsigned index);

int resampler_release(void __iomem *pci_base_addr);

void resampler_dump_registers(unsigned index);

int resampler_enable_reg_update(unsigned index);

int  resampler_disable_reg_update(unsigned index);

void resampler_get_status(resampler_status_t *st, unsigned index);

int resampler_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height) ;

int resampler_get_regs(unsigned index, void *regs_ptr);

int resampler_set_reg(unsigned index, u16 offset, u32 value);

#endif