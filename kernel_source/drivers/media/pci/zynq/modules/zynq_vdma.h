#ifndef ZYNQ_VDMA_H
#define ZYNQ_VDMA_H
#include <linux/io.h>

#define VDMA_MAX_NUM 4

typedef enum vdma_option_flags {
    VDMA_OPTION_RESET = 1,
    VDMA_OPTION_UPDATE_CHANGE = 2,
    VDMA_OPTION_ENABLE = 3,
    VDMA_OPTION_SET_IN_SIZE  =4
} EVDMAOptionFlags;


typedef struct {
    unsigned int width;
    unsigned int height;
} vdma_size_t;


typedef struct {
    unsigned char value;
} vdma_enable_t;

typedef struct {
    u8 is_initialized;
    u8 is_started;
} vdma_status_t;

int vdma_initial(void __iomem *pci_base_addr);

int vdma_initial_by_index(void __iomem *pci_base_addr, unsigned index);

int vdma_setoption( EVDMAOptionFlags flag, void *userdata, unsigned  index);

int vdma_reset(unsigned index);

int vdma_start(unsigned index);

int vdma_stop(unsigned index);

int vdma_release(void __iomem *pci_base_addr);

void vdma_dump_registers(unsigned index);

int vdma_enable_reg_update(unsigned index);

int vdma_disable_reg_update(unsigned index);

void vdma_get_status(vdma_status_t *st, unsigned index);

int vdma_get_regs(unsigned index, void *regs_ptr);

int vdma_set_reg(unsigned index, u16 offset, u32 value);

int vdma_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height);

#endif