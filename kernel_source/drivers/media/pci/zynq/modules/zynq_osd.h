#ifndef ZYNQ_OSD_H
#define ZYNQ_OSD_H
#include <linux/io.h>

#define OSD_MAX_NUM 2

typedef enum osd_option_flags {
    OSD_OPTION_RESET = 1,
    OSD_OPTION_UPDATE_CHANGE = 2,
    OSD_OPTION_ENABLE = 3,
    OSD_OPTION_SET_OUTPUT_ACTIVE_SIZE  =4,
    OSD_OPTION_SET_LAYER = 5,
	OSD_OPTION_ENABLE_LAYER0 = 6,
	OSD_OPTION_ENABLE_LAYER1 = 7
} EOSDOptionFlags;

typedef enum osd_layer_id {
    OSD_LAYER_0 = 0,
    OSD_LAYER_1 = 1,
    OSD_LAYER_NUM = 2
} EOSDLayerID;

typedef struct {
    unsigned int width;
    unsigned int height;
} osd_size_t;

typedef struct {
    EOSDLayerID id;
    unsigned int enable;
    unsigned int global_alpha_enable;
    unsigned int priority; //0(the lowest) ~ 7(the heighest)
    unsigned int alpha_value; //default is 0x100
    unsigned int position_x;
    unsigned int position_y;
    unsigned int width;
    unsigned int height;
} osd_layer_paramter_t;

typedef struct {
    unsigned char value;
} osd_enable_t;

typedef struct {
    u8 is_initialized;
    u8 is_started;
} osd_status_t;


int osd_initial_by_index(void __iomem *pci_base_addr, unsigned index);

int osd_initial(void __iomem *pci_base_addr);

int osd_setoption( EOSDOptionFlags flag, void *userdata, unsigned  index);

int osd_reset(unsigned index);

int osd_start(unsigned index);

int osd_stop(unsigned index);

int osd_release(void __iomem *pci_base_addr);

void osd_dump_registers(unsigned index);

int osd_enable_reg_update(unsigned index);

int osd_disable_reg_update(unsigned index);

void osd_get_status(osd_status_t *st, unsigned index);

int osd_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height);

int osd_get_regs(unsigned index, void *regs_ptr);

int osd_set_reg(unsigned index, u16 offset, u32 value);

#endif