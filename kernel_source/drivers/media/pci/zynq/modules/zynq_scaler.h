#ifndef ZYNQ_SCALER_H
#define ZYNQ_SCALER_H
#include <linux/io.h>

/*Refer from : http://www.xilinx.com/support/documentation/ip_documentation/v_scaler/v8_0/pg009_v_scaler.pdf*/
#define SCALER_MAX_NUM 4
#define SCALER_COEF_DATA_ROW_NUM  4
#define SCALER_COEF_DATA_COL_NUM 4
extern short scaler_coef_data_0[SCALER_COEF_DATA_ROW_NUM][SCALER_COEF_DATA_COL_NUM];

typedef short scaler_coef_row_t[SCALER_COEF_DATA_COL_NUM];


typedef enum scaler_option_flags {
    SCALER_OPTION_RESET = 1,
    SCALER_OPTION_SET_IN_SIZE = 2,
    SCALER_OPTION_SET_OUT_SIZE = 3,
    SCALER_OPTION_SET_CROP = 4,
    SCALER_OPTION_SET_NUM_PHASES = 5,
    SCALER_OPTION_SET_COEF_DATA = 6,
    SCALER_OPTION_SET_SHRINK_FACTOR = 7,
    SCALER_OPTION_UPDATE_CHANGE = 8,
    SCALER_OPTION_COEF_READ_ENABLE = 9,
    SCALER_OPTION_ENABLE = 10
} EScalerOptionFlags;

typedef struct {
    unsigned int width;
    unsigned int height;
} scaler_size_t;

typedef struct {
    unsigned int start_x;
    unsigned int start_y;
    unsigned int width;
    unsigned int height;
} scaler_crop_area_t;

typedef struct {
    unsigned int num_h_phases;
    unsigned int num_v_phases;
} scaler_num_phases_t;


typedef struct {
    scaler_coef_row_t *coef_data;
} scaler_coef_data_t;

typedef struct {
    unsigned int hsf;
    unsigned int vsf;
} scaler_shrink_factor_t;

typedef struct {
    unsigned char value;
} scaler_enable_t;

typedef struct {
    u8 is_initialized;
    u8 is_started;
} scaler_status_t;

int scaler_initial(void __iomem *pci_base_addr);

int scaler_initial_by_index(void __iomem *pci_base_addr, unsigned index);

int scaler_setoption(EScalerOptionFlags flag, void *userdata, unsigned  index);

int scaler_reset(unsigned index);

int scaler_start(unsigned index);

int scaler_stop(unsigned index);

int scaler_release(void __iomem *pci_base_addr);

void scaler_dump_registers(unsigned index);

int scaler_enable_reg_update(unsigned index);

int  scaler_disable_reg_update(unsigned index);

void scaler_get_status(scaler_status_t *st, unsigned index);

int scaler_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height) ;

int scaler_config_crop(unsigned int index,  unsigned int crop_start_x,  unsigned int crop_start_y,  unsigned int crop_width, unsigned int crop_height) ;

int scaler_get_regs(unsigned index, void *regs_ptr);

int scaler_set_reg(unsigned index, u16 offset, u32 value);
#endif