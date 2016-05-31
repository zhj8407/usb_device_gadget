#include <linux/mutex.h>
#include "zynq_scaler.h"
//#include "zynq_regs.h"
#include "../zynq_fpga_verify.h"
#include "../zynq_debug.h"
#include "../zynq_types.h"

short scaler_coef_data_0[SCALER_COEF_DATA_ROW_NUM][SCALER_COEF_DATA_COL_NUM ]= {
    {0, 0, 16384, 0},
    {-289, 3817, 14231, -1373},
    {-1023, 9216, 9216, -1023},
    {-1373, 14231, 3817, -289}
};

typedef struct {
    u16 offset;
    u32 value;
} scaler_cfg_reg;

extern scaler_cfg_reg scaler_cached_registers_0[];
extern scaler_cfg_reg scaler_cached_registers_1[];
extern scaler_cfg_reg  scaler_cached_registers_2[];
extern scaler_cfg_reg  scaler_cached_registers_3[];

typedef struct {
    void __iomem *base;
    u16 ctrl;
    u16 status;
    u16 error;
    u16 irq_enable;
    u16 version;
    u16 hsf;
    u16 vsf;
    u16 source_video_size;
    u16 h_aperture;
    u16 v_aperture;
    u16 output_size;
    u16 num_phases;
    u16 active_coefs;
    u16 hpa_y;
    u16 hpa_c;
    u16 vpa_y;
    u16 vpa_c;
    u16 coef_set_wr_addr;
    u16 coef_data_in;
    u16 coef_set_bank_rd_addr;
    u16 coef_mem_rd_addr;
    u16 coef_mem_output;
    u32 width;
    u32 height;
} scaler_handle_t;

#define SCALER_REG_NUM  (22)
static scaler_handle_t handles[SCALER_MAX_NUM];
static struct mutex locks[SCALER_MAX_NUM];
static scaler_cfg_reg *cached_registers[SCALER_MAX_NUM];
static unsigned short cached_coef_data[SCALER_MAX_NUM][SCALER_COEF_DATA_ROW_NUM * SCALER_COEF_DATA_COL_NUM];

static int sw_reset(unsigned int index);
static int set_source_size(scaler_size_t *size, unsigned int index);
static int set_dest_size(scaler_size_t *userdata, unsigned int index);
static int set_crop(scaler_crop_area_t *userdata, unsigned int  index);
static int set_num_phases(scaler_num_phases_t *userdata, unsigned int index);
static int set_coef_data(scaler_coef_data_t *userdata, unsigned int index);
static int set_shrink_factor(scaler_shrink_factor_t *userdata,  unsigned int  index);
static int set_update_change(scaler_enable_t *userdata, unsigned int index);
static int set_coef_read_enable(scaler_enable_t *userdata, unsigned int index);
static int set_enable(scaler_enable_t *userdata, unsigned int index);

static u8 is_initialized[SCALER_MAX_NUM] = {0};
static u8 is_started[SCALER_MAX_NUM] = {0};

static u32 get_cached_registers(unsigned int index, u16 reg)
{
    unsigned int count  = SCALER_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg)
            return cached_registers[index][i].value;

    return 0;
}

static int set_cached_registers(unsigned int index, u16 reg, u32 value)
{
    unsigned int count  = SCALER_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg) {
            cached_registers[index][i].value = value;
            return 0;
        }

    return -1;
}

void scaler_get_status(scaler_status_t *st, unsigned index)
{
    st->is_initialized = is_initialized[index];
    st->is_started = is_started[index];
}

int scaler_initial_by_index(void __iomem *pci_base_addr, unsigned index)
{

    unsigned i = index;

    if (i >= SCALER_MAX_NUM) return 0;

    if(is_initialized[i] == 1) return  0;

   // if ( i ==3) return 0;

    if (i == 0)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER0_REG);
    else if (i == 1)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER1_REG);
    else if (i == 2)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER2_REG);
    else if (i == 3)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER3_REG);

    handles[i].ctrl = 0x0000;
    handles[i].status =  0x0004;
    handles[i].error = 0x0008;
    handles[i].irq_enable =  0x000c; // This register does not exist in the Polycom scaler datasheet
    handles[i].version =0x0010; // This register does not exist in the Polycom scaler datasheet
    handles[i].hsf = 0x0100;
    handles[i].vsf = 0x0104;
    handles[i].source_video_size=  0x0108;
    handles[i].h_aperture =  0x010c;
    handles[i].v_aperture = 0x0110;
    handles[i].output_size = 0x0114;
    handles[i].num_phases = 0x0118;
    handles[i].active_coefs = 0x011c;
    handles[i].hpa_y = 0x0120;
    handles[i].hpa_c = 0x0124;
    handles[i].vpa_y = 0x0128;
    handles[i].vpa_c = 0x012c;
    handles[i].coef_set_wr_addr = 0x0130;
    handles[i].coef_data_in = 0x0134; //This register is written-only.
    handles[i]. coef_set_bank_rd_addr = 0x0138;
    handles[i]. coef_mem_rd_addr = 0x013c;
    handles[i]. coef_mem_output = 0x0140;
    mutex_init(&locks[i]);
    if (i == 0)
        cached_registers[i] = &scaler_cached_registers_0[0];
    else  if (i == 1)
        cached_registers[i] = &scaler_cached_registers_1[0];
    else if (i == 2)
        cached_registers[i] = &scaler_cached_registers_2[0];
    else if (i == 3)
        cached_registers[i] = &scaler_cached_registers_3[0];

    sw_reset(i);

    set_cached_registers(i, handles[i].ctrl , fpga_reg_read(handles[i].base , handles[i].ctrl ));
    set_cached_registers(i, handles[i].status , fpga_reg_read(handles[i].base , handles[i].status));
    set_cached_registers(i, handles[i].error , fpga_reg_read(handles[i].base , handles[i].error));

    set_cached_registers(i, handles[i].hsf, fpga_reg_read(handles[i].base , handles[i].hsf));
    set_cached_registers(i, handles[i].vsf, fpga_reg_read(handles[i].base , handles[i].vsf));

    set_cached_registers(i, handles[i].source_video_size , fpga_reg_read(handles[i].base ,handles[i].source_video_size));

    set_cached_registers(i, handles[i].h_aperture, fpga_reg_read(handles[i].base , handles[i].h_aperture));
    set_cached_registers(i, handles[i].v_aperture, fpga_reg_read(handles[i].base , handles[i].v_aperture));

    set_cached_registers(i, handles[i].output_size, fpga_reg_read(handles[i].base , handles[i].output_size));
    set_cached_registers(i, handles[i].num_phases, fpga_reg_read(handles[i].base , handles[i].num_phases));
    set_cached_registers(i, handles[i].active_coefs, fpga_reg_read(handles[i].base , handles[i].active_coefs));

    set_cached_registers(i, handles[i].hpa_y, fpga_reg_read(handles[i].base , handles[i].hpa_y));
    set_cached_registers(i, handles[i].hpa_c, fpga_reg_read(handles[i].base , handles[i].hpa_c));

    set_cached_registers(i, handles[i].vpa_y, fpga_reg_read(handles[i].base , handles[i].vpa_y));
    set_cached_registers(i, handles[i].vpa_c, fpga_reg_read(handles[i].base , handles[i].vpa_c));

    set_cached_registers(i, handles[i].coef_set_wr_addr, fpga_reg_read(handles[i].base , handles[i].coef_set_wr_addr));
    set_cached_registers(i, handles[i]. coef_set_bank_rd_addr, fpga_reg_read(handles[i].base , handles[i]. coef_set_bank_rd_addr));
    set_cached_registers(i, handles[i]. coef_mem_rd_addr, fpga_reg_read(handles[i].base , handles[i]. coef_mem_rd_addr));
    set_cached_registers(i, handles[i]. coef_mem_output, fpga_reg_read(handles[i].base , handles[i]. coef_mem_output));
    is_initialized[i] = 1;

    return 0;
}

int scaler_initial(void __iomem *pci_base_addr)
{

    int i = 0;
    for (i = 0; i < SCALER_MAX_NUM; i++) {

        if(is_initialized[i] == 1) continue;
      //  zynq_printk(1, "[zynq_scaler](%d)%d \n", __LINE__,i);
        //NOTE:The scaler 2 and scaler 3 is not workable for 1029 FPGA image.
        //if (i== 2 || i ==3) continue;

        if (i == 0)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER0_REG);
        else if (i == 1)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER1_REG);
        else if (i == 2)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER2_REG);
        else if (i == 3)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_SCALER3_REG);

        handles[i].ctrl = 0x0000;
        handles[i].status =  0x0004;
        handles[i].error = 0x0008;
        handles[i].irq_enable =  0x000c; // This register does not exist in the Polycom scaler datasheet
        handles[i].version =0x0010; // This register does not exist in the Polycom scaler datasheet
        handles[i].hsf = 0x0100;
        handles[i].vsf = 0x0104;
        handles[i].source_video_size=  0x0108;
        handles[i].h_aperture =  0x010c;
        handles[i].v_aperture = 0x0110;
        handles[i].output_size = 0x0114;
        handles[i].num_phases = 0x0118;
        handles[i].active_coefs = 0x011c;
        handles[i].hpa_y = 0x0120;
        handles[i].hpa_c = 0x0124;
        handles[i].vpa_y = 0x0128;
        handles[i].vpa_c = 0x012c;
        handles[i].coef_set_wr_addr = 0x0130;
        handles[i].coef_data_in = 0x0134; //This register is written-only.
        handles[i]. coef_set_bank_rd_addr = 0x0138;
        handles[i]. coef_mem_rd_addr = 0x013c;
        handles[i]. coef_mem_output = 0x0140;
        mutex_init(&locks[i]);
        if (i == 0)
            cached_registers[i] = &scaler_cached_registers_0[0];
        else  if (i == 1)
            cached_registers[i] = &scaler_cached_registers_1[0];
        else if (i == 2)
            cached_registers[i] = &scaler_cached_registers_2[0];
        else if (i == 3)
            cached_registers[i] = &scaler_cached_registers_3[0];

        sw_reset(i);

        set_cached_registers(i, handles[i].ctrl , fpga_reg_read(handles[i].base , handles[i].ctrl ));
        set_cached_registers(i, handles[i].status , fpga_reg_read(handles[i].base , handles[i].status));
        set_cached_registers(i, handles[i].error , fpga_reg_read(handles[i].base , handles[i].error));

        set_cached_registers(i, handles[i].hsf, fpga_reg_read(handles[i].base , handles[i].hsf));
        set_cached_registers(i, handles[i].vsf, fpga_reg_read(handles[i].base , handles[i].vsf));

        set_cached_registers(i, handles[i].source_video_size , fpga_reg_read(handles[i].base ,handles[i].source_video_size));

        set_cached_registers(i, handles[i].h_aperture, fpga_reg_read(handles[i].base , handles[i].h_aperture));
        set_cached_registers(i, handles[i].v_aperture, fpga_reg_read(handles[i].base , handles[i].v_aperture));

        set_cached_registers(i, handles[i].output_size, fpga_reg_read(handles[i].base , handles[i].output_size));

        set_cached_registers(i, handles[i].num_phases, fpga_reg_read(handles[i].base , handles[i].num_phases));
        set_cached_registers(i, handles[i].active_coefs, fpga_reg_read(handles[i].base , handles[i].active_coefs));

        set_cached_registers(i, handles[i].hpa_y, fpga_reg_read(handles[i].base , handles[i].hpa_y));
        set_cached_registers(i, handles[i].hpa_c, fpga_reg_read(handles[i].base , handles[i].hpa_c));

        set_cached_registers(i, handles[i].vpa_y, fpga_reg_read(handles[i].base , handles[i].vpa_y));
        set_cached_registers(i, handles[i].vpa_c, fpga_reg_read(handles[i].base , handles[i].vpa_c));

        set_cached_registers(i, handles[i].coef_set_wr_addr, fpga_reg_read(handles[i].base , handles[i].coef_set_wr_addr));

        set_cached_registers(i, handles[i]. coef_set_bank_rd_addr, fpga_reg_read(handles[i].base , handles[i]. coef_set_bank_rd_addr));

        set_cached_registers(i, handles[i]. coef_mem_rd_addr, fpga_reg_read(handles[i].base , handles[i]. coef_mem_rd_addr));

        set_cached_registers(i, handles[i]. coef_mem_output, fpga_reg_read(handles[i].base , handles[i]. coef_mem_output));

        is_initialized[i] = 1;
        //zynq_printk(1, "[zynq_scaler](%d)%d \n", __LINE__,i);
        //zynq_printk(1, "[zynq_scaler] Register base address for scaler %d : 0x%8lx \n", i,   (unsigned long)handles[i].base);
    }

    return 0;
}

int scaler_setoption(EScalerOptionFlags flag, void *userdata, unsigned int index)
{

    if (index >= SCALER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    switch (flag) {
        case SCALER_OPTION_RESET:
            sw_reset(index);
            break;
        case SCALER_OPTION_SET_IN_SIZE:
            set_source_size((scaler_size_t *)userdata, index);
            break;
        case SCALER_OPTION_SET_OUT_SIZE:
            set_dest_size((scaler_size_t *)userdata, index);
            break;
        case SCALER_OPTION_SET_CROP:
            set_crop((scaler_crop_area_t *)userdata, index);
            break;
        case SCALER_OPTION_SET_NUM_PHASES:
            set_num_phases(( scaler_num_phases_t *)userdata, index);
            break;
        case SCALER_OPTION_SET_COEF_DATA:
            set_coef_data((scaler_coef_data_t *)userdata, index);
            break;
        case  SCALER_OPTION_SET_SHRINK_FACTOR:
            set_shrink_factor((scaler_shrink_factor_t *)userdata, index);
            break;
        case SCALER_OPTION_UPDATE_CHANGE:
            set_update_change((scaler_enable_t *)userdata, index);
            break;
        case  SCALER_OPTION_COEF_READ_ENABLE:
            set_coef_read_enable((scaler_enable_t *)userdata, index);
            break;
        case  SCALER_OPTION_ENABLE:
            set_enable((scaler_enable_t *)userdata, index);
            break;
        default:
            break;
    }
    return 0;
}

static int write_default_value(unsigned index)
{

    scaler_disable_reg_update(index);

    mutex_lock(&locks[index]);

    //0x011c
    fpga_reg_write(handles[index].base, handles[index].active_coefs, 0x0);
    set_cached_registers(index, handles[index].active_coefs, 0x0);

    //0x0120
    fpga_reg_write(handles[index].base, handles[index].hpa_y, 0x0);
    set_cached_registers(index, handles[index].hpa_y, 0x0);

    //0x0124
    fpga_reg_write(handles[index].base, handles[index].hpa_c, 0x0);
    set_cached_registers(index, handles[index].hpa_c, 0x0);


    //0x0128
    fpga_reg_write(handles[index].base, handles[index].vpa_y, 0x0);
    set_cached_registers(index, handles[index].vpa_y, 0x0);

    //0x012c
    fpga_reg_write(handles[index].base, handles[index].vpa_c, 0x0);
    set_cached_registers(index, handles[index].vpa_c, 0x0);

    //0x0130
    fpga_reg_write(handles[index].base, handles[index].coef_set_wr_addr, 0x0);
    set_cached_registers(index, handles[index].coef_set_wr_addr, 0x0);

    //0x0138
    fpga_reg_write(handles[index].base, handles[index].coef_set_bank_rd_addr, 0x0);
    set_cached_registers(index, handles[index].coef_set_bank_rd_addr, 0x0);

    //0x013c
    fpga_reg_write(handles[index].base, handles[index].coef_mem_rd_addr, 0x0);
    set_cached_registers(index, handles[index].coef_mem_rd_addr, 0x0);

    mutex_unlock(&locks[index]);


    scaler_enable_reg_update(index);

    return 0;
}


int scaler_reset(unsigned index)
{

    if (index >= SCALER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    return sw_reset(index);
}
int scaler_start(unsigned index)
{
    scaler_enable_t enable;

    if (index >= SCALER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    if (is_started[index] == 1) return 0;

    //write_default_value(index);

    enable.value = 1;
    set_enable(&enable, index);

    mutex_lock(&locks[index]);
    is_started[index] = 1;
    mutex_unlock(&locks[index]);

    return 0;
}

int scaler_stop(unsigned index)
{

    scaler_enable_t enable;

    if (index >= SCALER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    if (is_started[index] == 0) return 0;

    enable.value = 0;
    set_enable(&enable, index);

    mutex_lock(&locks[index]);
    is_started[index] = 0;
    mutex_unlock(&locks[index]);

    return 0;
}
int scaler_release(void __iomem *scaler_pci_base_addr)
{
    int i = 0;
    for (i = 0; i < SCALER_MAX_NUM; i++) {

        if (is_initialized[i] == 0) continue;

        mutex_lock(&locks[i]);
        is_initialized[i] = 0;
        mutex_unlock(&locks[i]);

        mutex_destroy(&locks[i]);
    }
    return 0;
}

void scaler_dump_registers(unsigned index)
{

    unsigned int i = 0;
    unsigned int count  = SCALER_REG_NUM;

    for (i  = 0; i < count; i ++ ) {
        zynq_printk(1, "[0x%02x] = 0x%08x \n", cached_registers[index][i].offset, cached_registers[index][i].value);
    }

    count  = SCALER_COEF_DATA_ROW_NUM * SCALER_COEF_DATA_COL_NUM;
    for (i = 0; i < count;  i++) {
        zynq_printk(1, "[%u] = 0x%04x \n", i, cached_coef_data[index][i]);
    }
}

int scaler_get_regs(unsigned index, void *regs_ptr)
{

    vpif_video_cfg_regs_t  *regs = (vpif_video_cfg_regs_t *)regs_ptr;

    if (!regs) return -1;

    regs->regs = (vpif_video_cfg_reg_t *)(&cached_registers[index][0]);
    regs->num = SCALER_REG_NUM;

    return 0;
}

int scaler_enable_reg_update(unsigned index)
{
    scaler_enable_t enable;

    if (index >= SCALER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    enable.value = 0x1;
    set_update_change(&enable, index);

    return 0;
}

int  scaler_disable_reg_update(unsigned index)
{
    scaler_enable_t enable;

    if (index >= SCALER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    enable.value = 0x0;
    set_update_change(&enable, index);

    return  0;
}


int scaler_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height)
{

    scaler_size_t size;
    unsigned int out_width = in_width >> 2;
    unsigned int out_height = in_height >> 2;

    if (is_initialized[index] == 0) return 0;

    if ((in_width == handles[index].width) && (in_height == handles[index].height) )  return 0;

    size.width = in_width;
    size.height = in_height;
    set_source_size(&size, index);

    size.width = out_width;
    size.height = out_height;
    set_dest_size(&size, index);

    return 0;
}


int scaler_config_crop(unsigned int index,  unsigned int crop_start_x,  unsigned int crop_start_y,  unsigned int crop_width,   unsigned int crop_height)
{

    scaler_shrink_factor_t shrink_factor;
    scaler_crop_area_t crop;
    unsigned int out_width = 0;
    unsigned int out_height = 0;
    u32 value = 0;

    if (is_initialized[index] == 0) return 0;

    crop.start_x= crop_start_x;
    crop.start_y= crop_start_y;
    crop.width= crop_width;
    crop.height= crop_height;
    set_crop(&crop, index);

    value =  get_cached_registers(index, handles[index].output_size);
    out_width = value & 0x00001fff;
    out_height = (value & 0x1fff0000) >> 16;

    shrink_factor.hsf = ((crop_width << 20 )/out_width);
    shrink_factor.vsf = ((crop_height << 20)/out_height);
    set_shrink_factor(&shrink_factor, index);

    return 0;
}


int scaler_set_reg(unsigned index, u16 offset, u32 value)
{

    if (index >= SCALER_MAX_NUM) return  -1;

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, offset, value);
    set_cached_registers(index, offset, value);
    mutex_unlock(&locks[index]);

    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////

static int set_update_change(scaler_enable_t *enable, unsigned int index)
{
#if 0
    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(index, handles[index].ctrl) | 0x00000002;
    else
        value = get_cached_registers(index, handles[index].ctrl) & ~(0x00000002);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
#endif
    return 0;
}
static int set_coef_read_enable(scaler_enable_t *enable, unsigned int index)
{

    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(index, handles[index].ctrl) | 0x00000008;
    else
        value = get_cached_registers(index, handles[index].ctrl) & ~(0x00000008);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
    return 0;
}
static int set_enable(scaler_enable_t *enable, unsigned int index)
{
#if 1
    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(index, handles[index].ctrl) | 0x00000003;
    else
        value = get_cached_registers(index, handles[index].ctrl) & ~(0x00000001);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
#endif
    return 0;
}

static int sw_reset(unsigned int index)
{

    write_default_value(index);
#if 0
    /*Bit 31 of the CONTROL register,  SW_RESET facilitates software reset. Setting  SW_RESET reinitializes the core to GUI default values,
     * all internal registers and outputs are cleared and  held at initial values until  SW_RESET is set to 0.
     */
    u32 value = get_cached_registers(index, handles[index].ctrl) | 0x80000000;

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
#endif
    return 0;
}

static int set_source_size(scaler_size_t *size, unsigned int index)
{
    u32 value = 0x00000000;
    u16 reg = handles[index].source_video_size;
    u32  width = size->width;
    u32 height = size->height;

    if (width != handles[index].width) handles[index].width = width;

    if (height != handles[index].height) handles[index].height = height;

    value = get_cached_registers(index, reg) & ~( 0x1fff0000 | 0x00001fff);
    value = value  | ((height << 16) | width);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

    return 0;
}

static int set_dest_size(scaler_size_t *size, unsigned int index)
{
    u32 value = 0x00000000;
    u16 reg = handles[index].output_size;
    u32  width = size->width;
    u32 height = size->height;

    value = get_cached_registers(index, reg) & ~(0x1fff0000 | 0x00001fff);
    value = value | ((height << 16) | width);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

    return 0;
}

static int set_crop(scaler_crop_area_t *crop, unsigned int  index)
{

    unsigned int start_x = crop->start_x;
    unsigned int end_x = crop->start_x + crop->width - 1;
    unsigned int start_y = crop->start_y;
    unsigned int end_y = crop->start_y+ crop->height -1;
    u16 reg_h =  handles[index].h_aperture;
    u16 reg_v =  handles[index].v_aperture;
    u32 value_h = 0x0;
    u32 value_v = 0x0;

    value_h = get_cached_registers(index, reg_h) & ~(0x1fff0000 | 0x00001fff);
    value_h = value_h | ((end_x << 16) | start_x);

    value_v = get_cached_registers(index, reg_v) & ~(0x1fff0000 | 0x00001fff);
    value_v = value_v | ((end_y << 16) | start_y);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg_h, value_h);
    set_cached_registers(index, reg_h, value_h);
    fpga_reg_write(handles[index].base, reg_v, value_v);
    set_cached_registers(index, reg_v, value_v);
    mutex_unlock(&locks[index]);
    return  0;
}

static int set_num_phases(scaler_num_phases_t *num_phases, unsigned int index)
{
    unsigned int h_num_phases = num_phases->num_h_phases;
    unsigned int v_num_phases = num_phases->num_v_phases;
    u16 reg = handles[index].num_phases;
    u32 value = 0x0;

    value = get_cached_registers(index, reg) & ~(0x00007f00 | 0x0000007f);
    value = value | ((v_num_phases << 8) | h_num_phases);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

    return 0;
}

static int set_coef_data(scaler_coef_data_t *data, unsigned int index)
{
    scaler_coef_row_t *coef_data = (scaler_coef_row_t *)data->coef_data;
    unsigned int i = 0, j = 0;
    unsigned int count = 0;
    u16 reg = handles[index].coef_data_in;
    u32 value = 0;

    mutex_lock(&locks[index]);
    for (i = 0; i < SCALER_COEF_DATA_ROW_NUM;  i++) {
        for (j = 0; j < SCALER_COEF_DATA_COL_NUM; j++)
            cached_coef_data[index][i * SCALER_COEF_DATA_COL_NUM + j] = (unsigned short )coef_data[i][j];
    }

    count = SCALER_COEF_DATA_ROW_NUM * SCALER_COEF_DATA_COL_NUM;

    for (i = 0; i < count; i+=2) {
        value = get_cached_registers(index, reg) & ~(0xffff0000 | 0x0000ffff);
        value = value |  ((cached_coef_data[index][i + 1] << 16) | cached_coef_data[index][i] );
        fpga_reg_write(handles[index].base, reg, value);
        set_cached_registers(index, reg, value);
    }
    mutex_unlock(&locks[index]);

    return 0;
}


static int set_shrink_factor(scaler_shrink_factor_t *factor,  unsigned int  index)
{

    unsigned int hsf = factor->hsf;
    unsigned int vsf = factor->vsf;

    u32 value = 0;
    u16 reg = 0;

    reg = handles[index].hsf;

    value = get_cached_registers(index, reg) & ~(0x00ffffff);
    value = value | (hsf);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);


    reg = handles[index].vsf;

    value = get_cached_registers(index, reg) & ~(0x00ffffff);
    value = value | (vsf);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
scaler_cfg_reg scaler_cached_registers_0[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0004, 0x00000000}, //status
    {0x0008, 0x00000000}, //error
    {0x000c, 0x00000000}, //irq_enable
    {0x0010, 0x00000000}, //version
    {0x0100, 0x00000000}, //hsf
    {0x0104, 0x00000000}, //vsf
    {0x0108, 0x00000000}, //source_video_size
    {0x010c, 0x00000000}, //h_aperture
    {0x0110, 0x00000000}, //v_aperture
    {0x0114, 0x00000000}, //output_size
    {0x0118, 0x00000000}, //num_phases
    {0x011c, 0x00000000}, //active_coefs
    {0x0120, 0x00000000}, //hpa_y
    {0x0124, 0x00000000}, //hpa_c
    {0x0128, 0x00000000}, //vpa_c
    {0x012c, 0x00000000}, //vpa_c
    {0x0130, 0x00000000}, //coef_set_wr_addr
    {0x0134, 0x00000000}, //coef_data_in
    {0x0138, 0x00000000}, //coef_set_bank_rd_addr
    {0x013c, 0x00000000}, //coef_mem_rd_addr
    {0x0140, 0x00000000}, //coef_mem_output
};
scaler_cfg_reg scaler_cached_registers_1[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0004, 0x00000000}, //status
    {0x0008, 0x00000000}, //error
    {0x000c, 0x00000000}, //irq_enable
    {0x0010, 0x00000000}, //version
    {0x0100, 0x00000000}, //hsf
    {0x0104, 0x00000000}, //vsf
    {0x0108, 0x00000000}, //source_video_size
    {0x010c, 0x00000000}, //h_aperture
    {0x0110, 0x00000000}, //v_aperture
    {0x0114, 0x00000000}, //output_size
    {0x0118, 0x00000000}, //num_phases
    {0x011c, 0x00000000}, //active_coefs
    {0x0120, 0x00000000}, //hpa_y
    {0x0124, 0x00000000}, //hpa_c
    {0x0128, 0x00000000}, //vpa_c
    {0x012c, 0x00000000}, //vpa_c
    {0x0130, 0x00000000}, //coef_set_wr_addr
    {0x0134, 0x00000000}, //coef_data_in
    {0x0138, 0x00000000}, //coef_set_bank_rd_addr
    {0x013c, 0x00000000}, //coef_mem_rd_addr
    {0x0140, 0x00000000}, //coef_mem_output
};
scaler_cfg_reg scaler_cached_registers_2[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0004, 0x00000000}, //status
    {0x0008, 0x00000000}, //error
    {0x000c, 0x00000000}, //irq_enable
    {0x0010, 0x00000000}, //version
    {0x0100, 0x00000000}, //hsf
    {0x0104, 0x00000000}, //vsf
    {0x0108, 0x00000000}, //source_video_size
    {0x010c, 0x00000000}, //h_aperture
    {0x0110, 0x00000000}, //v_aperture
    {0x0114, 0x00000000}, //output_size
    {0x0118, 0x00000000}, //num_phases
    {0x011c, 0x00000000}, //active_coefs
    {0x0120, 0x00000000}, //hpa_y
    {0x0124, 0x00000000}, //hpa_c
    {0x0128, 0x00000000}, //vpa_c
    {0x012c, 0x00000000}, //vpa_c
    {0x0130, 0x00000000}, //coef_set_wr_addr
    {0x0134, 0x00000000}, //coef_data_in
    {0x0138, 0x00000000}, //coef_set_bank_rd_addr
    {0x013c, 0x00000000}, //coef_mem_rd_addr
    {0x0140, 0x00000000}, //coef_mem_output
};
scaler_cfg_reg scaler_cached_registers_3[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0004, 0x00000000}, //status
    {0x0008, 0x00000000}, //error
    {0x000c, 0x00000000}, //irq_enable
    {0x0010, 0x00000000}, //version
    {0x0100, 0x00000000}, //hsf
    {0x0104, 0x00000000}, //vsf
    {0x0108, 0x00000000}, //source_video_size
    {0x010c, 0x00000000}, //h_aperture
    {0x0110, 0x00000000}, //v_aperture
    {0x0114, 0x00000000}, //output_size
    {0x0118, 0x00000000}, //num_phases
    {0x011c, 0x00000000}, //active_coefs
    {0x0120, 0x00000000}, //hpa_y
    {0x0124, 0x00000000}, //hpa_c
    {0x0128, 0x00000000}, //vpa_c
    {0x012c, 0x00000000}, //vpa_c
    {0x0130, 0x00000000}, //coef_set_wr_addr
    {0x0134, 0x00000000}, //coef_data_in
    {0x0138, 0x00000000}, //coef_set_bank_rd_addr
    {0x013c, 0x00000000}, //coef_mem_rd_addr
    {0x0140, 0x00000000}, //coef_mem_output
};
////////////////////////////////////////////////////////////////////////////////////////////
