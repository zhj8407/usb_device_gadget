#include <linux/mutex.h>
#include "zynq_osd.h"
//#include "zynq_regs.h"
#include "../zynq_fpga_verify.h"
#include "../zynq_debug.h"
#include "../zynq_types.h"

typedef struct {
    u16 offset;
    u32 value;
} osd_cfg_reg;

typedef struct {
    void __iomem *base;
    u16 ctrl;
    u16 active_output_size;
    u16 layer_0_ctrl;
    u16 layer_0_position;
    u16 layer_0_size;
    u16 layer_1_ctrl;
    u16 layer_1_position;
    u16 layer_1_size;
    u32 width;
    u32 height;
} osd_handle_t;

#define OSD_REG_NUM  (8)
extern osd_cfg_reg osd_cached_registers_0[];
extern osd_cfg_reg osd_cached_registers_1[];


static osd_handle_t  handles[OSD_MAX_NUM];
static struct mutex locks[OSD_MAX_NUM];
static  osd_cfg_reg  *cached_registers[OSD_MAX_NUM];


static int sw_reset(unsigned int  index);
static int set_update_change(osd_enable_t *userdata, unsigned int index);
static int set_enable(osd_enable_t *userdata, unsigned int index);
static int set_active_output_size(osd_size_t *size, unsigned int index);
static int set_layer(osd_layer_paramter_t *p, unsigned int index);

static void set_layer_ctrl(unsigned int index,  EOSDLayerID id, unsigned int enable, unsigned int global_alpha_enable, unsigned int priority, unsigned int alpha_value);
static 	void set_layer_position(unsigned int index,  EOSDLayerID id, unsigned int position_x,  unsigned int position_y);
static void set_layer_size(unsigned int index,  EOSDLayerID id, unsigned int width, unsigned int height);

static u8 is_initialized[OSD_MAX_NUM] = {0};
static u8 is_started[OSD_MAX_NUM] = {0};
static u8 is_reset[OSD_MAX_NUM] = {0};

static u32 get_cached_registers(unsigned int index, u16 reg)
{
    unsigned int count  = OSD_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg)
            return cached_registers[index][i].value;

    return 0;
}

static int set_cached_registers(unsigned int index, u16 reg, u32 value)
{
    unsigned int count  = OSD_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg) {
            cached_registers[index][i].value = value;
            return 0;
        }

    return -1;
}

void osd_get_status(osd_status_t *st, unsigned index)
{
    st->is_initialized = is_initialized[index];
    st->is_started = is_started[index];
}

int osd_initial_by_index(void __iomem *pci_base_addr, unsigned index)
{

    unsigned i = index;

    if (i >= OSD_MAX_NUM) return  0;

    if (is_initialized[i] == 1) return  0;

    if (i == 0)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_OSD0_REG);
    else if (i == 1)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_OSD1_REG);

    handles[i].ctrl = 0x0000;
    handles[i].active_output_size = 0x0020;
    handles[i].layer_0_ctrl = 0x0110;
    handles[i].layer_0_position = 0x0114;
    handles[i].layer_0_size = 0x0118;
    handles[i].layer_1_ctrl = 0x0120;
    handles[i].layer_1_position = 0x0124;
    handles[i].layer_1_size = 0x0128;
    mutex_init(&locks[i]);
    if (i == 0)
        cached_registers[i] = &osd_cached_registers_0[0];
    else  if (i == 1)
        cached_registers[i] = &osd_cached_registers_1[0];

    sw_reset(i);

    set_cached_registers(i, handles[i].ctrl , fpga_reg_read(handles[i].base , handles[i].ctrl ));
    set_cached_registers(i, handles[i].active_output_size, fpga_reg_read(handles[i].base ,handles[i].active_output_size ));

    set_cached_registers(i, handles[i].layer_0_ctrl , fpga_reg_read(handles[i].base , handles[i].layer_0_ctrl));
    set_cached_registers(i, handles[i].layer_0_position , fpga_reg_read(handles[i].base , handles[i].layer_0_position ));
    set_cached_registers(i, handles[i].layer_0_size, fpga_reg_read(handles[i].base , handles[i].layer_0_size ));

    set_cached_registers(i, handles[i].layer_1_ctrl , fpga_reg_read(handles[i].base , handles[i].layer_1_ctrl));
    set_cached_registers(i, handles[i].layer_1_position , fpga_reg_read(handles[i].base , handles[i].layer_1_position ));
    set_cached_registers(i, handles[i].layer_1_size, fpga_reg_read(handles[i].base , handles[i].layer_1_size ));

    is_initialized[i] = 1;
    is_reset[i] = 1;

    return 0;

}

int osd_initial(void __iomem *pci_base_addr)
{

    int i = 0;

    for (i = 0; i < OSD_MAX_NUM; i++) {

        if (is_initialized[i] == 1) continue;
        zynq_printk(1, "[zynq_osd](%d)%d \n", __LINE__,i);
        if (i == 0)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_OSD0_REG);
        else if (i == 1)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_OSD1_REG);

        handles[i].ctrl = 0x0000;
        handles[i].active_output_size = 0x0020;
        handles[i].layer_0_ctrl = 0x0110;
        handles[i].layer_0_position = 0x0114;
        handles[i].layer_0_size = 0x0118;
        handles[i].layer_1_ctrl = 0x0120;
        handles[i].layer_1_position = 0x0124;
        handles[i].layer_1_size = 0x0128;
        mutex_init(&locks[i]);
        if (i == 0)
            cached_registers[i] = &osd_cached_registers_0[0];
        else  if (i == 1)
            cached_registers[i] = &osd_cached_registers_1[0];

        sw_reset(i);

        set_cached_registers(i, handles[i].ctrl , fpga_reg_read(handles[i].base , handles[i].ctrl ));
        set_cached_registers(i, handles[i].active_output_size, fpga_reg_read(handles[i].base ,handles[i].active_output_size ));

        set_cached_registers(i, handles[i].layer_0_ctrl , fpga_reg_read(handles[i].base , handles[i].layer_0_ctrl));
        set_cached_registers(i, handles[i].layer_0_position , fpga_reg_read(handles[i].base , handles[i].layer_0_position ));
        set_cached_registers(i, handles[i].layer_0_size, fpga_reg_read(handles[i].base , handles[i].layer_0_size ));

        set_cached_registers(i, handles[i].layer_1_ctrl , fpga_reg_read(handles[i].base , handles[i].layer_1_ctrl));
        set_cached_registers(i, handles[i].layer_1_position , fpga_reg_read(handles[i].base , handles[i].layer_1_position ));
        set_cached_registers(i, handles[i].layer_1_size, fpga_reg_read(handles[i].base , handles[i].layer_1_size ));

        is_initialized[i] = 1;
        is_reset[i] = 1;
        zynq_printk(1, "[zynq_osd](%d)%d \n", __LINE__,i);
        //zynq_printk(1, "[zynq_osd] Register base address for osd %d : 0x%8lx \n", i,   (unsigned long)handles[i].base);
    }

    return 0;
}

int osd_setoption( EOSDOptionFlags flag, void *userdata, unsigned  index)
{

    if (index >= OSD_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    switch (flag) {
        case OSD_OPTION_RESET:
            sw_reset(index);
            break;
        case OSD_OPTION_UPDATE_CHANGE:
            set_update_change((osd_enable_t *)userdata, index);
            break;
        case  OSD_OPTION_ENABLE:
            set_enable((osd_enable_t *)userdata, index);
            break;
        case OSD_OPTION_SET_OUTPUT_ACTIVE_SIZE:
            set_active_output_size((osd_size_t *)userdata, index);
            break;
        case OSD_OPTION_SET_LAYER:
            set_layer((osd_layer_paramter_t *)userdata, index);
            break;
        default:
            break;
    }
    return 0;
}

int osd_reset(unsigned index)
{

    if (index >= OSD_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    return sw_reset(index);
}

int osd_start(unsigned index)
{
    osd_enable_t enable;

    if (index >= OSD_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    if (is_started[index] == 1) return 0;

    enable.value = 1;
    set_enable(&enable, index);

    mutex_lock(&locks[index]);
    is_started[index] = 1;
    mutex_unlock(&locks[index]);

    return 0;
}

int osd_stop(unsigned index)
{
    osd_enable_t enable;

    if (index >= OSD_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    if (is_started[index] == 0) return 0;

    enable.value = 0;
    set_enable(&enable, index);

    mutex_lock(&locks[index]);
    is_started[index] = 0;
    mutex_unlock(&locks[index]);

    return 0;
}

int osd_release(void __iomem *pci_base_addr)
{
    int i = 0;
    for (i = 0; i < OSD_MAX_NUM; i++) {

        if (is_initialized[i] == 0) continue;

        mutex_lock(&locks[i]);
        is_initialized[i] = 0;
        mutex_unlock(&locks[i]);

        mutex_destroy(&locks[i]);
    }
    return 0;
}

void osd_dump_registers(unsigned index)
{
    unsigned int i = 0;
    unsigned int count  = OSD_REG_NUM;

    for (i  = 0; i < count; i ++ ) {
        zynq_printk(1, "[0x%02x] = 0x%08x \n", cached_registers[index][i].offset, cached_registers[index][i].value);
    }
}

int osd_enable_reg_update(unsigned index)
{
#if 0
    osd_enable_t enable;

    if (index >= OSD_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    enable.value = 0x1;
    set_update_change(&enable, index);
#endif
    return 0;
}

int  osd_disable_reg_update(unsigned index)
{
#if 0
    osd_enable_t enable;

    if (index >= OSD_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    enable.value = 0x0;
    set_update_change(&enable, index);
#endif
    return  0;
}

int osd_get_regs(unsigned index, void *regs_ptr)
{

    vpif_video_cfg_regs_t  *regs = (vpif_video_cfg_regs_t *)regs_ptr;

    if (!regs) return -1;

    regs->regs = (vpif_video_cfg_reg_t *)(&cached_registers[index][0]);
    regs->num = OSD_REG_NUM;

    return 0;

}

int osd_set_reg(unsigned index, u16 offset, u32 value)
{

    if (index >= OSD_MAX_NUM) return  -1;

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, offset, value);
    set_cached_registers(index, offset, value);
    mutex_unlock(&locks[index]);

    return 0;
}

int osd_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height)
{

    osd_size_t active_output_size;
    osd_layer_paramter_t param;

    if (is_initialized[index] == 0) return 0;

    if ((in_width == handles[index].width) && (in_height == handles[index].height) )  return 0;


    active_output_size.width = in_width;
    active_output_size.height = in_height;
    set_active_output_size(&active_output_size, index);

    param.id = OSD_LAYER_0;
    param.enable = 1;
    param.global_alpha_enable = 1;
    param.priority = 0;
    param.alpha_value = 0x100;
    param.position_x = 0;
    param.position_y = 0;
    param.width = in_width;
    param.height = in_height;
    set_layer(&param, index);

    param.id = OSD_LAYER_1;
    param.enable = 1;
    param.global_alpha_enable = 1;
    param.priority = 1;
    param.alpha_value = 0x100;
    if ((in_width == 1920) && (in_height == 1080)) {
        param.position_x =  1440;
        param.position_y = 810;
    } else {
        param.position_x =  960;
        param.position_y = 540;
    }
    param.width = in_width >> 2;
    param.height = in_height >> 2;
    set_layer(&param, index);

    return 0;
}



///////////////////////////////////////////////////////////
static int sw_reset(unsigned int  index)
{
    u32 value = get_cached_registers(index, handles[index].ctrl) | 0x80000000;

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
    return  0;

}

static int set_update_change(osd_enable_t *enable, unsigned int index)
{
#if 1
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

static int set_enable(osd_enable_t *enable, unsigned int index)
{
    u32 value= 0x0;

    if (enable->value) {
        //value = get_cached_registers(index, handles[index].ctrl) | 0x00000003;
        value = get_cached_registers(index, handles[index].ctrl) | 0x00000001;
    } else  {
        value = get_cached_registers(index, handles[index].ctrl) & ~(0x00000001);
    }
    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
    return 0;
}

static int set_active_output_size(osd_size_t *size, unsigned int index)
{
    u32 value = 0x00000000;
    u16 reg = handles[index].active_output_size;
    u32  width = size->width;
    u32 height = size->height;

    if (width != handles[index].width) handles[index].width = width;

    if (height != handles[index].height) handles[index].height = height;

    value = get_cached_registers(index, reg) & ~(0x0fff0000 | 0x00000fff);
    value = value | ((height << 16) | width);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

    return 0;

}

static int set_layer(osd_layer_paramter_t *p, unsigned int index)
{


    if (!p) return -1;

    set_layer_ctrl(index, p->id, p->enable, p->global_alpha_enable, p->priority, p->alpha_value);
    set_layer_position(index, p->id, p->position_x, p->position_y);
    set_layer_size(index, p->id, p->width, p->height);

    return  0;
}

static void set_layer_ctrl(unsigned int index,  EOSDLayerID id, unsigned int enable, unsigned int global_alpha_enable, unsigned int priority, unsigned int alpha_value)
{

    u16 reg = 0x0;
    u32 value = 0x0;

    if (id == OSD_LAYER_0)
        reg = handles[index].layer_0_ctrl;
    else if (id == OSD_LAYER_1)
        reg = handles[index].layer_1_ctrl;
    else
        return;

    value = get_cached_registers(index, reg) & ~(0x01ff0703);
    value = value  | ( (alpha_value  << 16) | (priority << 8) | (global_alpha_enable << 1) | (enable));

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

}

static void set_layer_position(unsigned int index,  EOSDLayerID id, unsigned int position_x,  unsigned int position_y)
{


    u32 value = 0x00000000;
    u16 reg = 0x0;
    u32  x = position_x;
    u32  y  = position_y;

    if (id == OSD_LAYER_0)
        reg = handles[index].layer_0_position;
    else if (id == OSD_LAYER_1)
        reg = handles[index].layer_1_position;
    else
        return;


    value = get_cached_registers(index, reg) & ~(0x0fff0000 | 0x00000fff);
    value = value | ((y << 16) | x);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);
}

static void set_layer_size(unsigned int index,  EOSDLayerID id, unsigned int width, unsigned int height)
{

    u32 value = 0x00000000;
    u16 reg = 0x0;

    if (id == OSD_LAYER_0)
        reg = handles[index].layer_0_size;
    else if (id == OSD_LAYER_1)
        reg = handles[index].layer_1_size;
    else
        return;

    value = get_cached_registers(index, reg) & ~(0x0fff0000 | 0x00000fff);
    value = value | ((height << 16) | width);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);
}


/////////////////////////////////////////////////////////
osd_cfg_reg osd_cached_registers_0[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0020, 0x00000000}, //active_output_size
    {0x0110, 0x00000000}, //layer_0_ctrl
    {0x0114, 0x00000000}, //layer_0_position
    {0x0118, 0x00000000}, //layer_0_size
    {0x0120, 0x00000000}, //layer_1_ctrl
    {0x0124, 0x00000000}, //layer_1_position
    {0x0128, 0x00000000}, //layer_1_size
};

osd_cfg_reg osd_cached_registers_1[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0020, 0x00000000}, //active_output_size
    {0x0110, 0x00000000}, //layer_0_ctrl
    {0x0114, 0x00000000}, //layer_0_position
    {0x0118, 0x00000000}, //layer_0_size
    {0x0120, 0x00000000}, //layer_1_ctrl
    {0x0124, 0x00000000}, //layer_1_position
    {0x0128, 0x00000000}, //layer_1_size
};

////////////////////////////////////////////////////////