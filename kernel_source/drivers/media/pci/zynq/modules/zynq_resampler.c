#include <linux/mutex.h>
#include "zynq_resampler.h"
//#include "zynq_regs.h"
#include "../zynq_fpga_verify.h"
#include "../zynq_debug.h"
#include "../zynq_types.h"

typedef struct {
    u16 offset;
    u32 value;
} resampler_cfg_reg;

typedef struct {
    void __iomem *base;
    u16 ctrl;
    u16 active_size;
    u32 width;
    u32 height;
} resampler_handle_t;

#define RESANPLER_REG_NUM  (2)
extern resampler_cfg_reg resampler_cached_registers_0[];
extern resampler_cfg_reg resampler_cached_registers_1[];
extern resampler_cfg_reg resampler_cached_registers_2[];
extern resampler_cfg_reg resampler_cached_registers_3[];
extern resampler_cfg_reg resampler_cached_registers_4[];

static resampler_handle_t handles[RESAMPLER_MAX_NUM];
static struct mutex locks[RESAMPLER_MAX_NUM];
static resampler_cfg_reg *cached_registers[RESAMPLER_MAX_NUM];

static int sw_reset(unsigned int  index);
static int set_update_change(resampler_enable_t *userdata, unsigned int index);
static int set_bypass(resampler_enable_t *userdata, unsigned int index);
static int set_enable(resampler_enable_t *userdata, unsigned int index);
static int set_active_size(resampler_size_t *size, unsigned int index);

static u8 is_initialized[RESAMPLER_MAX_NUM] = {0};
static u8 is_started[RESAMPLER_MAX_NUM] = {0};

static u32 get_cached_registers(unsigned int index, u16 reg)
{
    unsigned int count  = RESAMPLER_MAX_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg)
            return cached_registers[index][i].value;

    return 0;
}

static int set_cached_registers(unsigned int index, u16 reg, u32 value)
{
    unsigned int count  = RESAMPLER_MAX_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg) {
            cached_registers[index][i].value = value;
            return 0;
        }

    return -1;
}

void resampler_get_status(resampler_status_t *st, unsigned index)
{
    st->is_initialized = is_initialized[index];
    st->is_started = is_started[index];
}

int resampler_initial_by_index(void __iomem *pci_base_addr, unsigned index)
{
    unsigned  i = index;

    if (i >= RESAMPLER_MAX_NUM) return 0;

    if (i == 4) return 0;

    if (is_initialized[i] == 1) return  0;

    if (i == 0)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER0_REG);
    else if (i == 1)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER1_REG);
    else if (i == 2)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER2_REG);
    else if (i == 3)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER3_REG);
    else if (i == 4)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER4_REG);

    handles[i].ctrl = 0x0000;
    handles[i].active_size = 0x0020;
    mutex_init(&locks[i]);

    if (i == 0)
        cached_registers[i] = &resampler_cached_registers_0[0];
    else  if (i == 1)
        cached_registers[i] = &resampler_cached_registers_1[0];
    else if (i == 2)
        cached_registers[i] = &resampler_cached_registers_2[0];
    else if (i == 3)
        cached_registers[i] = &resampler_cached_registers_3[0];
    else if (i == 4)
        cached_registers[i] = &resampler_cached_registers_4[0];

    sw_reset(i);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg)--->(0x%08x, 0x%04x)\n", __LINE__, i, handles[i].base, handles[i].ctrl);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg)--->(0x%08x, 0x%04x)\n", __LINE__, i, handles[i].base, handles[i].active_size);
    //set_cached_registers(i, handles[i].ctrl , fpga_reg_read(handles[i].base , handles[i].ctrl ));
    //set_cached_registers(i, handles[i].active_size , fpga_reg_read(handles[i].base , handles[i].active_size));
    is_initialized[i] = 1;

    return 0;
}

int resampler_initial(void __iomem *pci_base_addr)
{
    int i = 0;
    for (i = 0; i < RESAMPLER_MAX_NUM; i++) {

        if (i == 4)  return 0;

        if (is_initialized[i] == 1) continue;

        if (i == 0)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER0_REG);
        else if (i == 1)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER1_REG);
        else if (i == 2)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER2_REG);
        else if (i == 3)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER3_REG);
        else if (i == 4)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_CHROMA_RESAMPLER4_REG);

        handles[i].ctrl = 0x0000;
        handles[i].active_size = 0x0020;
        mutex_init(&locks[i]);
        if (i == 0)
            cached_registers[i] = &resampler_cached_registers_0[0];
        else  if (i == 1)
            cached_registers[i] = &resampler_cached_registers_1[0];
        else if (i == 2)
            cached_registers[i] = &resampler_cached_registers_2[0];
        else if (i == 3)
            cached_registers[i] = &resampler_cached_registers_3[0];
        else if (i == 4)
            cached_registers[i] = &resampler_cached_registers_4[0];

        sw_reset(i);
        //set_cached_registers(i, handles[i].ctrl , fpga_reg_read(handles[i].base , handles[i].ctrl ));
        //set_cached_registers(i, handles[i].active_size , fpga_reg_read(handles[i].base , handles[i].active_size));
        is_initialized[i] = 1;
        //zynq_printk(1, "[zynq_resampler] Register base address for resampler %d : 0x%8lx \n", i,   (unsigned long)handles[i].base);
    }
    return 0;
}

int resampler_setoption( EResamplerOptionFlags flag, void *userdata, unsigned  index)
{

    if (index >= RESAMPLER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    switch (flag) {
        case RESAMPLER_OPTION_RESET:
            sw_reset(index);
            break;
        case RESAMPLER_OPTION_UPDATE_CHANGE:
            set_update_change((resampler_enable_t *)userdata, index);
            break;
        case  RESAMPLER_OPTION_ENABLE:
            set_enable((resampler_enable_t *)userdata, index);
            break;
        case RESAMPLER_OPTION_BYPASS_ENABLE:
            set_bypass((resampler_enable_t *)userdata, index);
            break;
        case RESAMPLER_OPTION_SET_ACTIVE_SIZE:
            set_active_size((resampler_size_t *)userdata, index);
            break;
        default:
            break;
    }
    return 0;
}

int resampler_reset(unsigned index)
{

    if (index >= RESAMPLER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    return sw_reset(index);
}

int resampler_start(unsigned index)
{

    resampler_enable_t enable;

    if (index >= RESAMPLER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    if (is_started[index] == 1) return 0;

    enable.value = 1;
    set_enable(&enable, index);

    mutex_lock(&locks[index]);
    is_started[index] = 1;
    mutex_unlock(&locks[index]);

    return 0;
}

int resampler_stop(unsigned index)
{

    resampler_enable_t enable;

    if (index >= RESAMPLER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    if (is_started[index] == 0) return 0;

    enable.value = 0;
    set_enable(&enable, index);

    mutex_lock(&locks[index]);
    is_started[index] = 0;
    mutex_unlock(&locks[index]);

    return 0;
}

int resampler_release(void __iomem *pci_base_addr)
{
    int i = 0;
    for (i = 0; i < RESAMPLER_MAX_NUM; i++) {

        if (is_initialized[i] == 0) continue;

        mutex_lock(&locks[i]);
        is_initialized[i] = 0;
        mutex_unlock(&locks[i]);

        mutex_destroy(&locks[i]);
    }
    return 0;
}

void resampler_dump_registers(unsigned index)
{
    unsigned int i = 0;
    unsigned int count  = RESANPLER_REG_NUM;

    for (i  = 0; i < count; i ++ ) {
        zynq_printk(1, "[0x%02x] = 0x%08x \n", cached_registers[index][i].offset, cached_registers[index][i].value);
    }
}

int resampler_enable_reg_update(unsigned index)
{
    resampler_enable_t enable;

    if (index >= RESAMPLER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    enable.value = 0x1;
    set_update_change(&enable, index);

    return 0;
}

int  resampler_disable_reg_update(unsigned index)
{
    resampler_enable_t enable;

    if (index >= RESAMPLER_MAX_NUM) return  -1;

    if (is_initialized[index] == 0) return 0;

    enable.value = 0x0;
    set_update_change(&enable, index);

    return 0;
}

int resampler_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height)
{

    resampler_size_t size;

    if (is_initialized[index] == 0) return 0;

    if ((in_width == handles[index].width) && (in_height == handles[index].height) )  return 0;

    size.width = in_width;
    size.height = in_height;
    set_active_size(&size, index);

    return 0;
}

int resampler_get_regs(unsigned index, void *regs_ptr)
{

    vpif_video_cfg_regs_t  *regs = (vpif_video_cfg_regs_t *)regs_ptr;

    if (!regs) return -1;

    regs->regs = (vpif_video_cfg_reg_t *)(&cached_registers[index][0]);
    regs->num = RESANPLER_REG_NUM;

    return 0;

}

int resampler_set_reg(unsigned index, u16 offset, u32 value)
{

    if (index >= RESAMPLER_MAX_NUM) return  -1;

    mutex_lock(&locks[index]);
    //fpga_reg_write(handles[index].base, offset, value);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg, val)--->(0x%08x, 0x%04x, 0x%08x)\n", __LINE__, index, (unsigned int)handles[index].base,offset, value);
    set_cached_registers(index, offset, value);
    mutex_unlock(&locks[index]);

    return 0;
}

/////////////////////////////////////////////////////////////////////
static int set_active_size(resampler_size_t *size, unsigned int index)
{
    u32 value = 0x00000000;
    u16 reg = handles[index].active_size;
    u32  width = size->width;
    u32 height = size->height;

    if (width != handles[index].width) handles[index].width = width;

    if (height != handles[index].height) handles[index].height = height;

    value = get_cached_registers(index, reg) & ~(0x1fff0000 | 0x00001fff);
    value = value | ((height << 16) | width);

    mutex_lock(&locks[index]);
    //fpga_reg_write(handles[index].base, reg, value);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg, val)--->(0x%08x, 0x%04x, 0x%08x)\n", __LINE__, index, (unsigned int)handles[index].base,reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

    return 0;
}

static int set_update_change(resampler_enable_t *enable, unsigned int index)
{
    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(index, handles[index].ctrl) | 0x00000002;
    else
        value = get_cached_registers(index, handles[index].ctrl) & ~(0x00000002);

    mutex_lock(&locks[index]);
    //fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg, val)--->(0x%08x, 0x%04x, 0x%08x)\n", __LINE__, index, (unsigned int)handles[index].base,handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
    return 0;
}

static int set_bypass( resampler_enable_t *enable, unsigned int index)
{
    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(index, handles[index].ctrl) | 0x00000010;
    else
        value = get_cached_registers(index, handles[index].ctrl) & ~(0x00000010);

    mutex_lock(&locks[index]);
    //fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg, val)--->(0x%08x, 0x%04x, 0x%08x)\n", __LINE__, index, (unsigned int)handles[index].base,handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
    return 0;
}

static int set_enable( resampler_enable_t *enable, unsigned int index)
{
    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(index, handles[index].ctrl) | 0x00000003;
    else
        value = get_cached_registers(index, handles[index].ctrl) & ~(0x00000003);

    mutex_lock(&locks[index]);
    //fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg, val)--->(0x%08x, 0x%04x, 0x%08x)\n", __LINE__, index, (unsigned int)handles[index].base,handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
    return 0;

}

static int sw_reset(unsigned int  index)
{
    u32 value = get_cached_registers(index, handles[index].ctrl) | 0x80000000;

    mutex_lock(&locks[index]);
    //fpga_reg_write(handles[index].base, handles[index].ctrl, value);
    //zynq_printk(0, "[zynq_resampler](%d)[%d] (base, reg, val)--->(0x%08x, 0x%04x, 0x%08x)\n", __LINE__, index, (unsigned int)handles[index].base,handles[index].ctrl, value);
    set_cached_registers(index, handles[index].ctrl,value);
    mutex_unlock(&locks[index]);
    return  0;
}
/////////////////////////////////////////////////////////////////////
resampler_cfg_reg  resampler_cached_registers_0[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0020, 0x00000000}, //active_size
};
resampler_cfg_reg  resampler_cached_registers_1[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0020, 0x00000000}, //active_size
};
resampler_cfg_reg  resampler_cached_registers_2[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0020, 0x00000000}, //active_size
};
resampler_cfg_reg  resampler_cached_registers_3[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0020, 0x00000000}, //active_size
};
resampler_cfg_reg  resampler_cached_registers_4[] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0020, 0x00000000}, //active_size
};
/////////////////////////////////////////////////////////////////////