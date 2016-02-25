#include <linux/mutex.h>
#include "zynq_video_selector.h"
//#include "zynq_regs.h"
#include "../zynq_fpga_verify.h"
#include "../zynq_debug.h"
#include "../zynq_types.h"

typedef struct {
    u16 offset;
    u32 value;
} vselector_cfg_reg;

#define VSELECTOR_REG_NUM  (6)
extern vselector_cfg_reg  vselector_cached_registers[];

typedef struct {
    void __iomem *base;
    u16 vout_frame_size;//0x0000
    u16 vout0_full_src;//0x0004
    u16 vout1_full_src; //0x0008
    u16 vout0_1_16_src;//0x000c
    u16 vout1_1_16_src;//0x0010
    u16 version;//0x0014
} vselector_handle_t;

static vselector_handle_t handle;
static struct mutex lock;

static int sw_reset(void);
static int set_enable(vselector_enable_t *userdata);
static int set_source(EVSelectorOptionFlags flag, vselector_source_t *userdata);
static int set_vout_framesize(vselector_vout_frame_size_t *size);
static u8 is_initialized = 0;
static u8 is_started = 0;

static u32 get_cached_registers(u16 reg)
{
    unsigned int count  = VSELECTOR_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (vselector_cached_registers[i].offset == reg)
            return vselector_cached_registers[i].value;

    return 0;
}

static int set_cached_registers(u16 reg, u32 value)
{
    unsigned int count  = VSELECTOR_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (vselector_cached_registers[i].offset == reg) {
            vselector_cached_registers[i].value = value;
            return 0;
        }
    return -1;
}

void vselector_get_status(vselector_status_t *st)
{
    st->is_initialized = is_initialized;
    st->is_started = is_started;
}

int vselector_initial(void __iomem *pci_base_addr)
{

    if (is_initialized == 1) return 0;

    handle.base = (void __iomem *)((u32)pci_base_addr+ FPGA_VSELECTOR_REG);
    handle.vout_frame_size =0x0000;
    handle.vout0_full_src = 0x0004;
    handle.vout1_full_src = 0x0008;
    handle.vout0_1_16_src = 0x000c;
    handle.vout1_1_16_src = 0x0010;
    handle.version = 0x0014;
    mutex_init(&lock);

    sw_reset();
#if 1
    set_cached_registers(handle.vout_frame_size , fpga_reg_read(handle.base , handle.vout_frame_size));
    set_cached_registers(handle.vout0_full_src , fpga_reg_read(handle.base , handle.vout0_full_src));
    set_cached_registers(handle.vout1_full_src , fpga_reg_read(handle.base , handle.vout1_full_src));
    set_cached_registers(handle.vout0_1_16_src  , fpga_reg_read(handle.base , handle.vout0_1_16_src ));
    set_cached_registers(handle.vout1_1_16_src  , fpga_reg_read(handle.base , handle.vout1_1_16_src ));
    set_cached_registers(handle.version , fpga_reg_read(handle.base , handle.version));
#endif
    is_initialized = 1;

    //zynq_printk(1, "[zynq_video_selector]xxx Register base address for vselector : 0x%8lx \n",  (unsigned long)handle.base);

    return 0;
}

int vselector_setoption(EVSelectorOptionFlags flag, void *userdata)
{

    if (is_initialized == 0) return 0;

    switch (flag) {
        case VSELECTOR_OPTION_ENABLE:
            set_enable((vselector_enable_t *)userdata);
            break;
        case VSELECTOR_OPTION_SET_VOUT0_FULL_SRC:
        case VSELECTOR_OPTION_SET_VOUT1_FULL_SRC:
        case VSELECTOR_OPTION_SET_VOUT0_1_16_SRC:
        case VSELECTOR_OPTION_SET_VOUT1_1_16_SRC:
            set_source(flag, (vselector_source_t *)userdata);
            break;
        case VSELECTOR_OPTION_SET_VOUT_FRAME_SIZE:
            set_vout_framesize((vselector_vout_frame_size_t *)userdata);
            break;
        default:
            break;
    }

    return 0;
}

int vselector_reset()
{

    if (is_initialized == 0) return 0;

    return sw_reset();
}

int vselector_start()
{

    vselector_enable_t enable;

    if (is_initialized == 0) return 0;

    if (is_started == 1) return 0;

    enable.value = 0x1;

    mutex_lock(&lock);
    is_started = 1;
    mutex_unlock(&lock);

    return set_enable(&enable);
}

int vselector_stop()
{

    vselector_enable_t enable;

    if (is_initialized == 0) return 0;

    if (is_started == 0) return 0;

    enable.value = 0x0;

    mutex_lock(&lock);
    is_started = 0;
    mutex_unlock(&lock);

    return set_enable(&enable);
}

int vselector_release(void __iomem *pci_base_addr)
{


    if (is_initialized == 0) return 0;

    mutex_lock(&lock);
    is_initialized = 0;
    mutex_unlock(&lock);

    mutex_destroy(&lock);

    return 0;
}

int vselector_enable_reg_update(void)
{
    return 0;
}

int  vselector_disable_reg_update(void)
{
    return 0;
}

void vselector_dump_registers()
{
    unsigned int i = 0;
    unsigned int count  =  VSELECTOR_REG_NUM;

    for (i  = 0; i < count; i ++ ) {
        zynq_printk(1, "[0x%02x] = 0x%08x \n", vselector_cached_registers[i].offset, vselector_cached_registers[i].value);
    }
    return;
}

int vselector_get_regs(void *regs_ptr)
{

    vpif_video_cfg_regs_t  *regs = (vpif_video_cfg_regs_t *)regs_ptr;

    if (!regs) return -1;

    regs->regs = (vpif_video_cfg_reg_t *)(&vselector_cached_registers[0]);
    regs->num = VSELECTOR_REG_NUM;

    return 0;
}

int vselector_set_reg(u16 offset, u32 value)
{

    mutex_lock(&lock);
#if 1
    fpga_reg_write(handle.base, offset, value);
#endif
    zynq_printk(0, "[zynq_video_selector](%d) (reg, vlaue)--->(0x%08x, 0x%08x)\n", __LINE__, offset, value);
    set_cached_registers(offset, value);
    mutex_unlock(&lock);

    return 0;
}

/////////////////////////////////////////////////////////////////////////
static int sw_reset(void)
{
    return 0;
}

static int set_enable(vselector_enable_t *enable)
{
    //TODO:There is no enable/disbale control for  video selector module ??
    return 0;
}

static int set_source(EVSelectorOptionFlags flag, vselector_source_t *src)
{

    u16 reg = (u16)-1;
    u32 value =  0x0;

    switch (flag) {
        case VSELECTOR_OPTION_SET_VOUT0_FULL_SRC:
            reg = handle.vout0_full_src;
            break;
        case VSELECTOR_OPTION_SET_VOUT1_FULL_SRC:
            reg = handle.vout1_full_src;
            break;
        case VSELECTOR_OPTION_SET_VOUT0_1_16_SRC:
            reg = handle.vout0_1_16_src;
            break;
        case VSELECTOR_OPTION_SET_VOUT1_1_16_SRC:
            reg = handle.vout1_1_16_src;
            break;
        default:
            return -1;
    }
    value = get_cached_registers(reg) & ~( 0x0000000f );
    value = value | ( (src->cpu << 3)  | (src->vin2 << 2)  | (src->vin1 << 1) | (src->vin0 << 0) );

    mutex_lock(&lock);
#if 1
    fpga_reg_write(handle.base, reg, value);
#endif
//	zynq_printk(0, "[zynq_video_selector](%d) (reg, vlaue)--->(0x%08x, 0x%08x)\n", __LINE__, reg, value);
    set_cached_registers(reg,value);
    mutex_unlock(&lock);

    return 0;
}

static int set_vout_framesize(vselector_vout_frame_size_t *size)
{

    u32 value = 0x00000000;
    u16 reg = handle.vout_frame_size;
    u32  width = size->width;
    u32 height = size->height;

    if (!size) return  0;

    value = get_cached_registers(reg) & ~( 0x1fff0000 | 0x00001fff);
    value = value  | ((height << 16) | width);

    mutex_lock(&lock);
#if 1
    fpga_reg_write(handle.base, reg, value);
#endif
//	zynq_printk(0, "[zynq_video_selector](%d) (reg, vlaue)--->(0x%08x, 0x%08x)\n", __LINE__, reg, value);
    set_cached_registers(reg, value);
    mutex_unlock(&lock);
    return 0;
}

////////////////////////////////////////////////////////////////////////
vselector_cfg_reg  vselector_cached_registers[] = {
    {0x0000, 0x00000000}, //vout_frame_size
    {0x0004, 0x00000000}, //vout_0_full_source
    {0x0008, 0x00000000}, //vout_1_full_source
    {0x000c, 0x00000000}, //vout_0_1/16_source
    {0x0010, 0x00000000}, //vout_1_1/16_source
    {0x0014, 0x00000000}, //version
};
/////////////////////////////////////////////////////////////////////////