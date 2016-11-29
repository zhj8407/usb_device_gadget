#include <linux/mutex.h>
#include "zynq_video_timing_controller.h"
//#include "zynq_regs.h"
#include "../zynq_fpga_verify.h"
#include "../zynq_debug.h"
#include "../zynq_types.h"

typedef struct {
    u16 offset;
    u32 value;
} vtiming_cfg_reg;

typedef struct {
    unsigned int h_active_size;
    unsigned int h_frame_size;
    unsigned int h_sync_start;
    unsigned int h_sync_end;
    unsigned int v_active_size;
    unsigned int v_frame_size;
    unsigned int v_sync_start;
    unsigned int v_sync_end;
    unsigned int v_blanking_cycle_index_start;
    unsigned int v_blanking_cycle_index_end;
    unsigned int v_sync_cycle_index_start;
    unsigned int v_sync_cycle_index_end;
} vtiming_parametr_t;

static vtiming_parametr_t vtiming_parameter_1080p60 = {
    .h_active_size = 1920,
    .h_frame_size = 2200,
    .h_sync_start = 2008,
    .h_sync_end = 2052,
    .v_active_size = 1080,
    .v_frame_size = 1125,
    .v_sync_start = 1084,
    .v_sync_end = 1088,
    .v_blanking_cycle_index_start = 1920,
    .v_blanking_cycle_index_end = 1920,
    .v_sync_cycle_index_start = 1920,
    .v_sync_cycle_index_end = 1920
};

static vtiming_parametr_t vtiming_parameter_720p60 = {
    .h_active_size = 1280,
    .h_frame_size = 1650,
    .h_sync_start = 1390,
    .h_sync_end = 1430,
    .v_active_size = 720,
    .v_frame_size = 750,
    .v_sync_start = 725,
    .v_sync_end = 730,
    .v_blanking_cycle_index_start = 1280,
    .v_blanking_cycle_index_end = 1280,
    .v_sync_cycle_index_start = 1280,
    .v_sync_cycle_index_end = 1280
};


static vtiming_parametr_t vtiming_parameter_1080p50= {
    .h_active_size = 1920,
    .h_frame_size = 2640,
    .h_sync_start = 2448,
    .h_sync_end = 2492,
    .v_active_size = 1080,
    .v_frame_size = 1125,
    .v_sync_start = 1084,
    .v_sync_end = 1089,
    .v_blanking_cycle_index_start = 1920,
    .v_blanking_cycle_index_end = 1920,
    .v_sync_cycle_index_start = 1920,
    .v_sync_cycle_index_end = 1920
};

static vtiming_parametr_t vtiming_parameter_720p50 = {
    .h_active_size = 1280,
    .h_frame_size = 1980,
    .h_sync_start = 1720,
    .h_sync_end = 1760,
    .v_active_size = 720,
    .v_frame_size = 750,
    .v_sync_start = 725,
    .v_sync_end = 730,
    .v_blanking_cycle_index_start = 1280,
    .v_blanking_cycle_index_end = 1280,
    .v_sync_cycle_index_start = 1280,
    .v_sync_cycle_index_end = 1280
};

#define VTIMING_REG_NUM  (10)
extern  vtiming_cfg_reg  vtiming_cached_registers_0[];
extern  vtiming_cfg_reg  vtiming_cached_registers_1[];

typedef struct {
    void __iomem *base;
    u16 ctrl;
    u16 active_size;
    u16 format;
    u16 polarity;
    u16 hsize;
    u16 vsize;
    u16 hsync;
    u16 vblank_cycle_index;
    u16 vsync_line_index;
    u16 vsync_cycle_index;
} vtiming_handle_t;

static vtiming_handle_t handles[VIDEO_TIMING_CONTROLLER_MAX_NUM];
static struct mutex locks[VIDEO_TIMING_CONTROLLER_MAX_NUM];
static vtiming_cfg_reg *cached_registers[VIDEO_TIMING_CONTROLLER_MAX_NUM];

static int sw_reset(unsigned int index);
static int set_enable(vtiming_enable_t *userdata, unsigned int index);
static int set_update_change(vtiming_enable_t*userdata, unsigned int index);
static int set_format(vtiming_fromat_t *userdata, unsigned int index);
static int set_size(vtiming_size_t *userdata, unsigned int index);

static void set_active_size (unsigned int h_active_size, unsigned int v_active_size, unsigned int index);
static  void set_h_frame_size(unsigned int h_frame_size, unsigned int index);
static void set_v_frame_size(unsigned int v_frame_size, unsigned int index);
static void set_h_sync(unsigned int h_sync_start, unsigned int h_sync_end, unsigned index);
static void set_vsync_line_index(unsigned int v_sync_start, unsigned int v_sync_end, unsigned index);
static void set_vsync_cycle_index(unsigned int start_index, unsigned int end_index, unsigned index);
static void set_vblank_cycle_index(unsigned int start_index, unsigned int end_index, unsigned index);

static u8 is_initialized[VIDEO_TIMING_CONTROLLER_MAX_NUM] = {0};
static u8 is_started[VIDEO_TIMING_CONTROLLER_MAX_NUM] = {0};

static u32 get_cached_registers(unsigned int index, u16 reg)
{
    unsigned int count  = VTIMING_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg)
            return cached_registers[index][i].value;

    return 0;
}

static int set_cached_registers(unsigned int index, u16 reg, u32 value)
{
    unsigned int count  = VTIMING_REG_NUM;
    unsigned int  i = 0;
    for (i = 0; i < count; i++)
        if (cached_registers[index][i].offset == reg) {
            cached_registers[index][i].value = value;
            return 0;
        }

    return -1;
}

void vtiming_get_status(vtiming_status_t *st, unsigned int index)
{
    st->is_initialized = is_initialized[index];
    st->is_started = is_started[index];
}


int vtiming_initial_by_index(void __iomem *pci_base_addr, unsigned i)
{

    if (i >= VIDEO_TIMING_CONTROLLER_MAX_NUM) return 0;

    if (is_initialized[i] == 1) return 0;

    if (i == 0)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VIDEO_TIMING_CONTROLLER0_REG);
    else if (i == 1)
        handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VIDEO_TIMING_CONTROLLER1_REG);

    handles[i].ctrl = 0x0000;
    handles[i].active_size = 0x0060;
    handles[i].format = 0x0068;
    handles[i].polarity = 0x006c;
    handles[i].hsize = 0x0070;
    handles[i].vsize = 0x0074;
    handles[i].hsync = 0x0078;
    handles[i].vblank_cycle_index = 0x007c;
    handles[i].vsync_line_index = 0x0080;
    handles[i].vsync_cycle_index = 0x0084;
    mutex_init(&locks[i]);

    if (i == 0)
        cached_registers[i] = &vtiming_cached_registers_0[0];
    else  if (i == 1)
        cached_registers[i] = &vtiming_cached_registers_1[0];

    sw_reset(i);
    set_cached_registers(i, handles[i].ctrl,  fpga_reg_read(handles[i].base , handles[i].ctrl));
    set_cached_registers(i, handles[i].active_size,  fpga_reg_read(handles[i].base , handles[i].active_size));
    set_cached_registers(i, handles[i].format,  fpga_reg_read(handles[i].base , handles[i].format));
    set_cached_registers(i, handles[i].polarity ,  fpga_reg_read(handles[i].base , handles[i].polarity));
    set_cached_registers(i, handles[i].hsize,  fpga_reg_read(handles[i].base , handles[i].hsize));
    set_cached_registers(i, handles[i].vsize,  fpga_reg_read(handles[i].base , handles[i].vsize));
    set_cached_registers(i, handles[i].hsync,  fpga_reg_read(handles[i].base , handles[i].hsync));
    set_cached_registers(i, handles[i].vblank_cycle_index,  fpga_reg_read(handles[i].base , handles[i].vblank_cycle_index));
    set_cached_registers(i, handles[i].vsync_line_index,  fpga_reg_read(handles[i].base , handles[i].vsync_line_index));
    set_cached_registers(i, handles[i].vsync_cycle_index ,  fpga_reg_read(handles[i].base , handles[i].vsync_cycle_index));
    is_initialized[i] = 1;
    //zynq_printk(1, "[zynq_video_timing_controller] Register base address for video timing controller %d : 0x%8lx \n", i,  (unsigned long)handles[i].base);

    return 0;
}

int  vtiming_initial(void __iomem *pci_base_addr)
{

    int i = 0;

    for (i = 0; i < VIDEO_TIMING_CONTROLLER_MAX_NUM; i++) {

        if (is_initialized[i] == 1) continue;

        if (i == 0)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VIDEO_TIMING_CONTROLLER0_REG);
        else if (i == 1)
            handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VIDEO_TIMING_CONTROLLER1_REG);

        handles[i].ctrl = 0x0000;
        handles[i].active_size = 0x0060;
        handles[i].format = 0x0068;
        handles[i].polarity = 0x006c;
        handles[i].hsize = 0x0070;
        handles[i].vsize = 0x0074;
        handles[i].hsync = 0x0078;
        handles[i].vblank_cycle_index = 0x007c;
        handles[i].vsync_line_index = 0x0080;
        handles[i].vsync_cycle_index = 0x0084;
        mutex_init(&locks[i]);

        if (i == 0)
            cached_registers[i] = &vtiming_cached_registers_0[0];
        else  if (i == 1)
            cached_registers[i] = &vtiming_cached_registers_1[0];


        sw_reset(i);
        set_cached_registers(i, handles[i].ctrl,  fpga_reg_read(handles[i].base , handles[i].ctrl));
        set_cached_registers(i, handles[i].active_size,  fpga_reg_read(handles[i].base , handles[i].active_size));
        set_cached_registers(i, handles[i].format,  fpga_reg_read(handles[i].base , handles[i].format));
        set_cached_registers(i, handles[i].polarity ,  fpga_reg_read(handles[i].base , handles[i].polarity));
        set_cached_registers(i, handles[i].hsize,  fpga_reg_read(handles[i].base , handles[i].hsize));
        set_cached_registers(i, handles[i].vsize,  fpga_reg_read(handles[i].base , handles[i].vsize));
        set_cached_registers(i, handles[i].hsync,  fpga_reg_read(handles[i].base , handles[i].hsync));
        set_cached_registers(i, handles[i].vblank_cycle_index,  fpga_reg_read(handles[i].base , handles[i].vblank_cycle_index));
        set_cached_registers(i, handles[i].vsync_line_index,  fpga_reg_read(handles[i].base , handles[i].vsync_line_index));
        set_cached_registers(i, handles[i].vsync_cycle_index ,  fpga_reg_read(handles[i].base , handles[i].vsync_cycle_index));
        is_initialized[i] = 1;
        //zynq_printk(1, "[zynq_video_timing_controller] Register base address for video timing controller %d : 0x%8lx \n", i,  (unsigned long)handles[i].base);
    }
    return 0;
}

int  vtiming_setoption(EVTimingOptionFlags  flag, void *userdata, unsigned int index)
{

    if (is_initialized == 0) return 0;

    switch (flag) {
        case VTIMING_OPTION_RESET:
            sw_reset(index);
            break;
        case VTIMING_OPTION_ENABLE:
            set_enable((vtiming_enable_t *)userdata, index);
            break;
        case VTIMING_OPTION_UPDATE_CHANGE :
            set_update_change((vtiming_enable_t *)userdata, index);
            break;
        case VTIMING_OPTION_SET_FORMAT:
            set_format((vtiming_fromat_t *)userdata, index);
            break;
        case VTIMING_OPTION_SET_SIZE:
            set_size((vtiming_size_t *)userdata, index);
            break;
        default:
            break;
    }
    return 0;
}

int  vtiming_reset(unsigned int i)
{

    if (is_initialized[i] == 0) return 0;

    return sw_reset(i);
}

int  vtiming_start(unsigned int i)
{

    vtiming_enable_t  enable;

    if (is_initialized[i] == 0) return 0;

    if (is_started[i] == 1) return 0;

    enable.value = 0x1;
    set_enable(&enable, i);

    mutex_lock(&locks[i]);
    is_started[i] = 1;
    mutex_unlock(&locks[i]);

    return 0;
}

int  vtiming_stop(unsigned int i)
{

    vtiming_enable_t  enable;

    if (is_initialized[i] == 0) return 0;

    if (is_started[i] == 0) return 0;

    enable.value = 0x0;
    set_enable(&enable, i);

    mutex_lock(&locks[i]);
    is_started[i] = 0;
    mutex_unlock(&locks[i]);

    return 0;
}

int vtiming_release(void __iomem *pci_base_addr)
{

    int  i = 0;

    for (i = 0; i < VIDEO_TIMING_CONTROLLER_MAX_NUM; i++) {
        if (is_initialized[i] == 0) continue;
        mutex_lock(&locks[i]);
        is_initialized[i] = 0;
        mutex_unlock(&locks[i]);
        mutex_destroy(&locks[i]);
    }

    return 0;
}


void  vtiming_dump_registers(unsigned int index)
{

    unsigned int i = 0;
    unsigned int count  =  VTIMING_REG_NUM;

    for (i  = 0; i < count; i ++ ) {
        zynq_printk(1, "[0x%02x] = 0x%08x \n", cached_registers[index][i].offset, cached_registers[index][i].value);
    }
    return;
}

int vtiming_enable_reg_update(unsigned int i)
{
    vtiming_enable_t enable;

    if (is_initialized[i] == 0) return 0;

    enable.value = 0x1;
    set_update_change(&enable, i);

    return 0;
}

int  vtiming_disable_reg_update(unsigned int i)
{
    vtiming_enable_t enable;

    if (is_initialized[i] == 0) return 0;

    enable.value = 0x0;
    set_update_change(&enable, i);

    return 0;
}

int vtiming_config_input_size(unsigned int index, unsigned int in_width, unsigned int in_height)
{

    vtiming_size_t size;

    if (is_initialized[index] == 0) return 0;

    if ((in_width == 1920) && (in_height == 1080))
        size.value = VTIMING_SIZE_1080P60;
    else
        size.value = VTIMING_SIZE_720P60;

    set_size(&size, index);

    return 0;
}

int vtiming_get_regs(unsigned int index, void *regs_ptr)
{

    vpif_video_cfg_regs_t  *regs = (vpif_video_cfg_regs_t *)regs_ptr;

    if (!regs) return -1;

    regs->regs = (vpif_video_cfg_reg_t *)(&cached_registers[index][0]);
    regs->num = VTIMING_REG_NUM;

    return 0;
}

int vtiming_set_reg(unsigned int index, u16 offset, u32 value)
{

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, offset, value);
    set_cached_registers(index, offset, value);
    mutex_unlock(&locks[index]);

    return 0;
}

/////////////////////////////////////////////////////
static int sw_reset(unsigned int i)
{

    u32 value = get_cached_registers(i, handles[i].ctrl) | 0x80000000;

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, handles[i].ctrl, value);
    set_cached_registers(i, handles[i].ctrl, value);
    value = 0x01ffff00;
    fpga_reg_write(handles[i].base, handles[i].ctrl, value);
    set_cached_registers(i, handles[i].ctrl, value);
    mutex_unlock(&locks[i]);
    return  0;
}

static int set_enable(vtiming_enable_t *enable, unsigned int i)
{

    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(i, handles[i].ctrl) | (0x01ffff07);
    else
        value = get_cached_registers(i, handles[i].ctrl) & ~(0x00000007);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, handles[i].ctrl, value);
    set_cached_registers(i, handles[i].ctrl, value);
    mutex_unlock(&locks[i]);

    //if (get_cached_registers(handles[i].polarity)  != (0x0000001f))
    {
        value = get_cached_registers(i, handles[i].polarity) & ~(0x0000001f);
        value = value | (0x0000001f);

        mutex_lock(&locks[i]);
        fpga_reg_write(handles[i].base, handles[i].polarity, value);
        set_cached_registers(i, handles[i].polarity, value);
        mutex_unlock(&locks[i]);
    }

    return 0;
}

static int set_update_change(vtiming_enable_t *enable, unsigned int i)
{

    u32 value= 0x0;

    if (enable->value)
        value = get_cached_registers(i, handles[i].ctrl) | (0x01ffff02);
    else
        value = get_cached_registers(i, handles[i].ctrl) & ~(0x00000002);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, handles[i].ctrl, value);
    set_cached_registers(i, handles[i].ctrl, value);
    mutex_unlock(&locks[i]);

    return 0;
}

static int set_format(vtiming_fromat_t *format, unsigned int i)
{
    u32 value = 0x0;

    value = get_cached_registers(i, handles[i].format) & ~(0x0000000f);

    switch (format->value) {
        case VTIMING_FORMAT_YUV422:
            value = value |  (0x00000000);
            break;
        case VTIMING_FORMAT_YUV444:
            value = value |  (0x00000001);
            break;
        case VTIMING_FORMAT_RGB:
            value = value |  (0x00000002);
            break;
        case VTIMING_FORMAT_YUV420:
            value = value |  (0x00000003);
            break;
        default:
            return -1;
    }

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, handles[i].format, value);
    set_cached_registers(i, handles[i].format, value);
    mutex_unlock(&locks[i]);

    return  0;
}

static int set_size(vtiming_size_t *size, unsigned int i)
{

    vtiming_parametr_t  *p = NULL;

    switch (size->value) {
        case VTIMING_SIZE_1080P60:
            p = &vtiming_parameter_1080p60;
            break;
        case VTIMING_SIZE_720P60:
            p = &vtiming_parameter_720p60;
            break;
        case VTIMING_SIZE_1080P50:
            p = &vtiming_parameter_1080p50;
            break;
        case VTIMING_SIZE_720P50:
            p = &vtiming_parameter_720p50;
            break;
        default:
            return -1;
    }

    set_active_size(p->h_active_size, p->v_active_size, i);
    set_h_frame_size(p->h_frame_size, i);
    set_v_frame_size(p->v_frame_size, i);
    set_h_sync(p->h_sync_start, p->h_sync_end, i);
    set_vsync_line_index(p->v_sync_start, p->v_sync_end, i);
    set_vsync_cycle_index(p->v_sync_cycle_index_start, p->v_sync_cycle_index_end, i);
    set_vblank_cycle_index(p->v_blanking_cycle_index_start, p->v_blanking_cycle_index_end, i);
    return  0;
}

static void set_active_size (unsigned int h_active_size, unsigned int v_active_size, unsigned int i)
{

    u16 reg = handles[i].active_size;
    u32 value = 0x0;

    value = get_cached_registers(i, reg) & ~( 0x1fff0000 | 0x00001fff);
    value = value  | ((v_active_size << 16) | h_active_size);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, reg, value);
    set_cached_registers(i, reg, value);
    mutex_unlock(&locks[i]);
}

static  void set_h_frame_size(unsigned int h_frame_size, unsigned int i)
{

    u16 reg = handles[i].hsize;
    u32 value = 0x0;

    value = get_cached_registers(i, reg) & ~(0x00001fff);
    value = value  | (h_frame_size);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, reg, value);
    set_cached_registers(i, reg, value);
    mutex_unlock(&locks[i]);
}

static void set_v_frame_size(unsigned int v_frame_size, unsigned int i)
{

    u16 reg = handles[i].vsize;
    u32 value = 0x0;

    //value = get_cached_registers(i, reg) & ~(0x00001fff);
    value = value  | (v_frame_size);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, reg, value);
    set_cached_registers(i, reg, value);
    mutex_unlock(&locks[i]);
}

static void set_h_sync(unsigned int h_sync_start, unsigned int h_sync_end, unsigned int i)
{

    u16 reg = handles[i].hsync;
    u32 value = 0x0;

    value = get_cached_registers(i, reg) & ~( 0x1fff0000 | 0x00001fff);
    value = value  | ((h_sync_end << 16) | h_sync_start);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, reg, value);
    set_cached_registers(i, reg, value);
    mutex_unlock(&locks[i]);

}

static void set_vsync_line_index(unsigned int v_sync_start, unsigned int v_sync_end, unsigned int i)
{

    u16 reg = handles[i].vsync_line_index;
    u32 value = 0x0;

    value = get_cached_registers(i, reg) & ~( 0x1fff0000 | 0x00001fff);
    value = value  | ((v_sync_end << 16) | v_sync_start);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, reg, value);
    set_cached_registers(i, reg, value);
    mutex_unlock(&locks[i]);
}


static void set_vsync_cycle_index(unsigned int start_index, unsigned int end_index, unsigned index)
{
    u16 reg = handles[index].vsync_cycle_index;
    u32 value = 0x0;

    value = get_cached_registers(index, reg) & ~( 0x1fff0000 | 0x00001fff);
    value = value  | ((end_index << 16) | start_index);

    mutex_lock(&locks[index]);
    fpga_reg_write(handles[index].base, reg, value);
    set_cached_registers(index, reg, value);
    mutex_unlock(&locks[index]);

}
static void set_vblank_cycle_index(unsigned int start_index, unsigned int end_index, unsigned i)
{

    u16 reg = handles[i].vblank_cycle_index;
    u32 value = 0x0;

    value = get_cached_registers(i, reg) & ~( 0x1fff0000 | 0x00001fff);
    value = value  | ((end_index << 16) | start_index);

    mutex_lock(&locks[i]);
    fpga_reg_write(handles[i].base, reg, value);
    set_cached_registers(i, reg, value);
    mutex_unlock(&locks[i]);
}

/////////////////////////////////////////////////////
vtiming_cfg_reg  vtiming_cached_registers_0 [] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0060, 0x00000000}, //active_size
    {0x0068, 0x00000000}, //format
    {0x006c, 0x00000000}, //polarity
    {0x0070, 0x00000000}, //hsize
    {0x0074, 0x00000000}, //vsize
    {0x0078, 0x00000000}, //hsync
    {0x007c, 0x00000000}, //vblank_cycle_index
    {0x0080, 0x00000000}, //vsync_line_index
    {0x0084, 0x00000000}, //vsync_cycle_index
};

vtiming_cfg_reg  vtiming_cached_registers_1 [] = {
    {0x0000, 0x00000000}, //ctrl
    {0x0060, 0x00000000}, //active_size
    {0x0068, 0x00000000}, //format
    {0x006c, 0x00000000}, //polarity
    {0x0070, 0x00000000}, //hsize
    {0x0074, 0x00000000}, //vsize
    {0x0078, 0x00000000}, //hsync
    {0x007c, 0x00000000}, //vblank_cycle_index
    {0x0080, 0x00000000}, //vsync_line_index
    {0x0084, 0x00000000}, //vsync_cycle_index
};
/////////////////////////////////////////////////////