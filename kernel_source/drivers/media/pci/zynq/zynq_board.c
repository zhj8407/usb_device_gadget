#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/adv761x.h>
#include <media/adv7511.h>

#include "zynq_types.h"
#include "zynq_gpio.h"
#include "zynq_board.h"

#include "modules/zynq_scaler.h"
#include "modules/zynq_osd.h"
#include "modules/zynq_video_selector.h"
#include "modules/zynq_video_timing_controller.h"
#include "modules/zynq_resampler.h"
#include "modules/zynq_vdma.h"

////////////////////////////////////////////////////////////////////////////////////
/*Video capture specific setting*/

static struct i2c_bus_adapter zynq_i2c_adapters[] = {
    [0] = {
        .bus_num = SUBDEV_CH0_I2C_BUS
    },
    [1] = {
        .bus_num = SUBDEV_CH1_I2C_BUS
    },
    [2] = {
        .bus_num = SUBDEV_CH2_I2C_BUS
    },
    [3] = {
        .bus_num = SUBDEV_CH3_I2C_BUS
    },
    [4] = {
        .bus_num = SUBDEV_CH4_I2C_BUS
    },
    [5] = {
        .bus_num = SUBDEV_CH5_I2C_BUS
    }
};


void zynq_setup_i2c_adapter(void)
{
    int i = 0;
    unsigned int  bus_num  = (unsigned int)-1;
    struct i2c_adapter *i2c_adap = NULL;

    for (i = 0;  i < ARRAY_SIZE(zynq_i2c_adapters); i ++) {
        if (zynq_i2c_adapters[i].bus_num != bus_num) {

            zynq_i2c_adapters[i].i2c_adap = i2c_get_adapter(zynq_i2c_adapters[i].bus_num);
            i2c_adap  = 	zynq_i2c_adapters[i].i2c_adap;
            bus_num = zynq_i2c_adapters[i].bus_num;
        } else {
            zynq_i2c_adapters[i].i2c_adap = i2c_adap;
        }
    }
    return;
}

struct i2c_adapter *zynq_get_i2c_adapter_by_bus_num(unsigned int  bus_num)
{
    int i = 0;
    for (i = 0;  i < ARRAY_SIZE(zynq_i2c_adapters); i ++) {
        if (bus_num == zynq_i2c_adapters[i].bus_num)
            return zynq_i2c_adapters[i].i2c_adap;
    }

    return NULL;
}

void  zynq_rls_i2c_adapter(void)
{
    int i = 0;
    unsigned int  bus_num  = (unsigned int)-1;
    for (i = 0;  i < ARRAY_SIZE(zynq_i2c_adapters); i ++) {
        if (zynq_i2c_adapters[i].bus_num != bus_num) {
            if (zynq_i2c_adapters[i].i2c_adap != NULL) i2c_put_adapter(zynq_i2c_adapters[i].i2c_adap);
            bus_num = zynq_i2c_adapters[i].bus_num;
        }
    }
    return;
}

///////////////////////////////////////////////////////////////////////

static struct resource zynq_interrupt_resources[] = {
    [0] = {
        .name = "interrupt0",
        .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
        //.start = gpio_to_irq(GPIO_PU0),
        //.end = gpio_to_irq(GPIO_PU0)
    },
    [1] = {
        .name = "interrupt1",
        .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
        //.start = gpio_to_irq(GPIO_PU1),
        //.end = gpio_to_irq(GPIO_PU1)
    },
    [2] = {
        .name = "interrupt2",
        .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
        //.start = gpio_to_irq(GPIO_PU2),
        //.end = gpio_to_irq(GPIO_PU2)
    },
    [3] = {
        .name = "interrupt3",
        .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
        //.start = gpio_to_irq(GPIO_PU3),
        //.end = gpio_to_irq(GPIO_PU3)
    }
};

void zynq_setup_interrupt(void)
{
#if 1
    zynq_interrupt_resources[0].start =  gpio_to_irq(GPIO_PU0);
    zynq_interrupt_resources[0].end =  gpio_to_irq(GPIO_PU0);

    zynq_interrupt_resources[1].start =  gpio_to_irq(GPIO_PU1);
    zynq_interrupt_resources[1].end =  gpio_to_irq(GPIO_PU1);
#endif

#if 0
    //For adv7611 0x98
    zynq_interrupt_resources[0].start =  gpio_to_irq(GPIO_PU1);
    zynq_interrupt_resources[0].end =  gpio_to_irq(GPIO_PU1);

    //For adv7611 0x9a
    zynq_interrupt_resources[1].start =  gpio_to_irq(GPIO_PU0);
    zynq_interrupt_resources[1].end =  gpio_to_irq(GPIO_PU0);
#endif
    //For MIPI
    zynq_interrupt_resources[2].start =  gpio_to_irq(GPIO_PU2);
    zynq_interrupt_resources[2].end =  gpio_to_irq(GPIO_PU2);

    //For Other video out
    zynq_interrupt_resources[3].start =  gpio_to_irq(GPIO_PU3);
    zynq_interrupt_resources[3].end =  gpio_to_irq(GPIO_PU3);

    return;
}

unsigned  zynq_get_irq(int channel_id)
{
    if (channel_id > 3) return (unsigned) -1;

    return (unsigned) zynq_interrupt_resources[channel_id].start;
}

//The default map address is refered from http://www.analog.com/media/en/technical-documentation/evaluation-documentation/ADV7611_Recommended_Register_Settings.pdf
static struct adv761x_platform_data adv7611_pdata_0 = {

    .i2c_cec = (0x80 >> 1),
    .i2c_inf = (0x7c >> 1) ,
    .i2c_dpll = (0x4c >> 1),
    .i2c_rep = (0x64 >> 1) ,
    .i2c_edid = (0x6c >> 1) ,
    .i2c_hdmi =(0x68 >> 1) ,
    .i2c_cp = (0x44 >> 1),

    .gpio =   GPIO_PK1
};

static struct adv761x_platform_data adv7611_pdata_1 = {

    .i2c_cec = (0x80 >> 1),
    .i2c_inf = (0x7c >> 1),
    .i2c_dpll = (0x4c >> 1),
    .i2c_rep = (0x64 >> 1),
    .i2c_edid = (0x6c >> 1),
    .i2c_hdmi =(0x68 >> 1),
    .i2c_cp = (0x44 >> 1),
    .gpio =  GPIO_PK2
};

//1. The default map address is refered from http://www.analog.com/media/en/technical-documentation/user-guides/ADV7511_Programming_Guide.pdf
//2. The cec clock setting is refered from P. 137 of http://www.analog.com/media/en/technical-documentation/user-guides/ADV7511_Programming_Guide.pdf
//		and http://lists.freedesktop.org/archives/dri-devel/2014-December/074537.html.
static struct adv7511_platform_data adv7511_pdata_0 = {
    .i2c_edid = (0x7e << 1),
    .i2c_cec = (0x78 << 1),
    .cec_clk = 750000,
    .gpio = GPIO_PK3
};

static struct adv7511_platform_data adv7511_pdata_1 = {
    .i2c_edid = (0x7e << 1) >> 1,
    .i2c_cec = (0x78 << 1) >> 1,
    .cec_clk = 750000,
    .gpio = GPIO_PK4
};


struct vpif_subdev_info board_subdev_info[] = {
    {
        .enable = 1,
        .name	= SUBDEV_CH0, //VIN_0 (0x9a)
        .board_info = {
            I2C_BOARD_INFO( ADV7611_I2C_ID_NAME, SUBDEV_CH0_I2C_ADDR ),
            .platform_data = &adv7611_pdata_0,
        },
        .data_pin = VIN_0
    },
    {
        .enable = 1,
        .name	= SUBDEV_CH1, //VIN_1(0x98)
        .board_info = {
            I2C_BOARD_INFO( ADV7611_I2C_ID_NAME, SUBDEV_CH1_I2C_ADDR),
            .platform_data = &adv7611_pdata_1,
        },
        .data_pin = VIN_1
    },
    {
        .enable = 0,
        .name	=SUBDEV_CH2, //VIN_2 (0x3e)
        .board_info = {
            I2C_BOARD_INFO( M10MO_I2C_ID_NAME, SUBDEV_CH2_I2C_ADDR ),
            .platform_data = NULL, //TODO: The  m10mo should need the platform_data ?
        },
        .data_pin = VIN_2
    },
    {
        .enable = 1,
        .name	=SUBDEV_CH3, //VOUT_0(0x7a)
        .board_info = {
            I2C_BOARD_INFO( ADV7511_I2C_ID_NAME, SUBDEV_CH3_I2C_ADDR ),
            .platform_data = &adv7511_pdata_0,
        },
        .data_pin = VOUT_0
    },
    {
        .enable = 1,
        .name	=SUBDEV_CH4,//VOUT_1(0x72)
        .board_info = {
            I2C_BOARD_INFO( ADV7511_I2C_ID_NAME, SUBDEV_CH4_I2C_ADDR ),
            .platform_data = &adv7511_pdata_1,
        },
        .data_pin = VOUT_1
    },
    {
        .enable = 0,
        .name	=SUBDEV_CH5,
        .board_info = {
            I2C_BOARD_INFO( TC358746A_I2C_ID_NAME, SUBDEV_CH5_I2C_ADDR ),
            .platform_data =NULL, //TODO: The  tc358746a should need the platform_data ?
        },
        .data_pin = VPINNONE
    }
};

unsigned int board_subdev_info_num = ARRAY_SIZE(board_subdev_info);

/////////////////////////////////////////////////////////////////////////////////////////////
/*The following definitions are about FPGA.*/
static void rls_imp(struct vpif_vidoe_pipelie_entity* handle, void __iomem *pci_base_addr)
{

    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;
    unsigned int is_vselector = 0;
    unsigned int is_vdma =0;
    unsigned int is_osd = 0;
    unsigned int is_resampler = 0;
    unsigned int is_vtiming = 0;

    if (!handle) return;

    if (!pci_base_addr) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            break;
        case VSELECTOR:
            is_vselector = 1;
            break;
        case VDMA0:
            is_vdma = 1;
            index = 0;
            break;
        case VDMA1:
            is_vdma = 1;
            index = 1;
            break;
        case VDMA2:
            is_vdma = 1;
            index = 2;
            break;
        case VDMA3:
            is_vdma = 1;
            index = 3;
            break;
        case OSD0:
            is_osd = 1;
            index = 0;
            break;
        case OSD1:
            is_osd = 1;
            index = 1;
            break;
        case CRESAMPLER0:
            is_resampler = 1;
            index =0;
            break;
        case CRESAMPLER1:
            is_resampler = 1;
            index =1;
            break;
        case CRESAMPLER2:
            is_resampler = 1;
            index =2;
            break;
        case CRESAMPLER3:
            is_resampler = 1;
            index =3;
            break;
        case CRESAMPLER4:
            is_resampler = 1;
            index = 4;
            break;
        case VTIMING0:
            is_vtiming = 1;
            index = 0;
            break;
        case VTIMING1:
            is_vtiming = 1;
            index = 1;
            break;
        default:
            return;
    }

    if (is_vtiming) {
        vtiming_status_t st;
        vtiming_get_status(&st, index);
        if (st.is_initialized) vtiming_release(pci_base_addr);
    } else if (is_vselector) {
        vselector_status_t st;
        vselector_get_status(&st);
        if (st.is_initialized) vselector_release(pci_base_addr);
    } else if (is_scaler && (index != (unsigned int)-1) ) {
        scaler_status_t st;
        scaler_get_status(&st, index);
        if (st.is_initialized) scaler_release(pci_base_addr);
    }  else if (is_osd && (index != (unsigned int)-1)) {
        osd_status_t st;
        osd_get_status(&st, index);
        if (st.is_initialized) osd_release(pci_base_addr);
    } else if (is_resampler && (index != (unsigned int)-1)) {
        resampler_status_t st;
        resampler_get_status(&st, index);
        if (st.is_initialized) resampler_release(pci_base_addr);
    } else if (is_vdma && (index != (unsigned int)-1)) {
        vdma_status_t st;
        vdma_get_status(&st, index);
        if (st.is_initialized) vdma_release(pci_base_addr);
    }
    return;
}

static void init_imp(struct vpif_vidoe_pipelie_entity* handle, void __iomem *pci_base_addr)
{

    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;
    unsigned int is_vselector = 0;
    unsigned int is_vdma =0;
    unsigned int is_osd = 0;
    unsigned int is_resampler = 0;
    unsigned int is_vtiming = 0;

    if (!handle) return;

    if (!pci_base_addr) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            break;
        case VSELECTOR:
            is_vselector = 1;
            break;
        case VDMA0:
            is_vdma = 1;
            index = 0;
            break;
        case VDMA1:
            is_vdma = 1;
            index = 1;
            break;
        case VDMA2:
            is_vdma = 1;
            index = 2;
            break;
        case VDMA3:
            is_vdma = 1;
            index = 3;
            break;
        case OSD0:
            is_osd = 1;
            index = 0;
            break;
        case OSD1:
            is_osd = 1;
            index = 1;
            break;
        case CRESAMPLER0:
            is_resampler = 1;
            index =0;
            break;
        case CRESAMPLER1:
            is_resampler = 1;
            index =1;
            break;
        case CRESAMPLER2:
            is_resampler = 1;
            index =2;
            break;
        case CRESAMPLER3:
            is_resampler = 1;
            index =3;
            break;
        case CRESAMPLER4:
            is_resampler = 1;
            index = 4;
            break;
        case VTIMING0:
            is_vtiming = 1;
            index = 0;
            break;
        case VTIMING1:
            is_vtiming = 1;
            index = 1;
            break;
        default:
            return;
    }

    if (is_vtiming) {
        vtiming_initial_by_index(pci_base_addr, index);
    } else if (is_vselector) {
        vselector_initial(pci_base_addr);
    } else if (is_scaler && (index != (unsigned int)-1) ) {
        scaler_initial_by_index(pci_base_addr, index);
    }  else if (is_osd && (index != (unsigned int)-1)) {
        osd_initial_by_index(pci_base_addr, index);
    } else if (is_resampler && (index != (unsigned int)-1)) {
        resampler_initial_by_index(pci_base_addr, index);
    } else if (is_vdma && (index != (unsigned int)-1)) {
        vdma_initial_by_index(pci_base_addr, index);
    }
    return;
}


static void dump_scaler_regs(unsigned int index)
{
    scaler_status_t st;
    scaler_get_status(&st, index);
    if (st.is_initialized) scaler_dump_registers(index);
}

static void dump_resampler_regs(unsigned int index)
{
    resampler_status_t st;
    resampler_get_status(&st, index);
    if (st.is_initialized) resampler_dump_registers(index);
}

static void dump_osd_regs(unsigned int index)
{
    osd_status_t st;
    osd_get_status(&st, index);
    if (st.is_initialized) osd_dump_registers(index);
}

static void dump_vdma_rges(unsigned int index)
{
    vdma_status_t st;
    vdma_get_status(&st, index);
    if (st.is_initialized) vdma_dump_registers(index);
    return;
}

static void dump_regs_imp(struct vpif_vidoe_pipelie_entity* handle)
{
    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;
    unsigned int is_vselector = 0;
    unsigned int is_vdma =0;
    unsigned int is_osd = 0;
    unsigned int is_resampler = 0;
    unsigned int is_vtiming = 0;

    if (!handle) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            dump_scaler_regs(index);
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            dump_scaler_regs(index);
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            dump_scaler_regs(index);
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            dump_scaler_regs(index);
            break;
        case VSELECTOR:
            is_vselector = 1;
            break;
        case VDMA0:
            is_vdma = 1;
            index = 0;
            dump_vdma_rges(index);
            break;
        case VDMA1:
            is_vdma = 1;
            index = 1;
            dump_vdma_rges(index);
            break;
        case VDMA2:
            is_vdma = 1;
            index = 2;
            dump_vdma_rges(index);
            break;
        case VDMA3:
            is_vdma = 1;
            index = 3;
            dump_vdma_rges(index);
            break;
        case OSD0:
            is_osd = 1;
            index = 0;
            dump_osd_regs(index);
            break;
        case OSD1:
            is_osd = 1;
            index = 1;
            dump_osd_regs(index);
            break;
        case CRESAMPLER0:
            is_resampler = 1;
            index =0;
            dump_resampler_regs(index);
            break;
        case CRESAMPLER1:
            is_resampler = 1;
            index =1;
            dump_resampler_regs(index);
            break;
        case CRESAMPLER2:
            is_resampler = 1;
            index =2;
            dump_resampler_regs(index);
            break;
        case CRESAMPLER3:
            is_resampler = 1;
            index =3;
            dump_resampler_regs(index);
            break;
        case CRESAMPLER4:
            is_resampler = 1;
            index = 4;
            dump_resampler_regs(index);
            break;
        case VTIMING0:
            is_vtiming = 1;
            index = 0;
            break;
        case VTIMING1:
            is_vtiming = 1;
            index = 1;
            break;
        default:
            return;
    }

    if (is_vtiming) {
        vtiming_status_t st;
        vtiming_get_status(&st, index);
        if (st.is_initialized) vtiming_dump_registers(index);
    } else if (is_vselector) {
        vselector_status_t st;
        vselector_get_status(&st);
        if (st.is_initialized) vselector_dump_registers();
    }

    return;
}


static void stop_scaler(unsigned int index)
{
    scaler_status_t st;
    scaler_get_status(&st, index);
    if (st.is_initialized) scaler_stop(index);
}

static void stop_resampler(unsigned int index)
{
    resampler_status_t st;
    resampler_get_status(&st, index);
    if (st.is_initialized) resampler_stop(index);
}

static void stop_osd(unsigned int index)
{
    osd_status_t st;
    osd_get_status(&st, index);
    if (st.is_initialized) osd_stop(index);
}

static void stop_vdma(unsigned int index)
{
    vdma_status_t st;
    vdma_get_status(&st, index);
    if (st.is_initialized) vdma_stop(index);
}

static void stop_imp(struct vpif_vidoe_pipelie_entity* handle)
{
    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;
    unsigned int is_vselector = 0;
    unsigned int is_vdma =0;
    unsigned int is_osd = 0;
    unsigned int is_resampler = 0;
    unsigned int is_vtiming = 0;

    if (!handle) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            stop_scaler(index);
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            stop_scaler(index);
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            stop_scaler(index);
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            stop_scaler(index);
            break;
        case VSELECTOR:
            is_vselector = 1;
            break;
        case VDMA0:
            is_vdma = 1;
            index = 0;
            stop_vdma(index);
            break;
        case VDMA1:
            is_vdma = 1;
            index = 1;
            stop_vdma(index);
            break;
        case VDMA2:
            is_vdma = 1;
            index = 2;
            stop_vdma(index);
            break;
        case VDMA3:
            is_vdma = 1;
            index = 3;
            stop_vdma(index);
            break;
        case OSD0:
            is_osd = 1;
            index = 0;
            stop_osd(index) ;
            break;
        case OSD1:
            is_osd = 1;
            index = 1;
            stop_osd(index);
            break;
        case CRESAMPLER0:
            is_resampler = 1;
            index =0;
            stop_resampler(index);
            break;
        case CRESAMPLER1:
            is_resampler = 1;
            index =1;
            stop_resampler(index);
            break;
        case CRESAMPLER2:
            is_resampler = 1;
            index =2;
            stop_resampler(index);
            break;
        case CRESAMPLER3:
            is_resampler = 1;
            index =3;
            stop_resampler(index);
            break;
        case CRESAMPLER4:
            is_resampler = 1;
            index = 4;
            stop_resampler(index);
            break;
        case VTIMING0:
            is_vtiming = 1;
            index = 0;
            break;
        case VTIMING1:
            is_vtiming = 1;
            index = 1;
            break;
        default:
            return;
    }

    if (is_vtiming) {
        vtiming_status_t st;
        vtiming_get_status(&st, index);
        if (st.is_initialized) vtiming_stop(index);
    } else if (is_vselector) {
        vselector_status_t st;
        vselector_get_status(&st);
        if (st.is_initialized) vselector_stop();
    }

    return;
}


static void start_scaler(unsigned int index)
{
    scaler_status_t st;
    scaler_get_status(&st, index);
    if (st.is_initialized) scaler_start(index);
}

static void start_resampler(unsigned int index)
{
    resampler_status_t st;
    resampler_get_status(&st, index);
    if (st.is_initialized) resampler_start(index);
}

static void start_osd(unsigned int index)
{
    osd_status_t st;
    osd_get_status(&st, index);
    if (st.is_initialized) osd_start(index);
}

static void start_vdma(unsigned int index)
{
    vdma_status_t st;
    vdma_get_status(&st, index);
    if (st.is_initialized) vdma_start(index);
}

static void start_imp(struct vpif_vidoe_pipelie_entity* handle)
{

    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;
    unsigned int is_vselector = 0;
    unsigned int is_vdma =0;
    unsigned int is_osd = 0;
    unsigned int is_resampler = 0;
    unsigned int is_vtiming = 0;

    if (!handle) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            start_scaler(index);
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            start_scaler(index);
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            start_scaler(index);
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            start_scaler(index);
            break;
        case VSELECTOR:
            is_vselector = 1;
            break;
        case VDMA0:
            is_vdma = 1;
            index = 0;
            start_vdma(index) ;
            break;
        case VDMA1:
            is_vdma = 1;
            index = 1;
            start_vdma(index) ;
            break;
        case VDMA2:
            is_vdma = 1;
            index = 2;
            start_vdma(index) ;
            break;
        case VDMA3:
            is_vdma = 1;
            index = 3;
            start_vdma(index) ;
            break;
        case OSD0:
            is_osd = 1;
            index = 0;
            start_osd(index) ;
            break;
        case OSD1:
            is_osd = 1;
            index = 1;
            start_osd(index);
            break;
        case CRESAMPLER0:
            is_resampler = 1;
            index =0;
            start_resampler(index);
            break;
        case CRESAMPLER1:
            is_resampler = 1;
            index =1;
            start_resampler(index);
            break;
        case CRESAMPLER2:
            is_resampler = 1;
            index =2;
            start_resampler(index);
            break;
        case CRESAMPLER3:
            is_resampler = 1;
            index =3;
            start_resampler(index);
            break;
        case CRESAMPLER4:
            is_resampler = 1;
            index = 4;
            start_resampler(index);
            break;
        case VTIMING0:
            is_vtiming = 1;
            index = 0;
            break;
        case VTIMING1:
            is_vtiming = 1;
            index = 1;
            break;
        default:
            return;
    }

    if (is_vtiming) {
        vtiming_status_t st;
        vtiming_get_status(&st, index);
        if (st.is_initialized) vtiming_start(index);
    } else if (is_vselector) {
        vselector_status_t st;
        vselector_get_status(&st);
        if (st.is_initialized) vselector_start();
    }

    return;
}


static void config_scaler(unsigned int index,  vpif_vidoe_pipelie_entity_config_t* config)
{
    scaler_status_t st;
    scaler_get_status(&st, index);
    if (st.is_initialized) {
        scaler_disable_reg_update(index);
        scaler_setoption( (EScalerOptionFlags)config->flag, config->data, index);
        scaler_enable_reg_update(index);
    }
}

static void config_resampler(unsigned int index,  vpif_vidoe_pipelie_entity_config_t* config)
{
    resampler_status_t st;
    resampler_get_status(&st, index);
    if (st.is_initialized) {
        resampler_disable_reg_update(index);
        resampler_setoption( (EResamplerOptionFlags)config->flag, config->data, index);
        resampler_enable_reg_update(index);
    }
}

static void config_osd(unsigned int index,  vpif_vidoe_pipelie_entity_config_t* config)
{
    osd_status_t st;
    osd_get_status(&st, index);
    if (st.is_initialized) {
        osd_disable_reg_update(index);
        osd_setoption( (EOSDOptionFlags)config->flag, config->data, index);
        osd_enable_reg_update(index);
    }
}

static void config_vdma(unsigned int index, vpif_vidoe_pipelie_entity_config_t* config)
{
    vdma_status_t st;
    vdma_get_status(&st, index);
    if (st.is_initialized) {
        vdma_disable_reg_update(index);
        vdma_setoption( (EVDMAOptionFlags)config->flag, config->data, index);
        vdma_enable_reg_update(index);
    }
    return;
}

static void config_imp(struct vpif_vidoe_pipelie_entity* handle,  vpif_vidoe_pipelie_entity_config_t* config)
{
    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;
    unsigned int is_vselector = 0;
    unsigned int is_vdma =0;
    unsigned int is_osd = 0;
    unsigned int is_resampler = 0;
    unsigned int is_vtiming = 0;

    if (!handle) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            config_scaler(index,   config);
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            config_scaler(index,   config);
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            config_scaler(index,   config);
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            config_scaler(index,   config);
            break;
        case VSELECTOR:
            is_vselector = 1;
            break;
        case VDMA0:
            is_vdma = 1;
            index = 0;
            config_vdma(index, config);
            break;
        case VDMA1:
            is_vdma = 1;
            index = 1;
            config_vdma(index, config);
            break;
        case VDMA2:
            is_vdma = 1;
            index = 2;
            config_vdma(index, config);
            break;
        case VDMA3:
            is_vdma = 1;
            index = 3;
            config_vdma(index, config);
            break;
        case OSD0:
            is_osd = 1;
            index = 0;
            config_osd(index, config);
            break;
        case OSD1:
            is_osd = 1;
            index = 1;
            config_osd(index, config);
            break;
        case CRESAMPLER0:
            is_resampler = 1;
            index =0;
            config_resampler(index, config);
            break;
        case CRESAMPLER1:
            is_resampler = 1;
            index =1;
            config_resampler(index, config);
            break;
        case CRESAMPLER2:
            is_resampler = 1;
            index =2;
            config_resampler(index, config);
            break;
        case CRESAMPLER3:
            is_resampler = 1;
            index =3;
            config_resampler(index, config);
            break;
        case CRESAMPLER4:
            is_resampler = 1;
            index = 4;
            config_resampler(index, config);
            break;
        case VTIMING0:
            is_vtiming = 1;
            index = 0;
            break;
        case VTIMING1:
            is_vtiming = 1;
            index = 1;
            break;
        default:
            return;
    }

    if (is_vtiming) {
        vtiming_status_t st;
        vtiming_get_status(&st, index);
        if (st.is_initialized) {
            vtiming_disable_reg_update(index);
            vtiming_setoption( (EVTimingOptionFlags)config->flag, config->data, index);
            vtiming_enable_reg_update(index);
        }
    } else if (is_vselector) {
        vselector_status_t st;
        vselector_get_status(&st);
        if (st.is_initialized) {
            vselector_disable_reg_update();
            vselector_setoption( (EVSelectorOptionFlags)config->flag, config->data);
            vselector_enable_reg_update();
        }
    }

    return;
}


static void config_scaler_input_size(unsigned int index, unsigned int in_width, unsigned int in_height)
{
    scaler_status_t st;
    scaler_get_status(&st, index);
    if (st.is_initialized) {
        scaler_disable_reg_update(index);
        scaler_config_input_size(index, in_width, in_height);
        scaler_enable_reg_update(index);
    }
}

static void config_osd_input_size(unsigned int index, unsigned int in_width, unsigned int in_height)
{
    osd_status_t st;
    osd_get_status(&st, index);
    if (st.is_initialized) {
        osd_disable_reg_update(index);
        osd_config_input_size(index, in_width, in_height);
        osd_enable_reg_update(index);
    }
}

static void config_resampler_input_size(unsigned int index, unsigned int in_width, unsigned int in_height)
{
    resampler_status_t st;
    resampler_get_status(&st, index);
    if (st.is_initialized) {
        resampler_disable_reg_update(index);
        resampler_config_input_size(index, in_width, in_height);
        resampler_enable_reg_update(index);
    }
}

static void config_vdma_input_size(unsigned int index, unsigned int in_width, unsigned int in_height)
{
    vdma_status_t st;
    vdma_get_status(&st, index);
    if (st.is_initialized) {
        vdma_disable_reg_update(index);
        vdma_config_input_size(index, in_width, in_height);
        vdma_enable_reg_update(index);
    }
    return;
}

static void config_input_size_imp(struct vpif_vidoe_pipelie_entity* handle, unsigned int in_width, unsigned int in_height)
{

    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;
    unsigned int is_vdma =0;
    unsigned int is_osd = 0;
    unsigned int is_resampler = 0;
    unsigned int is_vtiming = 0;

    if (!handle) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            config_scaler_input_size(index,   in_width,  in_height);
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            config_scaler_input_size(index,   in_width,  in_height);
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            config_scaler_input_size(index,   in_width,  in_height);
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            config_scaler_input_size(index,   in_width,  in_height);
            break;
        case VDMA0:
            is_vdma = 1;
            index = 0;
            config_vdma_input_size(index,   in_width,  in_height);
            break;
        case VDMA1:
            is_vdma = 1;
            index = 1;
            config_vdma_input_size(index,   in_width,  in_height);
            break;
        case VDMA2:
            is_vdma = 1;
            index = 2;
            config_vdma_input_size(index,   in_width,  in_height);
            break;
        case VDMA3:
            is_vdma = 1;
            index = 3;
            config_vdma_input_size(index,   in_width,  in_height);
            break;
        case OSD0:
            is_osd = 1;
            index = 0;
            config_osd_input_size(index,   in_width,  in_height);
            break;
        case OSD1:
            is_osd = 1;
            index = 1;
            config_osd_input_size(index,   in_width,  in_height);
            break;
        case CRESAMPLER0:
            is_resampler = 1;
            index =0;
            config_resampler_input_size(index,   in_width,  in_height);
            break;
        case CRESAMPLER1:
            is_resampler = 1;
            index =1;
            config_resampler_input_size(index,   in_width,  in_height);
            break;
        case CRESAMPLER2:
            is_resampler = 1;
            index =2;
            config_resampler_input_size(index,   in_width,  in_height);
            break;
        case CRESAMPLER3:
            is_resampler = 1;
            index =3;
            config_resampler_input_size(index,   in_width,  in_height);
            break;
        case CRESAMPLER4:
            is_resampler = 1;
            index = 4;
            config_resampler_input_size(index,   in_width,  in_height);
            break;
        case VTIMING0:
            is_vtiming = 1;
            index = 0;
            break;
        case VTIMING1:
            is_vtiming = 1;
            index = 1;
            break;
        case VSELECTOR:
            break;
        default:
            return;
    }

    if (is_vtiming) {
        vtiming_status_t st;
        vtiming_get_status(&st, index);
        if (st.is_initialized) {
            vtiming_disable_reg_update(index);
            vtiming_config_input_size(index, in_width, in_height);
            vtiming_enable_reg_update(index);
        }
    }
    return;

}

void config_scaler_crop(unsigned int index,  unsigned int crop_start_x,  unsigned int crop_start_y,  unsigned int crop_width, unsigned int crop_height)
{
    scaler_status_t st;
    scaler_get_status(&st, index);
    if (st.is_initialized) {
        scaler_disable_reg_update(index);
        scaler_config_crop(index,  crop_start_x, crop_start_y, crop_width, crop_height) ;
        scaler_enable_reg_update(index);
    }

}


static void config_crop_imp(struct vpif_vidoe_pipelie_entity* handle, unsigned int crop_start_x,  unsigned int crop_start_y,  unsigned int crop_width, unsigned int crop_height)
{

    unsigned int id = (unsigned int)-1;
    unsigned int index = (unsigned int)-1;
    unsigned int is_scaler = 0;

    if (!handle) return;

    id = handle->id;

    switch (id) {
        case SCALER0:
            is_scaler = 1;
            index =0;
            config_scaler_crop(index,  crop_start_x, crop_start_y, crop_width, crop_height) ;
            break;
        case SCALER1:
            is_scaler = 1;
            index =1;
            config_scaler_crop(index,  crop_start_x, crop_start_y, crop_width, crop_height) ;
            break;
        case SCALER2:
            is_scaler = 1;
            index =2;
            config_scaler_crop(index,  crop_start_x, crop_start_y, crop_width, crop_height) ;
            break;
        case SCALER3:
            is_scaler = 1;
            index =3;
            config_scaler_crop(index,  crop_start_x, crop_start_y, crop_width, crop_height) ;
            break;
        case VSELECTOR:
        case VDMA0:
        case VDMA1:
        case VDMA2:
        case VDMA3:
        case OSD0:
        case OSD1:
        case CRESAMPLER0:
        case CRESAMPLER1:
        case CRESAMPLER2:
        case CRESAMPLER3:
        case CRESAMPLER4:
        case VTIMING0:
        case VTIMING1:
        default:
            return;
    }
    return;
}


vpif_vidoe_pipelie_entity_t  board_video_pipeline_entities[] = {
    {.type= VDMA_TYPE, .id = VDMA0, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VDMA_TYPE, .id = VDMA1, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= SCALER_TYPE, .id = SCALER0, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VDMA_TYPE, .id = VDMA2, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VDMA_TYPE, .id = VDMA3, .config = config_imp,  .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= SCALER_TYPE, .id = SCALER1, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= SCALER_TYPE, .id = SCALER2, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= SCALER_TYPE, .id = SCALER3, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= OSD_TYPE, .id = OSD0, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= OSD_TYPE, .id = OSD1, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= CRESAMPLER_TYPE, .id = CRESAMPLER0, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= CRESAMPLER_TYPE, .id = CRESAMPLER1, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= CRESAMPLER_TYPE, .id = CRESAMPLER2, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= CRESAMPLER_TYPE, .id = CRESAMPLER3, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= CRESAMPLER_TYPE, .id = CRESAMPLER4, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VTIMING_TYPE, .id = VTIMING0, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VTIMING_TYPE, .id = VTIMING1, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VSELECTOR_TYPE, .id = VSELECTOR, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VIN_TYPE, .id = VIN0, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VIN_TYPE, .id = VIN1, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VIN_TYPE, .id = VIN2, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VOUT_TYPE, .id = VOUT0, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type= VOUT_TYPE, .id = VOUT1, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp},
    {.type = PCIEIF_TYPE, .id = PCIEIF, .config = config_imp, .start = start_imp, .stop = stop_imp, .init = init_imp, .rls = rls_imp, .dump_regs = dump_regs_imp, .config_input_size = config_input_size_imp, .config_crop = config_crop_imp}
} ;

unsigned int board_video_pipeline_entity_num = ARRAY_SIZE(board_video_pipeline_entities);

vpif_vidoe_pipelie_entity_t *board_find_video_pipeline_entity(vpif_vidoe_pipelie_entity_id_t id)
{

    unsigned int i  = 0;

    for (i = 0; i < board_video_pipeline_entity_num; i++) {
        if (id == board_video_pipeline_entities[i].id ) return &board_video_pipeline_entities[i];
    }

    return NULL;
}


const char *to_video_pipelin_entity_name(vpif_vidoe_pipelie_entity_id_t  id)
{

    switch (id) {
        case SCALER0:
            return "SCALER0";
        case SCALER1:
            return  "SCALER1";
        case SCALER2:
            return  "SCALER2";
        case SCALER3:
            return  "SCALER3";
        case CRESAMPLER0:
            return  "CRESAMPLER0";
        case CRESAMPLER1:
            return  "CRESAMPLER1";
        case CRESAMPLER2:
            return  "CRESAMPLER2";
        case CRESAMPLER3:
            return  "CRESAMPLER3";
        case CRESAMPLER4:
            return  "CRESAMPLER4";
        case VSELECTOR :
            return  "VSELECTOR";
        case VDMA0:
            return "VDMA0";
        case VDMA1:
            return "VDMA1";
        case VDMA2:
            return "VDMA2";
        case VDMA3:
            return "VDMA3";
        case OSD0:
            return "OSD0";
        case OSD1:
            return "OSD1";
        case VTIMING0:
            return "VTIMING0";
        case VTIMING1:
            return "VTIMING1";
        case VIN0:
            return "VIN0";
        case VIN1:
            return "VIN1";
        case VIN2:
            return "VIN2";
        case VOUT0:
            return "VOUT0";
        case VOUT1:
            return "VOUT1";
        case PCIEIF:
            return "PCIEIF";
        default:
            return "UNKOWNID";
    }
}

const char *to_video_data_pin_name(vpif_video_data_pin_t pin)
{

    switch (pin) {
        case VIN0_IN:
            return "VIN0_IN";
        case VIN0_OUT:
            return "VIN0_OUT";
        case VIN1_IN:
            return "VIN1_IN";
        case VIN1_OUT:
            return "VIN1_OUT";
        case VIN2_IN:
            return "VIN2_IN";
        case VIN2_OUT:
            return "VIN2_OUT";
        case VOUT0_IN:
            return "VOUT0_IN";
        case VOUT0_OUT:
            return "VOUT0_OUT";
        case VOUT1_IN:
            return "VOUT1_IN";
        case VOUT1_OUT:
            return "VOUT1_OUT";
        case OSD0_IN:
            return "OSD0_IN";
        case OSD0_OUT:
            return "OSD0_OUT";
        case OSD1_IN:
            return "OSD1_IN";
        case OSD1_OUT:
            return "OSD1_OUT";
        case SCALER0_IN:
            return "SCALER0_IN";
        case SCALER0_OUT:
            return "SCALER0_OUT";
        case SCALER1_IN:
            return "SCALER1_IN";
        case SCALER1_OUT:
            return "SCALER1_OUT";
        case SCALER2_IN:
            return "SCALER2_IN";
        case SCALER2_OUT:
            return "SCALER2_OUT";
        case SCALER3_IN:
            return "SCALER3_IN";
        case SCALER3_OUT:
            return "SCALER3_OUT";
        case VSELECTOR_IN:
            return "VSELECTOR_IN";
        case VSELECTOR_OUT:
            return "VSELECTOR_OUT";
        case VDMA0_IN:
            return "VDMA0_IN ";
        case VDMA0_OUT:
            return  "VDMA0_OUT";
        case VDMA1_IN:
            return "VDMA1_IN ";
        case VDMA1_OUT:
            return  "VDMA1_OUT";
        case VDMA2_IN:
            return "VDMA2_IN ";
        case VDMA2_OUT:
            return  "VDMA2_OUT";
        case VDMA3_IN:
            return "VDMA3_IN ";
        case VDMA3_OUT:
            return  "VDMA3_OUT";
        case CRESAMPLER0_IN:
            return "CRESAMPLER0_IN";
        case CRESAMPLER0_OUT:
            return  "CRESAMPLER0_OUT";
        case CRESAMPLER1_IN:
            return "CRESAMPLER1_IN";
        case CRESAMPLER1_OUT:
            return  "CRESAMPLER1_OUT";
        case CRESAMPLER2_IN:
            return "CRESAMPLER2_IN";
        case CRESAMPLER2_OUT:
            return  "CRESAMPLER2_OUT";
        case CRESAMPLER3_IN:
            return "CRESAMPLER3_IN";
        case CRESAMPLER3_OUT:
            return  "CRESAMPLER3_OUT";
        case CRESAMPLER4_IN:
            return "CRESAMPLER4_IN";
        case CRESAMPLER4_OUT:
            return  "CRESAMPLER4_OUT";
        case PCIEIF_IN:
            return "PCIEIF_IN";
        case PCIEIF_OUT:
            return "PCIEIF_OUT";
        default:
            return "UNKOWNPN";
    }
}
