#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <media/adv761x.h>
#include <media/adv7511.h>
#include "zynq_gpio.h"

#define ZYNQ_VERSION "0.0.1"


MODULE_DESCRIPTION("Fake i2c master  driver  module");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(ZYNQ_VERSION);

#define SUBDEV_CH0_I2C_ADDR (0x98 >> 1) //ADV7611
#define SUBDEV_CH1_I2C_ADDR (0x9a >> 1) //ADV7611
#define SUBDEV_CH2_I2C_ADDR (0x3e >> 1) //M10MO
#define SUBDEV_CH3_I2C_ADDR (0x72 >> 1) //ADV7511
#define SUBDEV_CH4_I2C_ADDR (0x7a >> 1) //ADV7511


#define MAX_I2C_DEV_NUM 4

static unsigned int enable[MAX_I2C_DEV_NUM] = {1, 0, 1, 0};


#define ADV7611_I2C_ID_NAME "adv761x"
#define ADV7511_I2C_ID_NAME "adv7511"

#define I2C_DEV_0_ADDR 	(0x98 >> 1)
#define I2C_DEV_0_BUS 	0x1
#define I2C_DEV_0_ID_NAME "adv7611-0"

#define I2C_DEV_1_ADDR		(0x9a >> 1)
#define I2C_DEV_1_BUS	0x1
#define I2C_DEV_1_ID_NAME "adv7611-1"

#define I2C_DEV_2_ADDR	 (0x72 >> 1)
#define I2C_DEV_2_BUS 0x1
#define I2C_DEV_2_ID_NAME "adv7511-0"

#define I2C_DEV_3_ADDR	  (0x7a >> 1)
#define I2C_DEV_3_BUS 0x1
#define I2C_DEV_3_ID_NAME "adv7511-1"


//The default map address is refered from http://www.analog.com/media/en/technical-documentation/evaluation-documentation/ADV7611_Recommended_Register_Settings.pdf
static struct adv761x_platform_data adv7611_pdata_0 = {
    .i2c_cec = (0x80 >> 1),
    .i2c_inf = (0x7c >> 1),
    .i2c_dpll = (0x4c >> 1),
    .i2c_rep = (0x64 >> 1),
    .i2c_edid = (0x6c >> 1),
    .i2c_hdmi =(0x68 >> 1),
    .i2c_cp = (0x44 >> 1),
    .gpio = GPIO_PK2// GPIO_PK1
};

static struct adv761x_platform_data adv7611_pdata_1 = {
    .i2c_cec = (0x80 >> 1) >> 1,
    .i2c_inf = (0x7c >> 1) >> 1,
    .i2c_dpll = (0x4c >> 1) >> 1,
    .i2c_rep = (0x64 >> 1) >> 1,
    .i2c_edid = (0x6c >> 1) >> 1,
    .i2c_hdmi =(0x68 >> 1) >> 1,
    .i2c_cp = (0x44 >> 1) >> 1,
    .gpio =  GPIO_PK1 //GPIO_PK2
};

//1. The default map address is refered from http://www.analog.com/media/en/technical-documentation/user-guides/ADV7511_Programming_Guide.pdf
//2. The cec clock setting is refered from P. 137 of http://www.analog.com/media/en/technical-documentation/user-guides/ADV7511_Programming_Guide.pdf
//		and http://lists.freedesktop.org/archives/dri-devel/2014-December/074537.html.
static struct adv7511_platform_data adv7511_pdata_0 = {
    .i2c_edid = (0x7e << 1),
    .i2c_cec = (0x78 << 1),
    .cec_clk = 750000,
    .gpio =  GPIO_PK4 //GPIO_PK3
};

static struct adv7511_platform_data adv7511_pdata_1 = {
    .i2c_edid = (0x7e << 1) >> 1,
    .i2c_cec = (0x78 << 1) >> 1,
    .cec_clk = 750000,
    .gpio = GPIO_PK3//GPIO_PK4
};

struct i2c_client *i2c_clients[MAX_I2C_DEV_NUM] = {NULL, NULL, NULL, NULL};
struct i2c_adapter *i2c_adapters[MAX_I2C_DEV_NUM] = {NULL, NULL, NULL, NULL};

static struct i2c_board_info board_infos[] = {
    {
        I2C_BOARD_INFO(ADV7611_I2C_ID_NAME,  I2C_DEV_0_ADDR),
        .platform_data = &adv7611_pdata_0
    },

    {
        I2C_BOARD_INFO(ADV7611_I2C_ID_NAME,  I2C_DEV_1_ADDR),
        .platform_data = &adv7611_pdata_1
    },

    {
        I2C_BOARD_INFO(ADV7511_I2C_ID_NAME,  I2C_DEV_2_ADDR),
        .platform_data = &adv7511_pdata_0
    },

    {
        I2C_BOARD_INFO(ADV7511_I2C_ID_NAME,  I2C_DEV_3_ADDR),
        .platform_data = &adv7511_pdata_1
    }
};


static int __init zynq_fake_i2c_master_init(void)
{

    unsigned short  probe_addr =  (unsigned short)-1;

    printk(KERN_INFO "[fake_i2c_master] xxxx driver version %s loaded\n",
           ZYNQ_VERSION);

    if (enable[0]) {
        i2c_adapters[0] =  i2c_get_adapter(I2C_DEV_0_BUS);
        probe_addr = I2C_DEV_0_ADDR;
        if (i2c_adapters[0]) i2c_clients[0] = i2c_new_device(i2c_adapters[0], &board_infos[0]);
    }

    if (enable[1]) {
        i2c_adapters[1] =  i2c_get_adapter(I2C_DEV_1_BUS);
        probe_addr = I2C_DEV_1_ADDR;
        if (i2c_adapters[1]) i2c_clients[1] = i2c_new_device(i2c_adapters[1], &board_infos[1]);
    }

    if (enable[2]) {
        i2c_adapters[2] =  i2c_get_adapter(I2C_DEV_2_BUS);
        probe_addr = I2C_DEV_2_ADDR;
        if (i2c_adapters[2]) i2c_clients[2] = i2c_new_device(i2c_adapters[2], &board_infos[2]);
    }

    if (enable[3]) {
        i2c_adapters[3] =  i2c_get_adapter(I2C_DEV_3_BUS);
        probe_addr = I2C_DEV_3_ADDR;
        if (i2c_adapters[3]) i2c_clients[3] = i2c_new_device(i2c_adapters[3], &board_infos[3]);
    }
    return 0;
}

static void __exit zynq_fake_i2c_master_fini(void)
{
    printk(KERN_INFO "[fake_i2c_master] driver version %s unloaded\n",
           ZYNQ_VERSION);

    if (enable[0]) {
        if (i2c_clients[0])i2c_unregister_device(i2c_clients[0]);
        if (i2c_adapters[0]) i2c_put_adapter(i2c_adapters[0]);
    }

    if (enable[1]) {
        if (i2c_clients[1])i2c_unregister_device(i2c_clients[1]);
        if (i2c_adapters[1]) i2c_put_adapter(i2c_adapters[1]);
    }

    if (enable[2]) {
        if (i2c_clients[2])i2c_unregister_device(i2c_clients[2]);
        if (i2c_adapters[2]) i2c_put_adapter(i2c_adapters[2]);
    }

    if (enable[3]) {
        if (i2c_clients[3])i2c_unregister_device(i2c_clients[3]);
        if (i2c_adapters[3]) i2c_put_adapter(i2c_adapters[3]);
    }

    return;
}

module_init(zynq_fake_i2c_master_init);
module_exit(zynq_fake_i2c_master_fini);