#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "zynq_adv762x.h"

#define ADV762X_I2C_BUS 0x2
#define ADV762X_I2C_ADDR (0xB0 >> 1)

#define ZYNQ_HDMI_VERSION "0.0.1"



unsigned int adv762x_i2c_bus =  (unsigned int)(-1);
module_param(adv762x_i2c_bus, int, 0644);

unsigned int adv762x_i2c_addr =  (unsigned int)(-1);
module_param(adv762x_i2c_addr, int, 0644);

unsigned int adv762x_debug  =  0;
module_param(adv762x_debug, int, 0644);
EXPORT_SYMBOL_GPL(adv762x_debug);


unsigned int adv762x_hdmi_src = (unsigned int)(-1);
module_param(adv762x_hdmi_src, int, 0644);

unsigned int adv762x_mode=(unsigned int)(-1);//0:mux mode, 1:transceiver mode, 2:unknown mode
module_param(adv762x_mode, int, 0644);

unsigned int adv762x_use_fixed_edid= 0;
module_param(adv762x_use_fixed_edid, int, 0644);
EXPORT_SYMBOL_GPL(adv762x_use_fixed_edid);



MODULE_DESCRIPTION("Driver for zynq based HDMI  module");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION( ZYNQ_HDMI_VERSION);

static struct i2c_client * g_client = NULL;
static struct i2c_adapter *g_adapter = NULL;

#define ADV7627_I2C_ID_NAME  "adv7627"

struct adv762x_platform_data  adv7627_data = {
    .dpll_map = (0x90 >> 1),//dpll_b_map
    .cp_lite_map =	(0x94 >> 1),//cp_b_map
    .osd_map = (0xf0 >> 1),
    .edid_config_map =  (0x6a >> 1), //edid_config_map
    .rx_repeater_map =(0x66 >> 1),//rx2_repeater_map
    .rx_main_map  = (0x58 >> 1),//hdmi_rx2_map
    .rx_information_map = (0x62 >> 1), //rx2_infoframe_map
    .rx_edid_map = (0x7e >> 1), //edid_mem_map
    .rx_test_map = (0x82  >> 1),
    .tx_main_map = (0xb8 >> 1), //txb_main_map
    .tx_edid_map =(0x84 >> 1),//txb_edid_map
    .tx_test_map = (0xc0>> 1), //txb_test_map
    .tx_packet_map =  (0x70 >> 1),//txb_packet_memory_map
    .tx_cec_map = (0x8c >> 1)//txb_cec_map
};

struct i2c_board_info g_board_info_adv7627 = {
    I2C_BOARD_INFO( ADV7627_I2C_ID_NAME, ADV762X_I2C_ADDR),
    .platform_data =  &adv7627_data
};

static void power_up_sequence(void);
static void reset_sequence() ;
static inline void hdmi_delay(unsigned int msec);

static int __init zynq_hdmi_init(void)
{


    struct i2c_board_info *info = NULL;
    unsigned int bus = (unsigned int)ADV762X_I2C_BUS;
    struct adv762x_platform_data  *pdata = NULL;

    printk(KERN_INFO "[zynq_hdmi]Enter  zynq_hdmi_init()\n");

    info = &g_board_info_adv7627;


    if (adv762x_i2c_addr  != (unsigned int)-1)
        info->addr  = adv762x_i2c_addr;

    if (adv762x_i2c_bus  != (unsigned int)-1)
        bus  = adv762x_i2c_bus;

    pdata = (struct adv762x_platform_data  *)info->platform_data;

    if (adv762x_hdmi_src == 0)
        pdata->hdmi_src_id = TK1HDMI;//RXC
    else if (adv762x_hdmi_src == 1)
        pdata->hdmi_src_id = CON18HDMI ; //RXE
    else
        pdata->hdmi_src_id = UNKNOWNHDMI;//RXB

    if (adv762x_mode == 0)
        pdata->mode = MUXMODE ; //MUX mode
    else if (adv762x_mode == 1)
        pdata->mode = TRANSMODE; //Transceiver Mode
    else
        pdata->mode = UNKNOWNMODE  ; //UNKNOWNMODE

    printk(KERN_INFO "[zynq_hdmi] xxx (src, mode, bus, addr) = (%u, %u, 0x%02x, 0x%02x)\n", pdata->hdmi_src_id,  pdata->mode, bus, info->addr);
#if 0
    power_up_sequence();
    reset_sequence();
    pdata->is_initial_interrupt = 0;
    pdata->adv762x_use_fixed_edid = 1;
    g_adapter = i2c_get_adapter(bus);
    g_client =   i2c_new_device(g_adapter, info);
    if (g_client) i2c_unregister_device(g_client);
    if (g_adapter)i2c_put_adapter(g_adapter);

    hdmi_delay(1000);
#endif
    //power_up_sequence();
    //reset_sequence();
    pdata->is_initial_interrupt = 1;
    pdata->adv762x_use_fixed_edid = adv762x_use_fixed_edid;
    g_adapter = i2c_get_adapter(bus);
    g_client =   i2c_new_device(g_adapter, info);


    printk(KERN_INFO "[zynq_hdmi] I2C client object is = 0x%p\n", g_client);
    printk(KERN_INFO "[zynq_hdmi] [jeff@12345]Leave  zynq_hdmi_init()(adv762x_use_fixed_edid = %u)\n",adv762x_use_fixed_edid);
    return 0;
}

static void __exit zynq_hdmi_fini(void)
{

    printk(KERN_INFO "[zynq_hdmi] Enter zynq_hdmi_fini()\n");

    if (g_client) i2c_unregister_device(g_client);

    if (g_adapter)i2c_put_adapter(g_adapter);

    printk(KERN_INFO "[zynq_hdmi] Leave zynq_hdmi_fini()\n");

    return;
}

static inline void hdmi_delay(unsigned int msec)
{

    //unsigned long timeout = jiffies + HZ * sec;
    //while (time_before(jiffies, timeout)) cpu_relax(  );
    msleep_interruptible(msec);
}

/*
 #/bin/bash

reset_pin=83 #GPIO_PK3


#default low -->high-->low-->delay 5 msec--->high
cd /sys/class/gpio
echo $reset_pin > export
echo out > gpio$reset_pin/direction
echo 1 > gpio$reset_pin/value
echo 0 > gpio$reset_pin/value
sleep 0.05
echo 1 > gpio$reset_pin/value
cd - > /dev/null

cd /sys/class/gpio
#echo 0  > gpio$hpd_pin/value
echo $reset_pin >  unexport
cd - > /dev/n

 */
static void power_up_sequence()
{
    int ret = -1;
    unsigned int reset_gpio_pin = 83; //GPIO_PK3
    ret = gpio_request(reset_gpio_pin, "ADV7627  Reset  pin");
    if (ret) {
        printk(KERN_INFO"[zynq_hdmi] Call gpio_request() for %u  error\n", __func__, reset_gpio_pin);
        return;
    }
    gpio_direction_output(reset_gpio_pin, 1);
    gpio_direction_output(reset_gpio_pin, 0);
    hdmi_delay(50);
    gpio_direction_output(reset_gpio_pin, 1);

    gpio_free(reset_gpio_pin);
    return;
}

/*

 #/bin/bash

reset_pin=83 #GPIO_PK3

#The ADV7625 includes an active low reset pin, RESETB. The ADV7625 can be reset by applying a low reset pulse to the RESETB pin with
#a minimum width of 5 ms. It is recommended to wait 5 ms after the low pulse before an I2C write is performed to the ADV7625.
#RESETB  is always used. Level of pin should be controlled by external control processor.

#default low -->low-->delay 5 msec
cd /sys/class/gpio
echo $reset_pin > export
echo out > gpio$reset_pin/direction
echo 0 > gpio$reset_pin/value
sleep 0.05
echo 1 > gpio$reset_pin/value
cd - > /dev/null

cd /sys/class/gpio
#echo 0  > gpio$hpd_pin/value
echo $reset_pin >  unexport
cd - > /dev/nul

 */

static void reset_sequence()
{
    int ret = -1;
    unsigned int reset_gpio_pin = 83; //GPIO_PK3
    ret = gpio_request(reset_gpio_pin, "ADV7627  Reset  pin");
    if (ret) {
        printk(KERN_INFO"[zynq_hdmi] Call gpio_request() for %u  error\n", __func__, reset_gpio_pin);
        return;
    }
    gpio_direction_output(reset_gpio_pin, 0);
    hdmi_delay(50);
    gpio_direction_output(reset_gpio_pin, 1);

    gpio_free(reset_gpio_pin);

}

module_init(zynq_hdmi_init);
module_exit(zynq_hdmi_fini);