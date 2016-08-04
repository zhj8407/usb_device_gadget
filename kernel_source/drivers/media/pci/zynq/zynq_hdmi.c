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
#include "zynq_adv762x.h"

#define ADV762X_I2C_BUS 0x2
#define ADV762X_I2C_ADDR (0xB0 >> 1)

#define ZYNQ_HDMI_VERSION "0.0.1"

//insmod  ./zynqhdmi.ko  adv762x_id=1 adv762x_i2c_bus=0x2 adv762x_i2c_addr=0x4c 

unsigned int adv762x_id = 0; //0: adv7625, 1:adv7627
module_param(adv762x_id, int, 0644);

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


MODULE_DESCRIPTION("Driver for zynq based HDMI  module");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION( ZYNQ_HDMI_VERSION);

static struct i2c_client * g_client = NULL;
static struct i2c_adapter *g_adapter = NULL;

#define ADV7625_I2C_ID_NAME  "adv7625"
#define ADV7627_I2C_ID_NAME  "adv7627"
/*
B0 EC 68 ; default=0x00, Set HDMI_Rx1_Map address
B0 F0 58 ; default=0x00, Set HDMI_Rx2_Map address
B0 EE 64 ; default=0x00, Set EDID_Config_and_Rx1_Repeater_Map address
B0 F2 F4 ; default=0x00, Set Rx2_Repeater_Map_Address
B0 EF 62 ; default=0x00, Set Rx1_Infoframe_Map address
B0 F3 F6 ; default=0x00, Set Rx2_Infoframe_Map address
B0 E8 44 ; default=0x00, Set CP_Lite_A_Map address
B0 E9 94 ; default=0x00, Set CP_Lite_B_Map address
B0 E6 4E ; default=0x00, Set DPLL_A_Map address 
B0 E7 90 ; default=0x00, Set DPLL_B_Map address 
B0 EA F0 ; default=0x00, Set OSD_Map address
B0 F4 B8 ; default=0x00, Set TxA_Main_Map address
B0 F9 F8 ; default=0x00, Set TxB_Main_Map address
B0 F7 70 ; default=0x00, Set TxA_Packet_Memory_Map address
B0 FC FA ; default=0x00, Set TxB_Packet_Memory_Map address
B0 F8 80 ; default=0x00, Set TxA_CEC_Map address
B0 FD 8C ; default=0x00, Set TxB_CEC_Map address
B0 F5 7E ; default=0x00, Set TxA_Edid_Map address
B0 FA 84 ; default=0x00, Set TxB_Edid_Map address
B0 F6 7C ; default=0x00, Set TxA_Test_Map address
B0 FB C0 ; default=0x00, Set TxB_Test_Map address
 */
struct adv762x_platform_data  adv7625_data = {
	.id = ADV7625,
	.rx_main_map[0]  = (0x68 >> 1), 
	.rx_main_map[1]  = (0x58 >> 1),
	.rx_repeater_map[0] =  (0x64>> 1), 
	.rx_repeater_map[1] =(0xf4 >> 1),
	.rx_information_map[0] =  (0x62 >> 1),
	.rx_information_map[1] = (0xf6 >> 1),
	.rx_edid_map[0] = (0x6c >> 1),
	.rx_edid_map[1] = (0xd6>> 1),
	.rx_test_map = (0x82  >> 1),
	.cp_lite_map[0] = (0x44 >> 1), 
	.cp_lite_map[1] =	(0x94 >> 1),
	.dpll_map[0] = (0x4e >> 1), 
	.dpll_map[1] = (0x90 >> 1),
	.osd_map = (0xf0 >> 1),
	.tx_main_map[0] = (0xb8 >> 1),
	.tx_main_map[1] = (0xf8 >> 1),
	.tx_packet_map[0] =  (0x70 >> 1), 
	.tx_packet_map[1] =  (0xfa >> 1),
	.tx_cec_map[0] =  (0x80 >> 1),
	.tx_cec_map[1] = (0x8c >> 1),
	.tx_edid_map[0] = (0x7e >> 1), 
	.tx_edid_map[1] =(0x84 >> 1),
	.tx_test_map[0] = (0x7c >> 1), 
	.tx_test_map[1] = (0xc0>> 1)
};


struct adv762x_platform_data  adv7627_data = {
	.id = ADV7627,
	.rx_main_map[0]  = (0x68 >> 1), 
	.rx_main_map[1]  = (0x58 >> 1),
	.rx_repeater_map[0] =  (0x64>> 1), 
	.rx_repeater_map[1] =(0xf4 >> 1),
	.rx_information_map[0] =  (0x62 >> 1),
	.rx_information_map[1] = (0xf6 >> 1),
	.rx_edid_map[0] = (0x6c >> 1),
	.rx_edid_map[1] = (0xd6 >> 1),
	.rx_test_map = (0x82  >> 1),
	.cp_lite_map[0] = (0x44 >> 1), 
	.cp_lite_map[1] =	(0x94 >> 1),
	.dpll_map[0] = (0x4e >> 1), 
	.dpll_map[1] = (0x90 >> 1),
	.osd_map = (0xf0 >> 1),
	.tx_main_map[0] = (0xb8 >> 1),
	.tx_main_map[1] = (0xf8 >> 1),
	.tx_packet_map[0] =  (0x70 >> 1), 
	.tx_packet_map[1] =  (0xfa >> 1),
	.tx_cec_map[0] =  (0x80 >> 1),
	.tx_cec_map[1] = (0x8c >> 1),
	.tx_edid_map[0] = (0x7e >> 1), 
	.tx_edid_map[1] =(0x84 >> 1),
	.tx_test_map[0] = (0x7c >> 1), 
	.tx_test_map[1] = (0xc0>> 1)
};

struct adv762x_platform_data  adv762x_data = {
	.id = ADV762X,
	.rx_main_map[0]  = (0x68 >> 1), 
	.rx_main_map[1]  = (0x58 >> 1),
	.rx_repeater_map[0] =  (0x64>> 1), 
	.rx_repeater_map[1] =(0xf4 >> 1),
	.rx_information_map[0] =  (0x62 >> 1),
	.rx_information_map[1] = (0xf6 >> 1),
	.rx_edid_map[0] = (0x6c >> 1),
	.rx_edid_map[1] = (0xd6 >> 1),
	.rx_test_map = (0x82  >> 1),
	.cp_lite_map[0] = (0x44 >> 1), 
	.cp_lite_map[1] =	(0x94 >> 1),
	.dpll_map[0] = (0x4e >> 1), 
	.dpll_map[1] = (0x90 >> 1),
	.osd_map = (0xf0 >> 1),
	.tx_main_map[0] = (0xb8 >> 1),
	.tx_main_map[1] = (0xf8 >> 1),
	.tx_packet_map[0] =  (0x70 >> 1), 
	.tx_packet_map[1] =  (0xfa >> 1),
	.tx_cec_map[0] =  (0x80 >> 1),
	.tx_cec_map[1] = (0x8c >> 1),
	.tx_edid_map[0] = (0x7e >> 1), 
	.tx_edid_map[1] =(0x84 >> 1),
	.tx_test_map[0] = (0x7c >> 1), 
	.tx_test_map[1] = (0xc0>> 1)
};

struct i2c_board_info g_board_info_adv7625 = {
	       I2C_BOARD_INFO( ADV7625_I2C_ID_NAME, ADV762X_I2C_ADDR),
            .platform_data =  &adv7625_data
};

struct i2c_board_info g_board_info_adv7627 = {
	       I2C_BOARD_INFO( ADV7627_I2C_ID_NAME, ADV762X_I2C_ADDR),
            .platform_data =  &adv7627_data
};

struct i2c_board_info g_board_info_adv762x = {
	       I2C_BOARD_INFO( ADV7627_I2C_ID_NAME, ADV762X_I2C_ADDR),
            .platform_data =  &adv762x_data
};

#if 0
//Refence from : http://lxr.free-electrons.com/source/drivers/i2c/i2c-core.c#L2195
static int i2c_clients_probe(struct i2c_adapter * adap, unsigned short addr) {
	int err;
    union i2c_smbus_data dummy;
    err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &dummy);
    return err >= 0;
}
#endif
static int __init zynq_hdmi_init(void) {

	struct i2c_board_info *info = NULL;
	unsigned int bus = (unsigned int)ADV762X_I2C_BUS;
	struct adv762x_platform_data  *pdata = NULL;
	
	printk(KERN_INFO "[zynq_hdmi]xxx  Enter  zynq_hdmi_init()\n");
	
	if (adv762x_id == 0)
		info = &g_board_info_adv7625;
	else if (adv762x_id == 1)
		info = &g_board_info_adv7627;
	else 
		info = &g_board_info_adv762x;
	
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
		
	printk(KERN_INFO "[zynq_hdmi] (id, src, mode, bus, addr) = (%u, %u, %u, 0x%02x, 0x%02x)\n", pdata->id, pdata->hdmi_src_id,  pdata->mode, bus, info->addr);
	
	g_adapter = i2c_get_adapter(bus);
	g_client =   i2c_new_device(g_adapter, info);
	printk(KERN_INFO "[zynq_hdmi] I2C client object is = 0x%p\n", g_client);
	printk(KERN_INFO "[zynq_hdmi] xxx Leave  zynq_hdmi_init()\n");
 	return 0;
}

static void __exit zynq_hdmi_fini(void) {
	
	printk(KERN_INFO "[zynq_hdmi] Enter zynq_hdmi_fini()\n");
	
	if (g_client) i2c_unregister_device(g_client);
	
	if (g_adapter)i2c_put_adapter(g_adapter);
   
	printk(KERN_INFO "[zynq_hdmi] Leave zynq_hdmi_fini()\n");

    return;
}

module_init(zynq_hdmi_init);
module_exit(zynq_hdmi_fini);