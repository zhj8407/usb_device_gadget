#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/delay.h>

#include "zynq_gpio.h"
#include "zynq_adv762x.h"

extern unsigned int adv762x_debug;

#define ADV762x_RST_PIN GPIO_PK3
#define ADV762x_INTX_PIN GPIO_PK4
/*
The ADV7625 features a 768-byte internal EDID memory which can be configured in a number of different manners; it can be
configured as dual 256-byte EDIDs which can be distributed to two of the input ports or it can be configured for one extended EDID
(512-byte) and one 256-byte EDID
*/

#define EDID_MEM_LEN 256

#define ADV762X_DRIVER_NAME "adv762x"

#define EDID_BIN_NAME		"/lib/firmware/EDID/edid.bin"

struct adv762x_edid {
	unsigned char  edid_buf[EDID_MEM_LEN];
};

struct adv762x_ctx {
	ADV762X_ID id;
	ADV762x_HDMI_SRC_ID hdmi_src_id;
	ADV762x_MODE mode;
	struct adv762x_edid edid;
	
	unsigned int rx_main_map_addr[2];
	unsigned int rx_repeater_map_addr[2];
	unsigned int rx_information_map_addr[2];
	unsigned int rx_edid_map_addr[2];
	unsigned int rx_test_map_addr;
	unsigned int cp_lite_map_addr[2];
	unsigned int dpll_map_addr[2];
	unsigned int osd_map_addr;
	unsigned int tx_main_map_addr[2];
	unsigned int tx_packet_map_addr[2];
	unsigned int tx_cec_map_addr[2];
	unsigned int tx_edid_map_addr[2];
	unsigned int tx_test_map_addr[2];
	
	struct i2c_client *io_client;
	struct i2c_client *rx_main_map[2];
	struct i2c_client *rx_repeater_map[2];
	struct i2c_client *rx_information_map[2];
	struct i2c_client *rx_edid_map[2];
	struct i2c_client *rx_test_map;
	struct i2c_client *cp_lite_map[2];
	struct i2c_client *dpll_map[2];
	struct i2c_client *osd_map;
	struct i2c_client *tx_main_map[2];
	struct i2c_client *tx_packet_map[2];
	struct i2c_client *tx_cec_map[2];
	struct i2c_client *tx_edid_map[2];
	struct i2c_client *tx_test_map[2];
} ;
struct adv762x_ctx  g_ctx;

extern unsigned char  HdmiRecomInitTable[];

void adv762x_edid_debug_add(struct adv762x_ctx *ctx);
void adv762x_edid_debug_remove(struct adv762x_ctx *ctx);

static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command);
static s32 adv_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value);
static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client, u8 command, u8 length, const u8 *values);
static struct i2c_client *adv_dummy_client(struct i2c_client *client,
					       u8 addr, u8 def_addr, u8 io_reg);
static void  adv_smbus_write_byte_field8 (struct i2c_client *client, 
										  unsigned char RegAddr, unsigned int Mask, 
                         					unsigned int BitPos, unsigned char  FieldVal);

//static int adv762x_hw_reset(struct adv762x_ctx  *ctx);
static int adv762x_init(struct adv762x_ctx  *ctx);
static int adv762x_rls(struct adv762x_ctx  *ctx);
static inline void adv762x_delay(unsigned int msec);

static int transceiver_mode_init(struct adv762x_ctx  *ctx);
static int mux_mode_init(struct adv762x_ctx  *ctx);

static int transceiver_mode_rls(struct adv762x_ctx  *ctx);
static int mux_mode_rls(struct adv762x_ctx  *ctx);

static int hdmi_audio_output_fmt (struct adv762x_ctx  *ctx, unsigned int mode, unsigned int bitwidth);
static int hdmi_query_cable_detect_status(struct adv762x_ctx  *ctx, u8 *value);
static int hdmi_hot_plug_assert (struct adv762x_ctx  *ctx, ADV762X_HPA ctrl);
static int hdmi_audio_output_mclk (struct adv762x_ctx  *ctx, ADV762X_MCLK_DIV mclk_div);
static int hdmi_audio_output_i2s_map(struct adv762x_ctx  *ctx, unsigned int map_mode, unsigned int invert);

typedef enum {
	RX_EDIDIM_512_A = 0,
	RX_EDIDIM_256_B = 1
}RX_EDID_IMAGE;

static int adv762x_edid(struct adv762x_ctx * ctx, unsigned char *edid_buf, unsigned int edid_len, RX_EDID_IMAGE edidIm);
static int adv762x_read_edid_form_bin_file(const char *bin_file_name, unsigned char *edid_buf);


static int adv762x_probe(struct i2c_client *client,  const struct i2c_device_id *id) {

    struct adv762x_platform_data* pdata = NULL;
    struct i2c_adapter *adapter = NULL;
	
	printk(KERN_INFO "[zynq_adv762x] Enter  adv762x_probe()\n");
	
	if (client == NULL) {
		printk(KERN_INFO"[zynq_adv762x]I2C client object is NULL!!\n");
        return -ENODEV;
	}
	
	printk(KERN_INFO"[zynq_adv762x]I2C client object is 0x%p \n", client);
	
	adapter = to_i2c_adapter(client->dev.parent);
	if (adapter == NULL) {
        printk(KERN_INFO"[zynq_adv762x]I2C adpter object is NULL!!\n");
        return -ENODEV;
    }
	
	 if (!client->dev.platform_data) {
        printk(KERN_INFO"[zynq_adv762x]No platform data found\n");
        return -ENODEV;
    }
	
	pdata =client->dev.platform_data;
	
	printk(KERN_INFO "[zynq_adv762x]Chip found @ 0x%02x (bus:%u)(id:%d)(ptr:%p)\n", client->addr, adapter->nr, pdata->id, client);

	//The following is for verify the i2c read function (adv_smbus_read_byte_data())
	if((client->addr == 0x4c && adapter->nr ==0x2) || (client->addr == 0x4c && adapter->nr ==0x1) ) {
		printk(KERN_INFO"[zynq_adv762x]Chip ID High Byte Chip ID High Byte (should be 0x20):%02x\n", 	adv_smbus_read_byte_data(client, 0xea));
		printk(KERN_INFO"[zynq_adv762x]Chip ID Low Byte Chip ID Low Byte (shoudl be 0x51):%02x\n", adv_smbus_read_byte_data(client,0xeb));
	}
	
	memset(&g_ctx, 0x0, sizeof(struct adv762x_ctx));
	
	g_ctx.id = pdata->id;
	g_ctx.hdmi_src_id = pdata->hdmi_src_id;
	g_ctx.mode =pdata->mode;
	g_ctx.io_client = client;
	g_ctx.rx_main_map_addr[0] = pdata->rx_main_map[0];
	g_ctx.rx_main_map_addr[1] = pdata->rx_main_map[1];
	g_ctx.rx_repeater_map_addr[0] = pdata->rx_repeater_map[0];
	g_ctx.rx_repeater_map_addr[1] = pdata->rx_repeater_map[1];
	g_ctx.rx_information_map_addr[0] = pdata->rx_information_map[0] ;
	g_ctx.rx_information_map_addr[1] = pdata->rx_information_map[1] ;
	g_ctx.rx_edid_map_addr[0] = pdata->rx_edid_map[0];
	g_ctx.rx_edid_map_addr[1] = pdata->rx_edid_map[1];
	g_ctx.rx_test_map_addr = pdata->rx_test_map;
	g_ctx.cp_lite_map_addr[0] = pdata->cp_lite_map[0];
	g_ctx.cp_lite_map_addr[1] = pdata->cp_lite_map[1];
	g_ctx.dpll_map_addr[0] = pdata->dpll_map[0];
	g_ctx.dpll_map_addr[1] = pdata->dpll_map[1];
	g_ctx.osd_map_addr = pdata->osd_map;
	g_ctx.tx_main_map_addr[0] = pdata->tx_main_map[0];
	g_ctx.tx_main_map_addr[1] = pdata->tx_main_map[1];
	g_ctx.tx_packet_map_addr[0] = pdata->tx_packet_map[0];
	g_ctx.tx_packet_map_addr[1] = pdata->tx_packet_map[1];
	g_ctx.tx_cec_map_addr[0] =  pdata->tx_cec_map[0];
	g_ctx.tx_cec_map_addr[1] =  pdata->tx_cec_map[1];
	g_ctx.tx_edid_map_addr[0] = pdata->tx_edid_map[0];
	g_ctx.tx_edid_map_addr[1] = pdata->tx_edid_map[1];
	g_ctx.tx_test_map_addr[0] = pdata->tx_test_map[0];
	g_ctx.tx_test_map_addr[1] = pdata->tx_test_map[1];

	if (adv762x_read_edid_form_bin_file(EDID_BIN_NAME,  &(g_ctx.edid.edid_buf[0])) != 0) goto exit;
	printk(KERN_INFO "[zynq_adv762xi] ADV762X id is  %u\n", g_ctx.id);
	printk(KERN_INFO "[zynq_adv762xi] ADV762X hdmi_src_id  is  %u\n", g_ctx.hdmi_src_id);
	printk(KERN_INFO "[zynq_adv762xi] ADV762X mode  is  %u\n", g_ctx.mode);
	if ((g_ctx.id == ADV7625) || (g_ctx.id == ADV7627)) {
			if (adv762x_init(&g_ctx) != 0) goto exit;
	}
	
	adv762x_edid_debug_add(&g_ctx);
exit:	
	printk(KERN_INFO "[zynq_adv762xi] Leave  adv762x_probe()\n");
	return  0;
}

static int adv762x_remove(struct i2c_client *client) {
	printk(KERN_INFO "[zynq_adv762x] Enter  adv762x_remove()\n");
	adv762x_edid_debug_remove(&g_ctx);
	if ((g_ctx.id == ADV7625) || (g_ctx.id == ADV7627)) {
		adv762x_rls(&g_ctx);
	}
	printk(KERN_INFO "[zynq_adv762x] Leave  adv762x_remove()\n");
    return 0;
}

static const struct i2c_device_id adv762x_id[] = {
    { "adv7627", 0 },
	{ "adv7625", 0 },
    { },
};

MODULE_DEVICE_TABLE(i2c, adv762x_id);

static struct i2c_driver adv762x_driver = {
    .driver = {
        .owner	= THIS_MODULE,
        .name	= ADV762X_DRIVER_NAME,
    },
    .probe		= adv762x_probe,
    .remove		= adv762x_remove,
    .id_table	= adv762x_id,
};

module_i2c_driver(adv762x_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ADV7627 driver");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");

//////////////////////////////////////////////////////////////////////////////////////////

static int adv762x_init(struct adv762x_ctx  *ctx){
	
	if (!ctx) goto exit;
	
	if (adv_smbus_write_byte_data(ctx->io_client, 0xff, 0xff) != 0) goto exit; // Reset
	adv762x_delay(5) ;//5ms delay after Reset 
	if(adv_smbus_write_byte_data(ctx->io_client, 0xe5, 0x82) != 0) goto exit; //ADI Required Write 
	
	//d_info[15:0], IO Map, Address 0xDF[7:0]; Address 0xE0[7:0] (Read Only)
	//rd_info[15:0]: 0x4080, 0x4081
	printk(KERN_INFO"[zynq_adv762x]Chip ID High Byte Chip ID High Byte (should be 0x40):%02x\n", 	adv_smbus_read_byte_data(ctx->io_client, 0xe0));
	printk(KERN_INFO"[zynq_adv762x]Chip ID Low Byte Chip ID Low Byte (shoudl be 0x81):%02x\n", adv_smbus_read_byte_data(ctx->io_client,0xdf));
	
	ctx->rx_main_map[0] = adv_dummy_client(ctx->io_client, ctx->rx_main_map_addr[0], (0x68 >> 1), 0xec);
	ctx->rx_main_map[1] = adv_dummy_client(ctx->io_client, ctx->rx_main_map_addr[1], (0x58 >> 1), 0xf0);//adv7627
	if ((ctx->rx_main_map[0] == NULL) || (ctx->rx_main_map[1] == NULL)) goto exit;
	
	ctx->rx_repeater_map[0] = adv_dummy_client(ctx->io_client, ctx->rx_repeater_map_addr[0], (0x64 >> 1), 0xee);//adv7627 (edid config)
	ctx->rx_repeater_map[1] = adv_dummy_client(ctx->io_client, ctx->rx_repeater_map_addr[1], (0xf4 >> 1), 0xf2);//adv7627
	if ((ctx->rx_repeater_map[0] == NULL) || (ctx->rx_repeater_map[1] == NULL)) goto exit;
	
	ctx->rx_information_map[0] = adv_dummy_client(ctx->io_client, ctx->rx_information_map_addr[0], (0x62 >> 1), 0xef);
	ctx->rx_information_map[1] = adv_dummy_client(ctx->io_client, ctx->rx_information_map_addr[1], (0xf6 >> 1), 0xf3);//adv7627
	if ((ctx->rx_information_map[0] == NULL) || (ctx->rx_information_map[1] == NULL)) goto exit;
	
	ctx->rx_edid_map[0] = adv_dummy_client(ctx->io_client, ctx->rx_edid_map_addr[0], (0x6c >> 1), 0xed);//adv7627
	ctx->rx_edid_map[1] = adv_dummy_client(ctx->io_client, ctx->rx_edid_map_addr[1], (0xd6 >> 1), 0xf1);
	if ((ctx->rx_edid_map[0] == NULL) || (ctx->rx_edid_map[1] == NULL)) goto exit;
	
	ctx->rx_test_map = adv_dummy_client(ctx->io_client, ctx->rx_edid_map_addr[1], (0x82 >> 1),  0xe5);
	if ((ctx->rx_test_map == NULL) || (ctx->rx_test_map == NULL)) goto exit;
	
	ctx->cp_lite_map[0] = adv_dummy_client(ctx->io_client, ctx->cp_lite_map_addr[0], (0x44 >> 1), 0xe8);
	ctx->cp_lite_map[1] = adv_dummy_client(ctx->io_client, ctx->cp_lite_map_addr[1], (0x94 >> 1), 0xe9);//adv7627
	if ((ctx->cp_lite_map[0] == NULL) || (ctx->cp_lite_map[1] == NULL)) goto exit;
	
	ctx->dpll_map[0] = adv_dummy_client(ctx->io_client, ctx->dpll_map_addr[0], (0x4e >> 1), 0xe6);
	ctx->dpll_map[1] = adv_dummy_client(ctx->io_client, ctx->dpll_map_addr[1], (0x90 >> 1), 0xe7);//adv7627
	if ((ctx->dpll_map[0] == NULL) || (ctx->dpll_map[1] == NULL)) goto exit;
	
	ctx->osd_map = adv_dummy_client(ctx->io_client, ctx->osd_map_addr, (0xf0 >> 1), 0xea);
	if (ctx->osd_map == NULL) goto exit;
	
	ctx->tx_main_map[0] = adv_dummy_client(ctx->io_client, ctx->tx_main_map_addr[0], (0xb8 >> 1), 0xf4);//adv7627
	ctx->tx_main_map[1] = adv_dummy_client(ctx->io_client, ctx->tx_main_map_addr[1], (0xf8 >> 1), 0xf9);//adv7627
	if ((ctx->tx_main_map[0] == NULL) || (ctx->tx_main_map[1] == NULL)) goto exit;
	
	ctx->tx_packet_map[0] = adv_dummy_client(ctx->io_client, ctx->tx_packet_map_addr[0], (0x70 >> 1), 0xf7);
	ctx->tx_packet_map[1] = adv_dummy_client(ctx->io_client, ctx->tx_packet_map_addr[1], (0xfa >> 1), 0xfc);//adv7627 (UDP)
	if ((ctx->tx_packet_map[0] == NULL) || (ctx->tx_packet_map[1] == NULL)) goto exit;
	
	ctx->tx_cec_map[0] = adv_dummy_client(ctx->io_client, ctx->tx_cec_map_addr[0], (0x80 >> 1), 0xf8);//adv7627
	ctx->tx_cec_map[1] = adv_dummy_client(ctx->io_client, ctx->tx_cec_map_addr[1], (0x8c >> 1), 0xfd);
	if ((ctx->tx_cec_map[0] == NULL) || (ctx->tx_cec_map[1] == NULL)) goto exit;
	
	ctx->tx_edid_map[0] = adv_dummy_client(ctx->io_client, ctx->tx_edid_map_addr[0], (0x7e >> 1), 0xf5);
	ctx->tx_edid_map[1] = adv_dummy_client(ctx->io_client, ctx->tx_edid_map_addr[1], (0x84 >> 1), 0xfa);//adv7627
	if ((ctx->tx_edid_map[0] == NULL) || (ctx->tx_edid_map[1] == NULL)) goto exit;
	
	ctx->tx_test_map[0] = adv_dummy_client(ctx->io_client, ctx->tx_test_map_addr[0], (0x7c >> 1), 0xf6);
	ctx->tx_test_map[1] = adv_dummy_client(ctx->io_client, ctx->tx_test_map_addr[1], (0xc0 >> 1), 0xfb);//adv7627
	if ((ctx->tx_test_map[0] == NULL) || (ctx->tx_test_map[1] == NULL)) goto exit;
	
	switch (ctx->mode) {
		case MUXMODE:
			printk(KERN_INFO"[zynq_adv762x]Working under the mux mode.\n");
			if (mux_mode_init(ctx) != 0) goto exit;
			break;
		case TRANSMODE:
			printk(KERN_INFO"[zynq_adv762x]Working under the transceiver mode.\n");
			if (transceiver_mode_init(ctx) != 0) goto exit;
			break;
		default:
			printk(KERN_INFO"[zynq_adv762x]Working under unkown mode.\n");
			break;
	}

	return 0;
	
exit:	
	printk(KERN_INFO"[zynq_adv762x] Call adv762x_init() failed !!\n");
	return  -1;
}

static int adv762x_rls(struct adv762x_ctx  *ctx) {
	
	if (!ctx) return -1;
	
	switch (ctx->mode) {
		case MUXMODE:
			transceiver_mode_rls(ctx);
			break;
		case TRANSMODE:
			mux_mode_rls(ctx);
			break;
		default:
			break;
	}
	
	if (ctx->rx_main_map[0]) i2c_unregister_device(ctx->rx_main_map[0]);
	if (ctx->rx_main_map[1]) i2c_unregister_device(ctx->rx_main_map[1]);
	
	if (ctx->rx_repeater_map[0]) i2c_unregister_device(ctx->rx_repeater_map[0]);
	if (ctx->rx_repeater_map[1]) i2c_unregister_device(ctx->rx_repeater_map[1]);
	
	if (ctx->rx_information_map[0]) i2c_unregister_device(ctx->rx_information_map[0]);
	if (ctx->rx_information_map[1]) i2c_unregister_device(ctx->rx_information_map[1]);
	
	if (ctx->rx_edid_map[0]) i2c_unregister_device(ctx->rx_edid_map[0]);
	if (ctx->rx_edid_map[1]) i2c_unregister_device(ctx->rx_edid_map[1]);
	
	if (ctx->rx_test_map) i2c_unregister_device(ctx->rx_test_map);
	
	if (ctx->cp_lite_map[0]) i2c_unregister_device(ctx->cp_lite_map[0]);
	if (ctx->cp_lite_map[1]) i2c_unregister_device(ctx->cp_lite_map[1]);
	
	if (ctx->dpll_map[0])  i2c_unregister_device(ctx->dpll_map[0]);
	if (ctx->dpll_map[1]) i2c_unregister_device(ctx->dpll_map[1]);
	
	if (ctx->osd_map) i2c_unregister_device(ctx->osd_map);
	
	if (ctx->tx_main_map[0]) i2c_unregister_device(ctx->tx_main_map[0]);
	if (ctx->tx_main_map[1]) i2c_unregister_device(ctx->tx_main_map[1]);
	
	if (ctx->tx_packet_map[0]) i2c_unregister_device(ctx->tx_packet_map[0]);
	if (ctx->tx_packet_map[1]) i2c_unregister_device(ctx->tx_packet_map[1]);
	
	if (ctx->tx_cec_map[0]) i2c_unregister_device(ctx->tx_cec_map[0]);
	if (ctx->tx_cec_map[1]) i2c_unregister_device(ctx->tx_cec_map[1]);
	
	if (ctx->tx_edid_map[0]) i2c_unregister_device(ctx->tx_edid_map[0]);
	if (ctx->tx_edid_map[1]) i2c_unregister_device(ctx->tx_edid_map[1]);
	
	if (ctx->tx_test_map[0]) i2c_unregister_device(ctx->tx_test_map[0]);
	if (ctx->tx_test_map[1]) i2c_unregister_device(ctx->tx_test_map[1]);
	
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
static int get_edid_spa_location(const u8 *edid) {
	u8 d = 0;

	if ((edid[0x7e] != 1) ||
	    (edid[0x80] != 0x02) ||
	    (edid[0x81] != 0x03)) {
		return -1;
	}

	/* search Vendor Specific Data Block (tag 3) */
	d = edid[0x82] & 0x7f;
	if (d > 4) {
		int i = 0x84;
		int end = 0x80 + d;

		do {
			u8 tag = edid[i] >> 5;
			u8 len = edid[i] & 0x1f;

			if ((tag == 3) && (len >= 5))
				return i + 4;
			i += len + 1;
		} while (i < end);
	}
	return -1;
}

/*
 When the EDID/Repeater configures the internal EDID in power-down mode, it also updates the
SPA registers for each port according to the SPA read from the external SPI PROM. The 2-byte SPA
is located at the address specified by SPA_LOCATION in addresses 0x7F and 0xFF of the SPI
PROM. The SPA of each port is set as follows:

SPA for port A located in EDID RAM is set to A.B.C.D
SPA for port B, SPA_PORT_B[15:0], is set to A+1.B.C.D
SPA for port C, SPA_PORT_C[15:0], is set to A+2.B.C.D
SPA for port D, SPA_PORT_D[15:0], is set to A+3.B.C.D
where A.B.C.D is the 2-byte SPA read from the SPI PROM. The format A.B.C.D is described in
the HDMI specification.

SpaValues[0] MSB (i.e. SPA nibbles A.B) of spa port B
SpaValues[1] LSB (i.e. SPA nibbles C.D) of spa port B
SpaValues[2] MSB (i.e. SPA nibbles A.B) of spa port C
SpaValues[3] LSB (i.e. SPA nibbles C.D) of spa port C
SpaValues[4] MSB (i.e. SPA nibbles A.B) of spa port D
SpaValues[5] LSB (i.e. SPA nibbles C.D) of spa port D
SpaValues[6] MSB (i.e. SPA nibbles A.B) of spa port E
SpaValues[7] LSB (i.e. SPA nibbles C.D) of spa port E 

spa_port_b[15:12] = A
spa_port_b[11:8] = B
spa_port_b[7:4] = C
spa_port_b[3:0] = D

spa_port_b[15:0] , HDMI Rx Repeater Map, Address 0x52[7:0]; Address 0x53[7:0]
								  				
spa_location		: A B 
spa_location+1	: C D

edid_buf[spa_location] = 0xAB
edid_buf[spa_location+1] = 0xCD

 */
static int adv762x_edid(struct adv762x_ctx * ctx, unsigned char *edid_buf, unsigned int edid_len, RX_EDID_IMAGE edidIm){
	
	unsigned short SpaOffset = 0;
	unsigned  char aSpaValue[8];
	
	if (!ctx || !edid_buf) return -1;
	
	SpaOffset= get_edid_spa_location(edid_buf);
	
	aSpaValue[0] = edid_buf[SpaOffset]+0x10;
	aSpaValue[1] = edid_buf[SpaOffset+1];
	aSpaValue[2] = edid_buf[SpaOffset]+0x20;
	aSpaValue[3] = edid_buf[SpaOffset+1];
	aSpaValue[4] = edid_buf[SpaOffset]+0x30;
	aSpaValue[5] = edid_buf[SpaOffset+1];
	aSpaValue[6] = edid_buf[SpaOffset]+0x40;
	aSpaValue[7] = edid_buf[SpaOffset+1];
	
	adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x74, 0x00);
	
	switch(edidIm) {
		case RX_EDIDIM_512_A:
			adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x7a, 0x3, 0x0, 0x0);
			adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x70, 0xF0, 4, 0x4);
			adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x7a, 0x70, 4, 0x0);
			adv_smbus_write_i2c_block_data(ctx->rx_edid_map[0], 0, edid_len, edid_buf);
			if (SpaOffset) {
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x71, 0xFF, 0,  (SpaOffset & 0xff) );
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x70, 0x7, 0,    (SpaOffset & 0xff00)>>8);
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x52, 0x7, 0,  aSpaValue[0]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x53, 0x7, 0,  aSpaValue[1]); // LSB
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x54, 0x7, 0,  aSpaValue[2]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x55, 0x7, 0,  aSpaValue[3]); // LSB
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x56, 0x7, 0,  aSpaValue[4]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x57, 0x7, 0,  aSpaValue[5]); // LSB
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x58, 0x7, 0,  aSpaValue[6]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x59, 0x7, 0,  aSpaValue[7]); // LSB
     		}
     		break;
		case  RX_EDIDIM_256_B:
			adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x7a, 0x3, 0x0, 0x1);
			adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x70, 0xF0, 4, 0x2);
			adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x7a, 0x70, 4, 0x0);
			adv_smbus_write_i2c_block_data(ctx->rx_edid_map[0], 0, edid_len, edid_buf);
			if (SpaOffset) {
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x7F, 0xFF, 0,  (SpaOffset & 0xff) );
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x7E, 0x7, 0,    (SpaOffset & 0xff00)>>8);
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x52, 0x7, 0,  aSpaValue[0]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x53, 0x7, 0,  aSpaValue[1]); // LSB
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x54, 0x7, 0,  aSpaValue[2]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x55, 0x7, 0,  aSpaValue[3]); // LSB
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x56, 0x7, 0,  aSpaValue[4]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x57, 0x7, 0,  aSpaValue[5]); // LSB
				
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x58, 0x7, 0,  aSpaValue[6]);// MSB
				adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0],  0x59, 0x7, 0,  aSpaValue[7]); // LSB
     		}
     		break;
		default:
				return -1;
	}
     //re-calculation of all checksums for the internal E-EDID for all ports.
     adv_smbus_write_byte_field8 (ctx->rx_repeater_map[0], 0x77, 0x4, 0x2,  1);
     
	 /*##04 EDID Configuration##
	:04-01 Enable all EDIDs:
	64 74 1F ; default=0x0F, Enable all EDIDs 
	End*/
	adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x74, 0x1f);
	
	if (edidIm == RX_EDIDIM_512_A)
			adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x7d, 0x00);
	else if (edidIm == RX_EDIDIM_256_B)
			adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x7d, 0x17);
	
	adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x40, 0x81) ;
	adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x7c, 0x80) ;
	 
     return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
static inline void adv762x_delay(unsigned int msec) {
	
	//unsigned long timeout = jiffies + HZ * sec; 
	//while (time_before(jiffies, timeout)) cpu_relax(  );
	 msleep_interruptible(msec);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 0
static int adv762x_hw_reset(struct adv762x_ctx  *ctx) {
	unsigned int reset_gpio_pin = ADV762x_RST_PIN;
	int ret = 0;
	
	if (!ctx) return -1;
	
	ret = gpio_request(reset_gpio_pin, "ADV762x reset pin");
	if (ret) {
		printk(KERN_INFO"[zynq_adv762x] Call gpio_request() for  pin %u  error !!\n", reset_gpio_pin);
		return -1;
	}
	gpio_direction_output(reset_gpio_pin, 0);
	adv762x_delay(5);
	gpio_direction_output(reset_gpio_pin, 1);
	gpio_free(reset_gpio_pin);
	
	return 0;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////
static int adv762x_read_edid_form_bin_file(const char *bin_file_name, unsigned char *edid_buf){
	
	struct file *file_id = NULL;
	mm_segment_t fs_old;
	int err = 0;
	loff_t pos_t = (loff_t)0;
	
	if (!bin_file_name) {
		printk(KERN_INFO"[zynq_adv762x] bin_file_name is NULL!!\n");
       	 return -1;
	}
	
	file_id = filp_open(bin_file_name, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(file_id ) || !file_id) {
        printk(KERN_INFO"[zynq_adv762x] Opne file %s failed!!\n", bin_file_name);
        return -1;
    }

	 fs_old = get_fs();
     set_fs(KERNEL_DS);
	
	err = vfs_read(file_id, edid_buf, EDID_MEM_LEN,  &pos_t);
	if( err < 0 ){
		printk(KERN_INFO"[zynq_adv762x] Read file %s failed!!\n", bin_file_name);
		goto exit;
	}
	
exit:	 
	 if(file_id != NULL) {
        filp_close(file_id, NULL);
    	file_id= NULL;
		set_fs(fs_old);
	 }
	 if (err < 0)
	 	return -1;
	 else
		 return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/* I2C I/O operations */
static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command) {
	s32 ret = 0 ;	
	s32 i = 0;
	for (i = 0; i < 3; i++) {
		ret = i2c_smbus_read_byte_data(client, command);
		if (ret >= 0)
			return ret;
	}
	printk(KERN_INFO"[zynq_adv762x] Reading addr:%02x reg:%02x\n failed",
		client->addr, command);
	return ret;
}

static s32 adv_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value) {
	s32 ret = 0;
	s32 i =0;
	for (i = 0; i < 3; i++) {
		ret = i2c_smbus_write_byte_data(client, command, value);
		if (!ret)
			return 0;
	}

	printk(KERN_INFO"[zynq_adv762x] Writing addr:%02x reg:%02x val:%02x failed\n",
		client->addr, command, value);

	return ret;
}

static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client, u8 command,
					  u8 length, const u8 *values) {
	s32 ret = 0;	
	s32 i = 0;
	ret = i2c_smbus_write_i2c_block_data(client, command, length, values);
	if (!ret)
		return 0;

	for (i = 0; i < length; i++) {
		ret = adv_smbus_write_byte_data(client, command + i, values[i]);
		if (ret)
			break;
	}

	return ret;
}

static void  adv_smbus_write_byte_field8 (struct i2c_client *client, 
										  unsigned char RegAddr, unsigned int Mask, 
                         					unsigned int BitPos, unsigned char  FieldVal)
{
    unsigned char Val = 0;
    
    Val = adv_smbus_read_byte_data(client, RegAddr) & 0xff;
    Val = (Val & ~Mask) | ((FieldVal << BitPos) & Mask);
	adv_smbus_write_byte_data(client, RegAddr, Val); 
}

static struct i2c_client *adv_dummy_client(struct i2c_client *client,
					       u8 addr, u8 def_addr, u8 io_reg)
{
	int ret = 0;
	if (!addr)
		addr = def_addr;

	if (!client) return NULL;
	
	ret = adv_smbus_write_byte_data(client, io_reg, addr << 1);
	
	if (ret !=  0) return NULL;
	
	return i2c_new_dummy(client->adapter, addr);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_DEBUG_FS

static struct dentry *g_dentry = NULL;;

static int adv762x_edid_show(struct seq_file *s, void *unused) {

	u8 *buf = NULL;
	int i;

	buf = &(g_ctx.edid.edid_buf[0]);

	for (i = 0; i < EDID_MEM_LEN; i++) {
		if (i % 16 == 0)
			seq_printf(s, "edid[%03x] =", i);

		seq_printf(s, " %02x", buf[i]);

		if (i % 16 == 15)
			seq_printf(s, "\n");
	}
	
	if ((g_ctx.id == ADV7625) || (g_ctx.id == ADV7627)) {
		u8 val = 0;
		hdmi_query_cable_detect_status(&g_ctx, &val); 
	}
	
	return 0;
}

static int adv762x_edid_debug_open(struct inode *inode, struct file *file) {
	return single_open(file, adv762x_edid_show, inode->i_private);
}
static ssize_t adv762x_edid_debug_write(struct file *file, const char __user *userbuf, size_t count, loff_t *ppos) {

	//u8 value = 0;
	//struct adv762x_ctx  *ctx=&g_ctx;
	char buf[EDID_MEM_LEN];
	size_t read_size = 0;
   unsigned int i = 0;
	memset(buf, 0x0, sizeof(buf));
	
	if (count <= sizeof(buf) )
		read_size =count;
	else 
		read_size = sizeof(buf);
		
	if (copy_from_user(buf, userbuf, read_size))
		return -EFAULT;
	
	if (adv762x_debug) {
		printk(KERN_INFO"[zynq_adv762x]read_size = %u\n", read_size);
		for (i = 0; i < read_size; i++)
			printk(KERN_INFO"[%u] 0x%02x\n", i,buf[i]);
		printk(KERN_INFO"[zynq_adv762x]\n");
	}
	
	if (read_size != EDID_MEM_LEN) {
		printk(KERN_INFO"[zynq_adv762x]EDID size should be %u but actul size is  %u .\n", EDID_MEM_LEN,  read_size);
		return -EFAULT;
	}

	return count;
}
static const struct file_operations adv762x_edid_debug_fops = {
	.open		= adv762x_edid_debug_open,
	.read		= seq_read,
	.write          = adv762x_edid_debug_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void adv762x_edid_debug_add(struct adv762x_ctx *ctx) {
	char name[] = "adv762x-edid";
	g_dentry = debugfs_create_file(name, S_IRUGO | S_IWUGO, NULL, ctx, &adv762x_edid_debug_fops);
}

void adv762x_edid_debug_remove(struct adv762x_ctx *ctx) {
	if (g_dentry) debugfs_remove(g_dentry);
}

#else
void adv762x_edid_debug_add(struct adv762x_ctx *ctx){ }
void adv762x_edid_debug_remove(struct adv762x_ctx *ctx) { }
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char  HdmiRecomInitTable[] = {
    /*===============================
     * HDMI Mode
     *==============================*/
      0x01, 0x04,
      0x3D, 0x00,        /*ADI RS TMDSPLL GEAR MON TO 1*/
      0x3E, 0x79,        /*ADI RS TMDSPLL*/
      0x3F, 0x04,        /*ADI RS TMDSPLL*/
      0x4E, 0xFE,        /*ADI RS TMDSPLL*/
      0x4F, 0x08,        /*ADI RS TMDSPLL*/
      0x57, 0xF5,        /*ADI RS TMDSPLL*/ /* 0x57, 0xA3,*/
      0x58, 0x1C,        /*ADI RS TMDSPLL*/ /* 0x58, 0x04,*/
      0x6F, 0x04,        /*ADI RX VSYNC FILT, DVI DETECT*/
      0x75, 0x00,        /*ADI RX Clk5X DC, VCO Setting*/
      0x85, 0x10,        /*ADI RX Equalizer Setting*/
      0x97, 0xC0,        /*ADI RS TMDSPLL*/
      0x98, 0x3F,          /*ADI Recommended Write*/
      0x99, 0xE3,          /*ADI Recommended Write*/
      0x9A, 0x9F,          /*ADI Recommended Write*/
      0x9B, 0x01,          /*ADI Recommended Write*/
      0x9C, 0x10,        /*ADI RS TMDSPLL*/
      /*0xC0, 0x10, */       /*ADI RS TMDSPLL Power Stepper Disabled*/
      0xCB, 0x01,        /*ADI RS TMDSPLL Power Stepper Disabled*/
      0, 0

};
////////////////////////////////////////////////////////////////////////////////////////
static int hdmi_query_cable_detect_status(struct adv762x_ctx  *ctx, u8 *value) {
		
	/*
	 * The ADV7625 can monitor the level on the input +5 V power signal pin of each connected HDMI input. The results of this detection can
	 * be read back from the following I 2 C registers.
	 */
	u8 v = 0;
	u8 v_rxa = 0;
	u8 v_rxb = 0;
	u8 v_rxc = 0;
	u8 v_rxd = 0;
	u8 v_rxe = 0;
	
	if (!ctx) return -1;
		
	v = (u8) adv_smbus_read_byte_data(ctx->io_client, 0x52);
	
	v_rxa = (v & 0x04) >> 2;
	v_rxb = (v & 0x08) >> 3;
	v_rxc = (v & 0x10) >> 4;
	v_rxd = (v & 0x20) >> 5;
	v_rxe = (v & 0x40) >> 6;
	
	printk(KERN_INFO"[zynq_adv762x] HDMI Cable detect staus (0x%02x) : (RXA, RXB, RXC, RXD, RXE) = (0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x)\n", v , v_rxa, v_rxb, v_rxc, v_rxd, v_rxe);
		
	*value = v;
	
	return 0;
}
static int hdmi_hot_plug_assert (struct adv762x_ctx  *ctx, ADV762X_HPA ctrl) {
	
	if (!ctx) return -1;
	
	switch (ctrl) {
		case EDIDACTIVE:
			/*
		 	* Hot Plug Config:
		 	* (1)disable manual control for the Hot Plug Assert output pins, 
		 	* (2)The HPA of a HDMI port is asserted high immediately after the internal EDID has been activated for that port. The HPA of a specific HDMI port is de-asserted low immediately after the internal E-EDID is de-activated for that port.
		 	* (3)1010:1s (The delay is a between +5 V detection and hot plug assertion on the HPA output pins.)
			*/
			adv_smbus_write_byte_data(ctx->io_client, 0xb1, 0x0a);
			break;
		case CABLEDECT:
			/*
		 	* Hot Plug Config:
		 	* (1)disable manual control for the Hot Plug Assert output pins, 
		 	* (2) The HPA of a HDMI port is asserted high following a programmable delay after the part detects an HDMI cable plug on that port. The HPA of an HDMI port is immediately de-asserted 
		 	* after the part detects a cable disconnect on that HDMI port.
		 	* (3)1010:1s (The delay is a between +5 V detection and hot plug assertion on the HPA output pins.)
			*/
			adv_smbus_write_byte_data(ctx->io_client, 0xb1, 0x1a);
			break;
		case EDIDACTIVE_AND_CABLEDECT:
			/*
		 	* Hot Plug Config:
		 	* (1)disable manual control for the Hot Plug Assert output pins, 
		 	* (2) The HPA of a HDMI port is asserted high after two conditions have been met. The conditions are detailed as follows : 
		 	* 	1) The internal EDID is active for that port. 
		 	* 	2) The delayed version of the cable detect signal cable_det_x_raw for that port is high. 
		 	* The HPA of an HDMI port is immediately de-asserted after any of the following two conditions have been met : 
		 	*	1) The internal EDID is de-activated for that port. 
		 	* 2) The cable detect signal cable_det_x_raw for that port is low.
		 	*(3)1010:1s (The delay is a between +5 V detection and hot plug assertion on the HPA output pins.)
			*/
			adv_smbus_write_byte_data(ctx->io_client, 0xb1, 0x2a);
			break;
		default:
			return 0;
	}
	
	//	if(adv_smbus_write_byte_data(ctx->io_client, 0xb2, 0x1f) != 0) goto exit;//Set Hot Plug 
	
	return 0;
}

static int hdmi_audio_output_mclk (struct adv762x_ctx  *ctx, ADV762X_MCLK_DIV mclk_div) {
	
	u8 val = 0;
	
	if (!ctx) return -1;
	
	
	switch (mclk_div) {
		case FS128:
			val = 0x0; break;
		case FS256:
			val = 0x1; break;
		case FS384:
			val = 0x2; break;
		case FS512:
			val = 0x3; break;
		case FS640:
			val = 0x4; break;
		case FS768:
			val = 0x5; break;
		default:
			return 0;
	}
	
	if (ctx->id == ADV7625) {
		adv_smbus_write_byte_data(ctx->dpll_map[0], 0xb5, val); 
		adv_smbus_write_byte_data(ctx->dpll_map[1], 0xb5, val);
	} else if (ctx->id == ADV7627) {
		adv_smbus_write_byte_data(ctx->dpll_map[1], 0xb5, val);
	} 
	
	return 0;
}

static int hdmi_audio_output_i2s_map(struct adv762x_ctx  *ctx, unsigned int map_mode, unsigned int invert) {

		u8 val  = 0;
		u8 reg = 0x6d;
		int i  = 0;
		int num = 1;
		
		if (ctx->id == ADV7625)  num  = 2;
		
		/*	
			i2s_spdif_map_rot[1:0] (0x6d [5:4] )
			(1)00:	[I2S0/SPDIF0 on APx_OUT1] [I2S1/SPDIF1 on APx_OUT2] [I2S2/SPDIF2 on APx_OUT3] [I2S3/SPDIF3 on APx_OUT4]
			(2)01:	[I2S3/SPDIF3 on APx_OUT1] [I2S0/SPDIF0 on APx_OUT2] [I2S1/SPDIF1 on APx_OUT3] [I2S2/SPDIF2 on APx_OUT4]
			(3)10:	[I2S2/SPDIF2 on APx_OUT1] [I2S3/SPDIF3 on APx_OUT2] [I2S0/SPDIF0 on APx_OUT3] [I2S1/SPDIF1 on APx_OUT4]
			(4)11:  [I2S1/SPDIF1 on APx_OUT1] [I2S2/SPDIF2 on APx_OUT2] [I2S3/SPDIF3 on APx_OUT3] [I2S0/SPDIF0 on APx_OUT4]
		*/
		
		for (i = 0 ; i < num; i ++) {
		 		val = (u8) adv_smbus_read_byte_data(ctx->rx_main_map[i], reg);
				switch (map_mode) {
					case 0:
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0xcf) |0x00 ); break;
					case 1:
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0xcf) |0x10 ); break;
					case 2:
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0xcf) |0x20 ); break;
					case 3:
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0xcf) |0x30 ); break;
					default :
						break;
				}
		}
		
		/*
 			I2S_SPDIF_MAP_INV[0] (0x6d [6])
			(1)0 - Do not invert arrangement of I2S/SPDIF channels in audio output port pins
			(2)1 - Invert arrangement of I2S/SPDIF channels in audio output port pins
		*/
		for (i = 0 ; i < num; i ++) {
		 		val = (u8) adv_smbus_read_byte_data(ctx->rx_main_map[i], reg);
				switch ( invert) {
					case 0:
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0xbf) | 0x00 ); break;
					case 1:
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0xbf) |0x40 ); break;
					default :
						break;
				}
		 }
		
		return 0;
}



static int hdmi_audio_output_fmt (struct adv762x_ctx  *ctx, unsigned int mode, unsigned int bitwidth) {
	
		u8 val  = 0;
		u8 reg = 0x03;
		int i  = 0;
		int num = 1;
		
		if (ctx->id == ADV7625)  num  = 2;
		
		
		for (i = 0 ; i < num; i ++) {
		 		val = (u8) adv_smbus_read_byte_data(ctx->rx_main_map[i], reg);
				switch (mode) {
					case 0: //I2S
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0x9f) |0x00 ); break;
					case 1: //Right Justified
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0x9f) |0x20 ); break;
					case 2: //Left Justified
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0x9f) |0x40 ); break;
					case 3: //RAW SPDIF (IEC60958) 
						adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0x9f) |0x60 ); break;
					default :
						break;
				}
		}
		
		for (i = 0 ; i < num; i ++) {
			val = (u8) adv_smbus_read_byte_data(ctx->rx_main_map[i], reg);
			adv_smbus_write_byte_data(ctx->rx_main_map[i], reg, (val & 0xe0) | bitwidth); 
		 }
	return 0;
}


////////////////////////////////////////////////////////////////////////////////////////
static int transceiver_mode_init(struct adv762x_ctx  *ctx) {
	
	//int i = 0;
	unsigned char *edid_buf = NULL ;
	u8 cable_detec_status = 0;
	
	if (!ctx) goto exit;
	
	edid_buf = &(ctx->edid.edid_buf[0]);
	
	if (ctx->id == ADV7625) {
		//ADV7625/26 Transceiver Mode
		if(adv_smbus_write_byte_data(ctx->io_client, 0x00, 0xc0) != 0) goto exit;  //Power Up DPLL_A, DPLL_B, xtal 
		if(adv_smbus_write_byte_data(ctx->tx_main_map[0], 0x41, 0x10) != 0) goto exit;  // Power up TxA 
		if(adv_smbus_write_byte_data(ctx->tx_main_map[1], 0x41, 0x10) != 0) goto exit;  //Power up TxB 
		
		hdmi_audio_output_mclk (ctx, FS128);
		
		if (ctx->hdmi_src_id == TK1HDMI) {//RXC
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x8a) != 0) goto exit; //HDMI Receivers Rx1 & Rx2 enabled, RxA-Rx1, RxC-Rx2 
		} else if (ctx->hdmi_src_id == CON18HDMI){ //RXE
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x8c) != 0) goto exit; //HDMI Receivers Rx1 & Rx2 enabled, RxA-Rx1, RxE-Rx2 
		} else {//RXB
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x89) != 0) goto exit; //HDMI Receivers Rx1 & Rx2 enabled, RxA-Rx1, RxB-Rx2
		}
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x08, 0x12) != 0) goto exit;//(1)TXA:no from RX1, (2)TXB:from RX2
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x09, 0x11) != 0) goto exit;// ADI Required Write 
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x0a, 0x12) != 0) goto exit;//(1) AP1_OUT: from RX1, (2)AP2_OUT:from RX2
		
		//Audio output fmt: (1) I2S mode, (2)bitwidth is 16 bits
		hdmi_audio_output_fmt (ctx, 0x00, 16);
		
		//I2S0 --->  AP1_OUT1 I2S0 --->AP2_OUT1
		hdmi_audio_output_i2s_map(ctx, 0x00, 0);
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x0f, 0x0c) != 0) goto exit; // Default is 0x08,  - AP1_IN and AP2_IN selected as the audio input ports. Pixel Port Input not available.
		
		 hdmi_hot_plug_assert(ctx, EDIDACTIVE);
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0xb3, 0xdf) != 0) goto exit;//ADI Required Write 
		if(adv_smbus_write_byte_data(ctx->io_client, 0x9d, 0x00) != 0) goto exit;//Turn off tristate on AP1_OUT 
		if(adv_smbus_write_byte_data(ctx->io_client, 0x9e, 0x00) != 0) goto exit;//Turn off tristate on AP2_OUT 
		if(adv_smbus_write_byte_data(ctx->cp_lite_map[0], 0xc9, 0x2d) != 0) goto exit;//CP_Lite_A Disable Auto Parameter Buffering - ADI Required Write 
		if(adv_smbus_write_byte_data(ctx->cp_lite_map[1], 0xc9, 0x2d) != 0) goto exit;//CP_Lite_B Disable Auto Parameter Buffering - ADI Required Write 
#if 0
		i = 0;
		while (HdmiRecomInitTable[i]) {
                   adv_smbus_write_byte_data(ctx->rx_main_map[0], HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]);
                   i+= 2;
		}
		i = 0;
		while (HdmiRecomInitTable[i]) {
                   adv_smbus_write_byte_data(ctx->rx_main_map[1], HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]);
                   i+= 2;
		}
#endif
		adv762x_edid(ctx, edid_buf, EDID_MEM_LEN, RX_EDIDIM_512_A);
		adv_smbus_write_byte_field8 (ctx->io_client, 0x00, 0x80, 0x7, 0x0);//Power up HDMI Rx1
		adv_smbus_write_byte_field8 (ctx->io_client, 0x00, 0x40, 0x6, 0x0);//Power up HDMI Rx2
	} else if (ctx->id == ADV7627) {
		//ADV7627 Transceiver Mode
		if(adv_smbus_write_byte_data(ctx->io_client, 0x00, 0xe0) != 0) goto exit;  //(1) power down RX1, (2)power down RX2, (3)Power down DPLL A, (4)Power up DPLL B, (5)Power up xtal amplifier, (6)Power down depending on individual power down bits, (7)Power down depending on individual power down bits 
		if(adv_smbus_write_byte_data(ctx->tx_main_map[1], 0x41, 0x10) != 0) goto exit;  //Power up Tx 
		
		hdmi_audio_output_mclk (ctx, FS128);
		
		if (ctx->hdmi_src_id == TK1HDMI) { //RXC
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x7a) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXC as the input port of RX2 
		} else if (ctx->hdmi_src_id == CON18HDMI) { //RXE
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x7c) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXE as the input port of RX2 
		} else {//RXB
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x79) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXB as the input port of RX2 
		}
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x08, 0x02) != 0) goto exit;  //(1)TXA:no audio, (2)TXB:from RX2
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x09, 0x51) != 0) goto exit;// ADI Required Write
		
		/*Visgae use the following pins for audio output :
		 * (1) F4 (AP2_OUT1) as the DOUT pin
		 * (2) H4 (AP2_OUT5) as the LRCK pin
		 * (3) J1 (AP2_OUT_MCLK) as the MCLK pin
		 * (4) J2 (AP2_OUT_SCLK) as the BCLK pin
		 * (5) These pin will be connected to PCM5100 (DAC)  and the output of PCM5100 (DAC_L and DAC_R) will be the input of PCM1865 (ADC).
		 * (6) PCM5100 only  Accepts 16-, 24-, And 32-Bit Audio Data and PCM data format only is I2S or Left-Justified. Sample rates is  up to 384kHz.
		 * (7) The FMT pin  ( Audio format selection : I2S (Low) / Left justified (High)) of PCM5100 is LOW.  
		 */
		if(adv_smbus_write_byte_data(ctx->io_client, 0x0a, 0x02) != 0) goto exit; //(1) AP1_OUT: no audio, (2)AP2_OUT:from RX2
		//if(adv_smbus_write_byte_data(ctx->io_client, 0x0a, 0x20) != 0) goto exit; // (1) AP1_OUT: from RX2, (2)AP2_OUT:no audio
		
		//Audio output fmt: (1) I2S mode, (2)bitwidth is 16 bits
		hdmi_audio_output_fmt (ctx, 0x00, 16);
		
		//I2S0 --->  AP2_OUT1
		hdmi_audio_output_i2s_map(ctx, 0x00, 0);
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x0f, 0x0c) != 0) goto exit; // Default is 0x08,  - AP1_IN and AP2_IN selected as the audio input ports. Pixel Port Input not available.
		
		 hdmi_hot_plug_assert(ctx, EDIDACTIVE);
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0xb3, 0xdf) != 0) goto exit;// ADI Required Write 
		if(adv_smbus_write_byte_data(ctx->io_client, 0x9e, 0x00) != 0) goto exit;// Turn off tristate on AP2_OUT 
		if(adv_smbus_write_byte_data(ctx->cp_lite_map[1], 0xc9, 0x2d) != 0) goto exit;//  CP_Lite Disable Auto Parameter Buffering - ADI Required Write 
#if 0
		i = 0;
		while (HdmiRecomInitTable[i]) {
                   adv_smbus_write_byte_data(ctx->rx_main_map[1], HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]);
                   i+= 2;
		}
#endif
		adv762x_edid(ctx, edid_buf, EDID_MEM_LEN, RX_EDIDIM_512_A);
		adv_smbus_write_byte_field8 (ctx->io_client, 0x00, 0x40, 0x6, 0x0);//Power up HDMI Rx2
	}
	
	if ((g_ctx.id == ADV7625) || (g_ctx.id == ADV7627)) {
		hdmi_query_cable_detect_status(ctx, &cable_detec_status); 
	}
	return 0;
exit:
	return -1;
}

static int mux_mode_init(struct adv762x_ctx  *ctx) {
	
	//int i = 0;
	unsigned char *edid_buf = NULL ;
	u8 cable_detec_status = 0;
	
	if (!ctx) goto exit;
	
	edid_buf = &(ctx->edid.edid_buf[0]);
	
	if (ctx->id == ADV7625) {
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x00, 0xf0) != 0) goto exit;  //Power Up xtal 
		if(adv_smbus_write_byte_data(ctx->tx_main_map[0], 0x41, 0x10) != 0) goto exit;  //  Power up TxA 
		if(adv_smbus_write_byte_data(ctx->tx_main_map[1], 0x41, 0x10) != 0) goto exit;  //Power up TxB 
		if(adv_smbus_write_byte_data(ctx->io_client, 0x09, 0x11) != 0) goto exit;// ADI Required Write 
		
		if (ctx->hdmi_src_id == TK1HDMI) {//RXC
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x8a) != 0) goto exit; //HDMI Receivers Rx1 & Rx2 enabled, RxA-Rx1, RxC-Rx2 
		} else if (ctx->hdmi_src_id == CON18HDMI){ //RXE
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x8c) != 0) goto exit; //HDMI Receivers Rx1 & Rx2 enabled, RxA-Rx1, RxE-Rx2 
		} else {//RXB
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x89) != 0) goto exit; //HDMI Receivers Rx1 & Rx2 enabled, RxA-Rx1, RxB-Rx2
		}
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x03, 0xe6) != 0) goto exit;//Enable Mux Mode DataPath A 
		if(adv_smbus_write_byte_data(ctx->io_client, 0x04, 0xe6) != 0) goto exit;//Enable Mux Mode DataPath B 
		
		 hdmi_hot_plug_assert(ctx, EDIDACTIVE);
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0xb3, 0xdf) != 0) goto exit;//ADI Required Write 
#if 0
		i = 0;
		while (HdmiRecomInitTable[i]) {
                   adv_smbus_write_byte_data(ctx->rx_main_map[0], HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]);
                   i+= 2;
		}
		i = 0;
		while (HdmiRecomInitTable[i]) {
                   adv_smbus_write_byte_data(ctx->rx_main_map[1], HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]);
                   i+= 2;
		}
#endif
		adv762x_edid(ctx, edid_buf, EDID_MEM_LEN, RX_EDIDIM_512_A);
		adv_smbus_write_byte_field8 (ctx->io_client, 0x00, 0x80, 0x7, 0x0);//Power up HDMI Rx1
		adv_smbus_write_byte_field8 (ctx->io_client, 0x00, 0x40, 0x6, 0x0);//Power up HDMI Rx2
	} else if (ctx->id == ADV7627) {
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x00, 0xf0) != 0) goto exit;  //(1) power down RX1, (2)power down RX2, (3)Power down DPLL A, (4)Power up DPLL B, (5)Power up xtal amplifier, (6)Power down depending on individual power down bits, (7)Power down depending on individual power down bits 
		if(adv_smbus_write_byte_data(ctx->tx_main_map[1], 0x41, 0x10) != 0) goto exit;  //default=0x50, Power up Tx  
		if(adv_smbus_write_byte_data(ctx->io_client, 0x09, 0x51) != 0) goto exit;// ADI Required Write
		
		if (ctx->hdmi_src_id == TK1HDMI) { //RXC
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x7a) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXC as the input port of RX2 
		} else if (ctx->hdmi_src_id == CON18HDMI) { //RXE
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x7c) != 0) goto exit;  // HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXE as the input port of RX2 
		} else {//RXB
			if(adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x79) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXB as the input port of RX2 
		}
		
		if(adv_smbus_write_byte_data(ctx->io_client, 0x04, 0xe6) != 0) goto exit;// Enable Mux Mode DataPath B 
		
		 hdmi_hot_plug_assert(ctx, EDIDACTIVE);
	
		if(adv_smbus_write_byte_data(ctx->io_client, 0xb3, 0xdf) != 0) goto exit;//ADI Required Write 
#if 0
		i = 0;
		while (HdmiRecomInitTable[i]) {
                   adv_smbus_write_byte_data(ctx->rx_main_map[1], HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]);
                   i+= 2;
		}
#endif
		adv762x_edid(ctx, edid_buf, EDID_MEM_LEN, RX_EDIDIM_512_A);
		adv_smbus_write_byte_field8 (ctx->io_client, 0x00, 0x40, 0x6, 0x0);//Power up HDMI Rx2
	} 
	
	
	if ((g_ctx.id == ADV7625) || (g_ctx.id == ADV7627)) {
		hdmi_query_cable_detect_status(ctx, &cable_detec_status); 
	}
	
	return 0;
exit:
	return -1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
static int transceiver_mode_rls(struct adv762x_ctx  *ctx) {
	
	if (!ctx) return -1;
	
	//Disable the edids of all ports
	adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x74, 0x00);
	//Disable RX1 and RX2
	adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x77);
	//Power down TX
	adv_smbus_write_byte_data(ctx->tx_main_map[0], 0x41, 0x50);//default=0x50, 01010000
	adv_smbus_write_byte_data(ctx->tx_main_map[1], 0x41, 0x50);//default=0x50, 01010000
	//Power down RX1, RX2,  DPLLA, DPLLB, CORE, master
	adv_smbus_write_byte_data(ctx->io_client, 0x00, 0xf3);
	
	return 0;	
}
/////////////////////////////////////////////////////////////////////////////////////////////////
static int mux_mode_rls(struct adv762x_ctx  *ctx) {
	
	if (!ctx) return -1;
	
	//Disable the edids of all ports
	adv_smbus_write_byte_data(ctx->rx_repeater_map[0], 0x74, 0x00);
	//Disable RX1 and RX2
	adv_smbus_write_byte_data(ctx->io_client, 0x01, 0x77);
	//Power down TX
	adv_smbus_write_byte_data(ctx->tx_main_map[0], 0x41, 0x50);//default=0x50, 01010000
	adv_smbus_write_byte_data(ctx->tx_main_map[1], 0x41, 0x50);//default=0x50, 01010000
	//Power down RX1, RX2,  DPLLA, DPLLB, CORE, master
	adv_smbus_write_byte_data(ctx->io_client, 0x00, 0xf3);
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////
