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

//extern unsigned int adv762x_use_fixed_edid;
extern unsigned int adv762x_debug;

#define TX_HDCP_AUTH_INT_BIT            0x02
#define TX_EDID_READY_INT_BIT           0x04
#define TX_EMB_SYNC_ERR_INT_BIT         0x08
#define TX_AUD_FIFO_FULL_INT_BIT        0x10
#define TX_VSYNC_EDGE_INT_BIT           0x20
#define TX_MSEN_CHNG_INT_BIT            0x40
#define TX_HPD_CHNG_INT_BIT             0x80
#define TX_INTS_ALL_BITS                0xFE
#define TX_VAL_CHARGE_INJ               0x39

#define ADV762x_RST_PIN GPIO_PK3
#define ADV762x_INTX_PIN GPIO_PK4
/*
The ADV7625 features a 768-byte internal EDID memory which can be configured in a number of different manners; it can be
configured as dual 256-byte EDIDs which can be distributed to two of the input ports or it can be configured for one extended EDID
(512-byte) and one 256-byte EDID
*/

#define EDID_SEGMENT_SIZE 256
#define TX_SUPPORTED_EDID_SEGMENTS 2
//static unsigned int EdidSegCount = 0;
//static  unsigned char EdidBuf[TX_SUPPORTED_EDID_SEGMENTS * 256] = {0x0};

#define EDID_MEM_LEN  256*TX_SUPPORTED_EDID_SEGMENTS

#define ADV762X_DRIVER_NAME "adv762x"

#define EDID_BIN_NAME		"/lib/firmware/EDID/edid.bin"

struct adv762x_edid {
	unsigned char  edid_buf[EDID_MEM_LEN];
};

struct adv762x_ctx {
	ADV762x_HDMI_SRC_ID hdmi_src_id;
	ADV762x_MODE mode;
	struct adv762x_edid edid;
	struct adv762x_edid original_edid;
	unsigned int rx_main_map;
	unsigned int edid_config_map;
	unsigned int rx_repeater_map;
	unsigned int rx_information_map;
	unsigned int rx_edid_map;
	unsigned int rx_test_map;
	unsigned int cp_lite_map;
	unsigned int dpll_map;
	unsigned int osd_map;
	unsigned int tx_main_map;
	unsigned int tx_packet_map;
	unsigned int tx_cec_map;
	unsigned int tx_edid_map;
	unsigned int tx_test_map;
	
	struct i2c_client *io;
	struct i2c_client *rx_main;
	struct i2c_client *edid_config;
	struct i2c_client *rx_repeater;
	struct i2c_client *rx_information;
	struct i2c_client *rx_edid;
	struct i2c_client *rx_test;
	struct i2c_client *cp_lite;
	struct i2c_client *dpll;
	struct i2c_client *osd;
	struct i2c_client *tx_main;
	struct i2c_client *tx_packet;
	struct i2c_client *tx_cec;
	struct i2c_client *tx_edid;
	struct i2c_client *tx_test;
	int irq;
	
	struct workqueue_struct			*work_queue;
	struct work_struct			interrupt_service;
	unsigned int edid_segment_num;
	unsigned int edid_segment_index; 
	unsigned int tx_edid_read_complete;
	struct mutex  lock;
	unsigned int is_initial_interrupt;
	unsigned int adv762x_use_fixed_edid;
} ;
struct adv762x_ctx  g_ctx;

static atomic_t g_interrupt_sevice_done;
static atomic_t g_interrupt_sevice_first_run;
extern unsigned char  HdmiRecomInitTable[];

static int create_control_sysfs(struct kobject *kobj) ;
static int destroy_control_sysfs(struct kobject *kobj);

void adv762x_edid_debug_add(struct adv762x_ctx *ctx);
void adv762x_edid_debug_remove(struct adv762x_ctx *ctx);

static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command);
static s32 adv_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value);
static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client, u8 command,
					  unsigned int length, const u8 *values);
static struct i2c_client *adv_dummy_client(struct i2c_client *client,
					       u8 addr, u8 def_addr, u8 io_reg);
static void  adv_smbus_write_byte_field8 (struct i2c_client *client, 
										  unsigned char RegAddr, unsigned int Mask, 
                         					unsigned int BitPos, unsigned char  FieldVal);

static int adv762x_hw_reset(struct adv762x_ctx  *ctx);
static int adv762x_init(struct adv762x_ctx  *ctx);
static int adv762x_rls(struct adv762x_ctx  *ctx);
static inline void adv762x_delay(unsigned int msec);

static int transceiver_mode_init(struct adv762x_ctx  *ctx);
static int mux_mode_init(struct adv762x_ctx  *ctx);

static int transceiver_mode_rls(struct adv762x_ctx  *ctx);
static int mux_mode_rls(struct adv762x_ctx  *ctx);

static int interrupt_init(struct adv762x_ctx  *ctx);
static int interrupt_rls(struct adv762x_ctx  *ctx);

//static int hdmi_audio_output_fmt (struct adv762x_ctx  *ctx, unsigned int mode, unsigned int bitwidth);
static int hdmi_query_cable_detect_status(struct adv762x_ctx  *ctx, u8 *value);
static int hdmi_hot_plug_assert (struct adv762x_ctx  *ctx, ADV762X_HPA ctrl);
static int hdmi_audio_output_mclk (struct adv762x_ctx  *ctx, ADV762X_MCLK_DIV mclk_div);
//static int hdmi_audio_output_i2s_map(struct adv762x_ctx  *ctx, unsigned int map_mode, unsigned int invert);


static unsigned int ctrl_set_change_status(unsigned int val);
static unsigned int ctrl_get_change_status(void);

static unsigned int ctrl_set_initial_status(unsigned int val);
static unsigned int ctrl_get_initial_status(void);

typedef enum {
	RX_EDIDIM_512_A = 0,
	RX_EDIDIM_256_B = 1
}RX_EDID_IMAGE;

static int adv762x_edid(struct adv762x_ctx * ctx,  RX_EDID_IMAGE edidIm);
static int adv762x_read_edid_form_bin_file(const char *bin_file_name, struct adv762x_ctx  *ctx);


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
	
	printk(KERN_INFO "[zynq_adv762x]Chip found @ 0x%02x (bus:%u)(ptr:%p)\n", client->addr, adapter->nr,  client);

	memset(&g_ctx, 0x0, sizeof(struct adv762x_ctx));
	
	g_ctx.hdmi_src_id = pdata->hdmi_src_id;
	g_ctx.mode =pdata->mode;
	g_ctx.io = client;
	g_ctx.rx_main_map  = pdata->rx_main_map;
	g_ctx.edid_config_map  = pdata->edid_config_map;
	g_ctx.rx_repeater_map  = pdata->rx_repeater_map;
	g_ctx.rx_information_map  = pdata->rx_information_map;
	g_ctx.rx_edid_map  = pdata->rx_edid_map;
	g_ctx.rx_test_map  = pdata->rx_test_map;
	g_ctx.cp_lite_map  = pdata->cp_lite_map;
	g_ctx.dpll_map  = pdata->dpll_map;
	g_ctx.osd_map  = pdata->osd_map;
	g_ctx.tx_main_map  = pdata->tx_main_map;
	g_ctx.tx_packet_map  = pdata->tx_packet_map;
	g_ctx.tx_cec_map  =  pdata->tx_cec_map;
	g_ctx.tx_edid_map  = pdata->tx_edid_map;
	g_ctx.tx_test_map  = pdata->tx_test_map;
	g_ctx.is_initial_interrupt = pdata->is_initial_interrupt;
	g_ctx.adv762x_use_fixed_edid = pdata->adv762x_use_fixed_edid;
	g_ctx.irq = -1;
	g_ctx.work_queue = NULL;
	g_ctx.edid_segment_num = 0;
	mutex_init(&g_ctx.lock);
	
	create_control_sysfs(&client->dev.kobj);
	
	if (adv762x_read_edid_form_bin_file(EDID_BIN_NAME, &g_ctx) != 0) goto exit;
	printk(KERN_INFO "[zynq_adv762x] ADV762X hdmi_src_id  is  %u\n", g_ctx.hdmi_src_id);
	printk(KERN_INFO "[zynq_adv762x] ADV762X mode  is  %u\n", g_ctx.mode);
	
	if (adv762x_init(&g_ctx) != 0) goto exit;

	adv762x_edid_debug_add(&g_ctx);
	
	atomic_set(&g_interrupt_sevice_done, -1);
	atomic_set(&g_interrupt_sevice_first_run, -1);
exit:
	printk(KERN_INFO "[zynq_adv762xi] Leave  adv762x_probe()\n");
	return  0;
}

static int adv762x_remove(struct i2c_client *client) {
	printk(KERN_INFO "[zynq_adv762x] Enter  adv762x_remove()\n");
	if (client) destroy_control_sysfs(&client->dev.kobj);
	adv762x_edid_debug_remove(&g_ctx);
	adv762x_rls(&g_ctx);
	printk(KERN_INFO "[zynq_adv762x] Leave  adv762x_remove()\n");
    return 0;
}

static const struct i2c_device_id adv762x_id[] = {
    { "adv7627", 0 },
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
static int adv762x_check_tx_monitor_present_status(struct adv762x_ctx *ctx) {
	u8 tx_state = 0;
	u8 hpd_state = 0;	
		
	if (!ctx) return -1;
	
	tx_state = (u8)adv_smbus_read_byte_data(ctx->tx_main, 0x42);
	hpd_state = (tx_state & 0x40) >> 0x6;
	printk(KERN_INFO"[adv762x] (%d) TX HPD state is 0x%02x\n", __LINE__, hpd_state);
	
	
	if (hpd_state) {
		// For trigger the HPD interrupt:
		adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x50); //Power down TX
		adv762x_delay(100);
		adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x10 ); //Power on TX
		adv_smbus_write_byte_data(ctx->io, 0xb2, 0x1f);
	}

	return  0;
}

static int adv762x_init(struct adv762x_ctx  *ctx){
	
	if (!ctx) goto exit;
	
		//d_info[15:0], IO Map, Address 0xDF[7:0]; Address 0xE0[7:0] (Read Only)
	//rd_info[15:0]: 0x4080, 0x4081
	printk(KERN_INFO"[zynq_adv762x]Chip ID High Byte Chip ID High Byte (should be 0x40):%02x\n", 	adv_smbus_read_byte_data(ctx->io, 0xdf));
	printk(KERN_INFO"[zynq_adv762x]Chip ID Low Byte Chip ID Low Byte (shoudl be 0x81):%02x\n", adv_smbus_read_byte_data(ctx->io,0xe0));
	
	
	if (adv_smbus_write_byte_data(ctx->io, 0xff, 0xff) != 0) goto exit; // Reset
	adv762x_delay(3000) ;//1000ms delay after Reset 
	if(adv_smbus_write_byte_data(ctx->io, 0xe5, 0x82) != 0) goto exit; //ADI Required Write 
	

	ctx->dpll = adv_dummy_client(ctx->io, ctx->dpll_map, (0x90 >> 1), 0xe7);//adv7627
	if (ctx->dpll == NULL) goto exit;
	
	ctx->cp_lite = adv_dummy_client(ctx->io, ctx->cp_lite_map, (0x94 >> 1), 0xe9);//adv7627
	if (ctx->cp_lite == NULL) goto exit;
	
	ctx->osd = adv_dummy_client(ctx->io, ctx->osd_map, (0xf0 >> 1), 0xea);
	if (ctx->osd == NULL) goto exit;
	
	ctx->edid_config = adv_dummy_client(ctx->io, ctx->edid_config_map, (0x6a >> 1), 0xee);//adv7627 (edid config)
	ctx->rx_repeater = adv_dummy_client(ctx->io, ctx->rx_repeater_map, (0x66 >> 1), 0xf2);//adv7627
	if ((ctx->edid_config == NULL) || (	ctx->rx_repeater == NULL)) goto exit;
	

	ctx->rx_main = adv_dummy_client(ctx->io, ctx->rx_main_map, (0x58 >> 1), 0xf0);//adv7627
	if (ctx->rx_main== NULL) goto exit;

	ctx->rx_information = adv_dummy_client(ctx->io, ctx->rx_information_map, (0x62 >> 1), 0xf3);//adv7627
	if (ctx->rx_information == NULL) goto exit;
	
	ctx->rx_edid = adv_dummy_client(ctx->io, ctx->rx_edid_map, (0x7e >> 1), 0xed);
	if (ctx->rx_edid  == NULL) goto exit;
	
	ctx->rx_test = adv_dummy_client(ctx->io, ctx->rx_test_map, (0x82 >> 1),  0xe5);
	if (ctx->rx_test == NULL) goto exit;
	
	ctx->tx_main = adv_dummy_client(ctx->io, ctx->tx_main_map, (0xb8 >> 1), 0xf9);//adv7627
	if (ctx->tx_main == NULL) goto exit;
	
	ctx->tx_edid = adv_dummy_client(ctx->io, ctx->tx_edid_map, (0x84 >> 1), 0xfa);//adv7627
	if (ctx->tx_edid == NULL) goto exit;
	
	ctx->tx_packet = adv_dummy_client(ctx->io, ctx->tx_packet_map, (0x70 >> 1), 0xfc);//adv7627 (UDP)
	if (ctx->tx_packet == NULL) goto exit;
	

	ctx->tx_cec = adv_dummy_client(ctx->io, ctx->tx_cec_map, (0x8c >> 1), 0xfd);
	if (ctx->tx_cec == NULL) goto exit;
	
	ctx->tx_test = adv_dummy_client(ctx->io, ctx->tx_test_map, (0xc0 >> 1), 0xfb);//adv7627
	if (ctx->tx_test == NULL) goto exit;
	
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
	
	if (ctx->is_initial_interrupt)
		if (interrupt_init(ctx) != 0) goto exit;
	
	adv762x_check_tx_monitor_present_status(ctx);
	
	return 0;
exit:	
	printk(KERN_INFO"[zynq_adv762x] Call adv762x_init() failed !!\n");
	return  -1;
}

static int adv762x_rls(struct adv762x_ctx  *ctx) {
	
	if (!ctx) return -1;
	
	interrupt_rls(ctx);
	
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
	
	if (ctx->rx_main) i2c_unregister_device(ctx->rx_main);
	if (ctx->edid_config) i2c_unregister_device(ctx->edid_config);
	if (ctx->rx_repeater) i2c_unregister_device(ctx->rx_repeater);
	if (ctx->rx_information) i2c_unregister_device(ctx->rx_information);
	if (ctx->rx_edid) i2c_unregister_device(ctx->rx_edid);
	if (ctx->rx_test) i2c_unregister_device(ctx->rx_test);
	if (ctx->cp_lite) i2c_unregister_device(ctx->cp_lite);
	if (ctx->dpll)  i2c_unregister_device(ctx->dpll);
	if (ctx->osd) i2c_unregister_device(ctx->osd);
	if (ctx->tx_main) i2c_unregister_device(ctx->tx_main);
	if (ctx->tx_packet) i2c_unregister_device(ctx->tx_packet);
	if (ctx->tx_cec) i2c_unregister_device(ctx->tx_cec);
	if (ctx->tx_edid) i2c_unregister_device(ctx->tx_edid);
	if (ctx->tx_test) i2c_unregister_device(ctx->tx_test);
	mutex_destroy(&ctx->lock);
	
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
static int adv762x_edid(struct adv762x_ctx * ctx, RX_EDID_IMAGE edidIm){
	
	unsigned short SpaOffset = 0;
	unsigned  char aSpaValue[8];
	unsigned int one_segment_edid_size = EDID_SEGMENT_SIZE;
	unsigned int edid_size = 0;
	unsigned int i = 0;
	unsigned int segment_num = 0;
	unsigned char *edid_buf = NULL;
	
	if (!ctx) return -1;
	
	edid_buf = &(ctx->edid.edid_buf[0]);
	segment_num =  ctx->edid_segment_num;
	edid_size = one_segment_edid_size * ctx->edid_segment_num;
	
	adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x00);
	
	if ((edid_size == 0) || (edid_size > EDID_MEM_LEN) || (!edid_buf)  ) return -1;
	
	SpaOffset= get_edid_spa_location(edid_buf);
	if ((SpaOffset > 0x00)  && (SpaOffset < 0xff)) {
		aSpaValue[0] = edid_buf[SpaOffset]+0x10;
		aSpaValue[1] = edid_buf[SpaOffset+1];
		aSpaValue[2] = edid_buf[SpaOffset]+0x20;
		aSpaValue[3] = edid_buf[SpaOffset+1];
		aSpaValue[4] = edid_buf[SpaOffset]+0x30;
		aSpaValue[5] = edid_buf[SpaOffset+1];
		aSpaValue[6] = edid_buf[SpaOffset]+0x40;
		aSpaValue[7] = edid_buf[SpaOffset+1];
	}
	
	adv_smbus_write_byte_data(ctx->edid_config, 0x72, 0x40);//default=0x00, External SPI EEPROM interface outputs tri-stated 
	//printk(KERN_INFO"[adv762x](%d) segment_num  = %u, SpaOffset = 0x%02x\n", __LINE__, segment_num, SpaOffset);
	
	for (i  = 0 ; i < segment_num; i++) {
		
		edid_buf = &(ctx->edid.edid_buf[i * one_segment_edid_size]);
		SpaOffset= get_edid_spa_location(edid_buf);
		//printk(KERN_INFO"[adv762x](%d) segment index  = %u, SpaOffset = 0x%02x\n", __LINE__, i, SpaOffset);
		if ((SpaOffset > 0x00)  && (SpaOffset < 0xfe)) {
			aSpaValue[0] = edid_buf[SpaOffset]+0x10;
			aSpaValue[1] = edid_buf[SpaOffset+1];
			aSpaValue[2] = edid_buf[SpaOffset]+0x20;
			aSpaValue[3] = edid_buf[SpaOffset+1];
			aSpaValue[4] = edid_buf[SpaOffset]+0x30;
			aSpaValue[5] = edid_buf[SpaOffset+1];
			aSpaValue[6] = edid_buf[SpaOffset]+0x40;
			aSpaValue[7] = edid_buf[SpaOffset+1];
		}
		
		adv_smbus_write_byte_field8 (ctx->edid_config, 0x7a, 0x3, 0x0, 0x0);//access primary HDMI EDID image
		adv_smbus_write_byte_field8 (ctx->edid_config, 0x70, 0xF0, 4, 0x4); //specify the PRIMARY_EDID_SIZE as 512 bytes
		if (i == 0) {
					adv_smbus_write_byte_field8 (ctx->edid_config, 0x7a, 0x70, 4, 0x0); //First 256 bytes of EDID data area
		} else if (i == 1) {
					adv_smbus_write_byte_field8 (ctx->edid_config, 0x7a, 0x70, 4, 0x1); //Second 256 bytes of EDID data area
		}
		
		mutex_lock(&ctx->lock);
		adv_smbus_write_i2c_block_data(ctx->rx_edid, 0, one_segment_edid_size, edid_buf); //write edid
		mutex_unlock(&ctx->lock);
		
		if ((SpaOffset > 0x00)  && (SpaOffset < 0xfe))  {
			adv_smbus_write_byte_field8 (ctx->edid_config,  0x71, 0xFF, 0,  (SpaOffset & 0xff) ); //SPA_LOCATION_LSB
			adv_smbus_write_byte_field8 (ctx->edid_config, 0x70, 0x7, 0,    (SpaOffset & 0xff00)>>8);//SPA_LOCATION_MSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x52,  aSpaValue[0]);// MSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x53,  aSpaValue[1]); // LSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x54,  aSpaValue[2]);// MSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x55,  aSpaValue[3]); // LSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x56,  aSpaValue[4]);// MSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x57,  aSpaValue[5]); // LSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x58,  aSpaValue[6]);// MSB
			adv_smbus_write_byte_data(ctx->edid_config, 0x59,  aSpaValue[7]); // LSB
		}
		//re-calculation of all checksums for the internal E-EDID for all ports.
     	adv_smbus_write_byte_field8 (ctx->edid_config, 0x77, 0x4, 0x2,  1);
	}
	if (ctx->hdmi_src_id == TK1HDMI) { //RXC
		if(adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x04) != 0) goto exit;  //default=0x0F, Enable the EDID of RXC
	} else if (ctx->hdmi_src_id == CON18HDMI) { //RXE
		if(adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x10) != 0) goto exit;  // default=0x0F, Enable the EDID of RXE
	} else {//use RXC as  the default port
		if(adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x10) != 0) goto exit;  // default=0x0F, Enable the EDID of RXE
	}
	
	if (edidIm == RX_EDIDIM_512_A)
			adv_smbus_write_byte_data(ctx->edid_config, 0x7d, 0x00);
	else if (edidIm == RX_EDIDIM_256_B)
			adv_smbus_write_byte_data(ctx->edid_config, 0x7d, 0x1f);
	
	adv_smbus_write_byte_data(ctx->edid_config, 0x40, 0x81) ;
	adv_smbus_write_byte_data(ctx->edid_config, 0x7c, 0x80) ;
exit:	 
     return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
static inline void adv762x_delay(unsigned int msec) {
	
	//unsigned long timeout = jiffies + HZ * sec; 
	//while (time_before(jiffies, timeout)) cpu_relax(  );
	 msleep_interruptible(msec);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 1
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
static int adv762x_read_edid_form_bin_file(const char *bin_file_name,struct adv762x_ctx  *ctx){
	
	struct file *file_id = NULL;
	mm_segment_t fs_old;
	int err = 0;
	ssize_t read_size = 0;
	loff_t pos_t = (loff_t)0;
	unsigned char *edid_buf = NULL;
	unsigned char *original_edid_buf = NULL;
	if (!ctx) return -1;
	
	if (!bin_file_name) {
		printk(KERN_INFO"[zynq_adv762x] bin_file_name is NULL!!\n");
       	 return -1;
	}
	
	edid_buf = &(ctx->edid.edid_buf[0]);
	original_edid_buf = &(ctx->original_edid.edid_buf[0]);
	
	if (!edid_buf || !original_edid_buf ) return -1;
	
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
	read_size = err;
	
	memcpy(original_edid_buf , edid_buf, read_size);
	
	mutex_lock(&ctx->lock);
	ctx->edid_segment_num = read_size / EDID_SEGMENT_SIZE;
	mutex_unlock(&ctx->lock);
	
	printk(KERN_INFO"[zynq_adv762x] The size read from %s is %u (segment num = %u)!! \n", bin_file_name, read_size, ctx->edid_segment_num);
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
					  unsigned int length, const u8 *values) {
	s32 ret = 0;	
	s32 i = 0;
	
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
	int i = 0;
	u8 val = 0;

	buf = &(g_ctx.edid.edid_buf[0]);

	for (i = 0; i < EDID_MEM_LEN; i++) {
		if (i % 16 == 0)
			seq_printf(s, "edid[%03x] =", i);

		seq_printf(s, " %02x", buf[i]);

		if (i % 16 == 15)
			seq_printf(s, "\n");
	}

	hdmi_query_cable_detect_status(&g_ctx, &val); 
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
	
	if (read_size > EDID_MEM_LEN) {
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
		
	v = (u8) adv_smbus_read_byte_data(ctx->io, 0x52);
	
	v_rxa = (v & 0x04) >> 2;
	v_rxb = (v & 0x08) >> 3;
	v_rxc = (v & 0x10) >> 4;
	v_rxd = (v & 0x20) >> 5;
	v_rxe = (v & 0x40) >> 6;
	
	//printk(KERN_INFO"[zynq_adv762x] HDMI Cable detect staus (0x%02x) : (RXA, RXB, RXC, RXD, RXE) = (0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x)\n", v , v_rxa, v_rxb, v_rxc, v_rxd, v_rxe);
		
	*value = v;
	
	return 0;
}
static int hdmi_hot_plug_assert (struct adv762x_ctx  *ctx, ADV762X_HPA ctrl) {
	
	if (!ctx) return -1;
	
	switch (ctrl) {
		case EDIDACTIVE:
			adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x00); //Disable the EDIDs of all ports
			/*
		 	* Hot Plug Config:
		 	* (1)disable manual control for the Hot Plug Assert output pins, 
		 	* (2)The HPA of a HDMI port is asserted high immediately after the internal EDID has been activated for that port. The HPA of a specific HDMI port is de-asserted low immediately after the internal E-EDID is de-activated for that port.
		 	* (3)1010:1s (The delay is a between +5 V detection and hot plug assertion on the HPA output pins.)
			*/
			adv_smbus_write_byte_data(ctx->io, 0xb1, 0x0a);
			break;
		case CABLEDECT:
			/*
		 	* Hot Plug Config:
		 	* (1)disable manual control for the Hot Plug Assert output pins, 
		 	* (2) The HPA of a HDMI port is asserted high following a programmable delay after the part detects an HDMI cable plug on that port. The HPA of an HDMI port is immediately de-asserted 
		 	* after the part detects a cable disconnect on that HDMI port.
		 	* (3)1010:1s (The delay is a between +5 V detection and hot plug assertion on the HPA output pins.)
			*/
			adv_smbus_write_byte_data(ctx->io, 0xb1, 0x1a);
			break;
		case EDIDACTIVE_AND_CABLEDECT:
			adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x00); //Disable the EDIDs of all ports
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
			adv_smbus_write_byte_data(ctx->io, 0xb1, 0x2a);
			break;
		case EDIDACTIVE_AND_CABLEDECT_ADN_MAN:
			adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x00); //Disable the EDIDs of all ports
			adv_smbus_write_byte_data(ctx->io, 0xb1, 0x3a);
			break;
		case MAN:
			adv_smbus_write_byte_data(ctx->io, 0xb1, 0x8a);
		default:
			return 0;
	}
	
	//	if(adv_smbus_write_byte_data(ctx->io, 0xb2, 0x1f) != 0) goto exit;//Set Hot Plug 
	
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

	adv_smbus_write_byte_data(ctx->dpll, 0xb5, val);

	
	return 0;
}
#if 0
static int hdmi_audio_output_i2s_map(struct adv762x_ctx  *ctx, unsigned int map_mode, unsigned int invert) {

		u8 val  = 0;
		u8 reg = 0x6d;
		
		/*	
			i2s_spdif_map_rot[1:0] (0x6d [5:4] )
			(1)00:	[I2S0/SPDIF0 on APx_OUT1] [I2S1/SPDIF1 on APx_OUT2] [I2S2/SPDIF2 on APx_OUT3] [I2S3/SPDIF3 on APx_OUT4]
			(2)01:	[I2S3/SPDIF3 on APx_OUT1] [I2S0/SPDIF0 on APx_OUT2] [I2S1/SPDIF1 on APx_OUT3] [I2S2/SPDIF2 on APx_OUT4]
			(3)10:	[I2S2/SPDIF2 on APx_OUT1] [I2S3/SPDIF3 on APx_OUT2] [I2S0/SPDIF0 on APx_OUT3] [I2S1/SPDIF1 on APx_OUT4]
			(4)11:  [I2S1/SPDIF1 on APx_OUT1] [I2S2/SPDIF2 on APx_OUT2] [I2S3/SPDIF3 on APx_OUT3] [I2S0/SPDIF0 on APx_OUT4]
		*/
		val = (u8) adv_smbus_read_byte_data(ctx->rx_main, reg);
		switch (map_mode) {
			case 0:
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0xcf) |0x00 ); break;
			case 1:
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0xcf) |0x10 ); break;
			case 2:
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0xcf) |0x20 ); break;
			case 3:
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0xcf) |0x30 ); break;
			default :
				break;
		}
		
		
		/*
 			I2S_SPDIF_MAP_INV[0] (0x6d [6])
			(1)0 - Do not invert arrangement of I2S/SPDIF channels in audio output port pins
			(2)1 - Invert arrangement of I2S/SPDIF channels in audio output port pins
		*/
		
		val = (u8) adv_smbus_read_byte_data(ctx->rx_main, reg);
		switch ( invert) {
			case 0:
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0xbf) | 0x00 ); break;
			case 1:
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0xbf) |0x40 ); break;
			default :
				break;
		}
		return 0;
}
#endif

#if 0
static int hdmi_audio_output_fmt (struct adv762x_ctx  *ctx, unsigned int mode, unsigned int bitwidth) {
	
		u8 val  = 0;
		u8 reg = 0x03;
		
		val = (u8) adv_smbus_read_byte_data(ctx->rx_main, reg);
		switch (mode) {
			case 0: //I2S
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0x9f) |0x00 ); break;
			case 1: //Right Justified
					adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0x9f) |0x20 ); break;
			case 2: //Left Justified
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0x9f) |0x40 ); break;
			case 3: //RAW SPDIF (IEC60958)
				adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0x9f) |0x60 ); break;
			default :
				break;
		}
		
		val = (u8) adv_smbus_read_byte_data(ctx->rx_main, reg);
		adv_smbus_write_byte_data(ctx->rx_main, reg, (val & 0xe0) | bitwidth); 

	return 0;
}
#endif

////////////////////////////////////////////////////////////////////////////////////////
static int transceiver_mode_init(struct adv762x_ctx  *ctx) {
	
	//unsigned char *edid_buf = NULL ;
	u8 cable_detec_status = 0;
	
	if (!ctx) goto exit;
	
	//edid_buf = &(ctx->edid.edid_buf[0]);
	//ADV7627 Transceiver Mode
	printk(KERN_INFO"[adv762x] Enter  transceiver_mode_init()\n");
	if(adv_smbus_write_byte_data(ctx->io, 0x00, 0xf0) != 0) goto exit;  //(1) power down RX1, (2)power down RX2, (3)Power down DPLL A, (4)Power up DPLL B, (5)Power up xtal amplifier, (6)Power down depending on individual power down bits, (7)Power down depending on individual power down bits 
	//	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x10) != 0) goto exit;  //Power up Tx 
		
	hdmi_audio_output_mclk (ctx, FS128);
	
	if(adv_smbus_write_byte_data(ctx->io, 0x08, 0x02) != 0) goto exit; //default=0x00, Tx Audio Insert Config - Audio Rx2-TxB
	
	if(adv_smbus_write_byte_data(ctx->io, 0x09, 0x51) != 0) goto exit; //default=0x55, ADI Required Write
	
	if(adv_smbus_write_byte_data(ctx->io, 0x0a, 0x02) != 0) goto exit; //default=0x00; Rx Audio Extract Config - Audio Rx2-AP_OUT2
	
	if (ctx->hdmi_src_id == TK1HDMI) { //RXC
		if(adv_smbus_write_byte_data(ctx->io, 0x01, 0x7a) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXC as the input port of RX2 
	} else if (ctx->hdmi_src_id == CON18HDMI) { //RXE
		if(adv_smbus_write_byte_data(ctx->io, 0x01, 0x7c) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXE as the input port of RX2 
	} else {//use RXC as  the default port
		if(adv_smbus_write_byte_data(ctx->io, 0x01, 0x7a) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXB as the input port of RX2 
	}
		
	if(adv_smbus_write_byte_data(ctx->io, 0x04, 0x26) != 0) goto exit; //default=0x26: Disable Rx2-TxB HDMI Mux mode/Enable CP-Lite-B/HDMI RX2 output video routed to CP-Lite-B/
		
	//hdmi_hot_plug_assert(ctx,  CABLEDECT);
	//hdmi_hot_plug_assert(ctx,  EDIDACTIVE);
	//hdmi_hot_plug_assert(ctx,  EDIDACTIVE_AND_CABLEDECT);
	hdmi_hot_plug_assert(ctx, EDIDACTIVE_AND_CABLEDECT_ADN_MAN);
	//hdmi_hot_plug_assert(ctx, MAN);
	
	if(adv_smbus_write_byte_data(ctx->io, 0xb3, 0xdf) != 0) goto exit; //default=0xDF, ADI Required Write
	
	if(adv_smbus_write_byte_data(ctx->io, 0x9e, 0x00) != 0) goto exit;  //default=0xFF, Turn off tristate on AP_OUT
	
	if(adv_smbus_write_byte_data(ctx->cp_lite, 0xc9, 0x2d) != 0) goto exit;//default=0x2C, CP_Lite Disable Auto Parameter Buffering - ADI Required Write 

	{
		int  i = 0;
		while (HdmiRecomInitTable[i]) {
			if (adv_smbus_write_byte_data(ctx->rx_main, HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]) != 0) goto exit;
			i+= 2;
		}
	}
	
	adv762x_delay(1000) ;//1000ms
	if(adv_smbus_write_byte_data(ctx->io, 0x00, 0xa0) != 0) goto exit; // default=0xF3, Power Up Rx2, DPLL B, xtal 
	if(adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x00) != 0) goto exit; //Disable the EDIDs of all ports
	
	adv762x_edid(ctx, RX_EDIDIM_512_A);

	if(adv_smbus_write_byte_data(ctx->tx_test, 0x24, 0x42) != 0) goto exit; //default=0x00, ADI Required Write - HDMI TxA Config 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x50) != 0) goto exit;//default=0x50, Power down  Tx 
	
	adv762x_delay(1000) ;//1000ms 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x01, 0x00) != 0) goto exit; //default=0x00, Set N Value(6144) used for Audio 48kHz
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x02, 0x18) != 0) goto exit; //default=0x00, Set N Value(6144) used for Audio 48kHz
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x03, 0x00) != 0) goto exit; //default=0x00, Set N Value(6144) used for Audio 48kHz
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x13, 0xff) != 0) goto exit; // default=0x00, Set Tx Category Code 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x15, 0x20) != 0) goto exit; //default 0x00, CS I2S Fs = 48kHz, Video Input Format = RGB/YCbCr 444 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x16, 0x20) != 0) goto exit; //default=0x00, Output video format RGB 444, Input Video RGB
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x40, 0x80) != 0) goto exit; // default=0x00, Enable Global Control Packet 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x4c, 0x04) != 0) goto exit; //default=0x00, 8-bit Deep Colour Mode 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x55, 0x00) != 0) goto exit; //default=0x00, AVI Infoframe - Y1Y0 = YCbCr 444
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x56, 0x08 ) != 0) goto exit;  // default=0x00, AVI Infoframe - R[3:0] = Active Format Apsect Ratio - Same as aspect ratio 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x73, 0x01 ) != 0) goto exit;  // default 0x00, Audio IF CC = 001, 2 channels 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x96, 0x20 ) != 0) goto exit;  // default=0x00, Configure TxA Interrupts 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xAF, 0x16 ) != 0) goto exit;  // default=0x14, Set HDMI Mode Output 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xBA, 0x70 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD0, 0x44 ) != 0) goto exit;  // default=0x4C, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD1, 0x3C ) != 0) goto exit;  // default=0x38, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD3, 0x07 ) != 0) goto exit;  // default=0x06, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD6, 0x02 ) != 0) goto exit;  // default=0x06, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xDB, 0x0B ) != 0) goto exit;  // default=0x0A, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE0, 0x90 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE1, 0xFC ) != 0) goto exit;  // default=0x0C, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE3, 0xD0 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE8, 0xF0 ) != 0) goto exit;  // default=0x10, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF3, 0x01 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF5, 0xCC ) != 0) goto exit;  // default=0xCC, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF6, 0x08 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF7, 0xF0 ) != 0) goto exit;  // default=0xFF, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xDA, 0x40 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF5, 0xD4 ) != 0) goto exit;  // default=0xCC, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x81, 0x33 ) != 0) goto exit;  // default=0x88, TxA Charge Injection Ch0 = 3, TxA Charge Injection Ch1 = 3 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x82, 0x33 ) != 0) goto exit;  // default=0x88, TxA Charge Injection Ch2 = 3, TxA Charge Injection Clk = 3 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x83, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x84, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x85, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x86, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xEA, 0x1D ) != 0) goto exit;  // default=0x84, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xED, 0x18 ) != 0) goto exit;  // default=0x80, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xEE, 0x18 ) != 0) goto exit;  // default=0x80, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xEF, 0x19 ) != 0) goto exit;  // default=0x82, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xFC, 0x55 ) != 0) goto exit;  // default=0x05, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x30 ) != 0) goto exit;  // default=0x50, ADI Required Write - HDMI TxA Config 
	
	adv762x_delay(1000) ;//1000ms 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x10 ) != 0) goto exit;  //default=0x50, ADI Required Write - HDMI TxA Config 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xAF, 0x16 ) != 0) goto exit; 
	
	adv762x_delay(1000) ;//1000ms 
	
	hdmi_query_cable_detect_status(ctx, &cable_detec_status); 
	
	printk(KERN_INFO"[adv762x] Leave  transceiver_mode_init()\n");
	
	return 0;
exit:
	return -1;
}

static int mux_mode_init(struct adv762x_ctx  *ctx) {
	
	u8 cable_detec_status = 0;
	
	if (!ctx) goto exit;

	printk(KERN_INFO"[adv762x] Enter  mux_mode_init()\n");
	
	if(adv_smbus_write_byte_data(ctx->io, 0x00, 0xf0) != 0) goto exit;  //(1) power down RX1, (2)power down RX2, (3)Power down DPLL A, (4)Power up DPLL B, (5)Power up xtal amplifier, (6)Power down depending on individual power down bits, (7)Power down depending on individual power down bits 
	//	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x10) != 0) goto exit;  //Power up Tx
	if(adv_smbus_write_byte_data(ctx->io, 0x09, 0x51) != 0) goto exit; //default=0x55, ADI Required Write
	
	if (ctx->hdmi_src_id == TK1HDMI) { //RXC
		if(adv_smbus_write_byte_data(ctx->io, 0x01, 0x7a) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXC as the input port of RX2 
	} else if (ctx->hdmi_src_id == CON18HDMI) { //RXE
		if(adv_smbus_write_byte_data(ctx->io, 0x01, 0x7c) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXE as the input port of RX2 
	} else {//use RXC as  the default port
		if(adv_smbus_write_byte_data(ctx->io, 0x01, 0x7a) != 0) goto exit;  //HDMI Receiver enabled, (1)Disable RX1, (2)input port of RX1 as reserved, (3)Enable RX2, (3)RXB as the input port of RX2 
	}
		
	if(adv_smbus_write_byte_data(ctx->io, 0x04, 0xe6) != 0) goto exit; //default=0x26, Enable Mux Mode 
		
	//hdmi_hot_plug_assert(ctx,  CABLEDECT);
	//hdmi_hot_plug_assert(ctx,  EDIDACTIVE);
	//hdmi_hot_plug_assert(ctx,  EDIDACTIVE_AND_CABLEDECT);
	hdmi_hot_plug_assert(ctx,  EDIDACTIVE_AND_CABLEDECT_ADN_MAN);
	//hdmi_hot_plug_assert(ctx,  MAN);
	
	if(adv_smbus_write_byte_data(ctx->io, 0xb3, 0xdf) != 0) goto exit; //default=0xDF, ADI Required Write
	
	{
		int  i = 0;
		while (HdmiRecomInitTable[i]) {
			if (adv_smbus_write_byte_data(ctx->rx_main, HdmiRecomInitTable[i], HdmiRecomInitTable[i+1]) != 0) goto exit;
			i+= 2;
		}
	}
	
	if(adv_smbus_write_byte_data(ctx->io, 0x00, 0xb0) != 0) goto exit; //  default=0xF3, Power Up Rx, xtal
	
	if(adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x00) != 0) goto exit; //Disable the EDIDs of all ports
	
	adv762x_edid(ctx, RX_EDIDIM_512_A);

	if(adv_smbus_write_byte_data(ctx->tx_test, 0x24, 0x42) != 0) goto exit; //default=0x00, ADI Required Write - HDMI TxA Config 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x50) != 0) goto exit;//default=0x50, Power down  Tx 
	
	adv762x_delay(1000) ;//1000ms 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x01, 0x00) != 0) goto exit; //default=0x00, Set N Value(6144) used for Audio 48kHz
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x02, 0x18) != 0) goto exit; //default=0x00, Set N Value(6144) used for Audio 48kHz
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x03, 0x00) != 0) goto exit; //default=0x00, Set N Value(6144) used for Audio 48kHz
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x13, 0xff) != 0) goto exit; // default=0x00, Set Tx Category Code 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x15, 0x20) != 0) goto exit; //default 0x00, CS I2S Fs = 48kHz, Video Input Format = RGB/YCbCr 444 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x16, 0x20) != 0) goto exit; //default=0x00, Output video format RGB 444, Input Video RGB
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x40, 0x80) != 0) goto exit; // default=0x00, Enable Global Control Packet 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x4c, 0x04) != 0) goto exit; //default=0x00, 8-bit Deep Colour Mode 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x55, 0x00) != 0) goto exit; //default=0x00, AVI Infoframe - Y1Y0 = YCbCr 444
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x56, 0x08 ) != 0) goto exit;  // default=0x00, AVI Infoframe - R[3:0] = Active Format Apsect Ratio - Same as aspect ratio 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x73, 0x01 ) != 0) goto exit;  // default 0x00, Audio IF CC = 001, 2 channels 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x96, 0x20 ) != 0) goto exit;  // default=0x00, Configure TxA Interrupts 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xAF, 0x16 ) != 0) goto exit;  // default=0x14, Set HDMI Mode Output 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xBA, 0x70 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD0, 0x44 ) != 0) goto exit;  // default=0x4C, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD1, 0x3C ) != 0) goto exit;  // default=0x38, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD3, 0x07 ) != 0) goto exit;  // default=0x06, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xD6, 0x02 ) != 0) goto exit;  // default=0x06, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xDB, 0x0B ) != 0) goto exit;  // default=0x0A, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE0, 0x90 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE1, 0xFC ) != 0) goto exit;  // default=0x0C, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE3, 0xD0 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xE8, 0xF0 ) != 0) goto exit;  // default=0x10, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF3, 0x01 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF5, 0xCC ) != 0) goto exit;  // default=0xCC, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF6, 0x08 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF7, 0xF0 ) != 0) goto exit;  // default=0xFF, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xDA, 0x40 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xF5, 0xD4 ) != 0) goto exit;  // default=0xCC, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x81, 0x33 ) != 0) goto exit;  // default=0x88, TxA Charge Injection Ch0 = 3, TxA Charge Injection Ch1 = 3 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x82, 0x33 ) != 0) goto exit;  // default=0x88, TxA Charge Injection Ch2 = 3, TxA Charge Injection Clk = 3 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x83, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x84, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x85, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x86, 0x81 ) != 0) goto exit;  // default=0x00, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xEA, 0x1D ) != 0) goto exit;  // default=0x84, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xED, 0x18 ) != 0) goto exit;  // default=0x80, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xEE, 0x18 ) != 0) goto exit;  // default=0x80, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xEF, 0x19 ) != 0) goto exit;  // default=0x82, ADI Required Write - HDMI TxA Source Termination OFF 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xFC, 0x55 ) != 0) goto exit;  // default=0x05, ADI Required Write - HDMI TxA Config 
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x30 ) != 0) goto exit;  // default=0x50, ADI Required Write - HDMI TxA Config 
	
	adv762x_delay(1000) ;//1000ms 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x10 ) != 0) goto exit;  //default=0x50, ADI Required Write - HDMI TxA Config 
	
	if(adv_smbus_write_byte_data(ctx->tx_main, 0xAF, 0x16 ) != 0) goto exit; 
	
	adv762x_delay(1000) ;//1000ms 
	
	hdmi_query_cable_detect_status(ctx, &cable_detec_status);
	
	printk(KERN_INFO"[adv762x] Leave  mux_mode_init()\n");
	
	return 0;
exit:
	return -1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
static int transceiver_mode_rls(struct adv762x_ctx  *ctx) {
#if 1
	if (!ctx) return -1;
	
	//Disable the edids of all ports
	adv_smbus_write_byte_data(ctx->rx_repeater, 0x74, 0x00);
	//Disable RX1 and RX2
	adv_smbus_write_byte_data(ctx->io, 0x01, 0x77);
	//Power down TX
	adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x50);//default=0x50, 01010000		
	//Power down RX1, RX2,  DPLLA, DPLLB, CORE, master
	adv_smbus_write_byte_data(ctx->io, 0x00, 0xf3);
#endif
	adv762x_hw_reset(ctx);
	return 0;	
}
/////////////////////////////////////////////////////////////////////////////////////////////////
static int mux_mode_rls(struct adv762x_ctx  *ctx) {
#if 1
	if (!ctx) return -1;

	//Disable the edids of all ports
	adv_smbus_write_byte_data(ctx->edid_config, 0x74, 0x00);
	//Disable RX1 and RX2
	adv_smbus_write_byte_data(ctx->io, 0x01, 0x77);
	//Power down TX
	adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x50);//default=0x50, 01010000

	//Power down RX1, RX2,  DPLLA, DPLLB, CORE, master
	adv_smbus_write_byte_data(ctx->io, 0x00, 0xf3);
#endif
	adv762x_hw_reset(ctx);
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////


static int set_tx_interrupt(struct adv762x_ctx  *ctx) {
	u8  interrupt_mask = 0x0;
	
	if (!ctx) return -1;
	
	interrupt_mask = (u8)adv_smbus_read_byte_data(ctx->tx_main, 0x94);
	interrupt_mask &= ~(TX_INTS_ALL_BITS);
	if (ctx->hdmi_src_id == TK1HDMI)
		interrupt_mask |= TX_HPD_CHNG_INT_BIT | TX_EDID_READY_INT_BIT /* | TX_EDID_READY_INT_BIT | TX_MSEN_CHNG_INT_BIT *//*  | TX_MSEN_CHNG_INT_BIT|TX_EDID_READY_INT_BIT | TX_HDCP_AUTH_INT_BIT*/;
	else 
		interrupt_mask |= TX_HPD_CHNG_INT_BIT  | TX_EDID_READY_INT_BIT /*  | TX_MSEN_CHNG_INT_BIT|TX_EDID_READY_INT_BIT | TX_HDCP_AUTH_INT_BIT*/;

	adv_smbus_write_byte_data(ctx->tx_main, 0x94, interrupt_mask);
	adv_smbus_write_byte_data(ctx->tx_main, 0x96, interrupt_mask);
	
	return 0;
}

static int init_tx_rx(struct adv762x_ctx  *ctx) {
	
	if (!ctx) return -1;
	
	switch (ctx->mode) {
			case MUXMODE:
			//	printk(KERN_INFO"[zynq_adv762x]xxx Working under the mux mode.\n");
			if (mux_mode_init(ctx) != 0) goto exit;
			break;
			case TRANSMODE:
			//printk(KERN_INFO"[zynq_adv762x]xxx Working under the transceiver mode.\n");
			if (transceiver_mode_init(ctx) != 0) goto exit;
				break;
			default:
			//	printk(KERN_INFO"[zynq_adv762x]xxx Working under unkown mode.\n");
			break;
	}
	if (ctx->is_initial_interrupt)
		if (interrupt_init(ctx) != 0) goto exit;
	
	return 0;
exit:
	return -1;
}

//Fixed header pattern: 00 FF FF FF FF FF FF 00 (0~7)
static unsigned char edid_fixed_pattern[] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

static int is_invalid_edid(struct adv762x_ctx  *ctx) {
	int is_invalid = 0;
	unsigned int pattern_size = 8;
	unsigned int i = 0;
	
	if (!ctx) return 0;
	
	for (i = 0 ; i <  pattern_size; i++ ) {
		if (edid_fixed_pattern[i] != ctx->edid.edid_buf[i]) {
			is_invalid = 1;
			break;
		}
	}
	
	return is_invalid;
}

static int process_tx_intrrupt(struct adv762x_ctx  *ctx) {
		
	u8 val = 0;
	u8  interrupt =0 ;
	u8 bHpd = 0;
	u8 bMonSen = 0 ;
	u8 bEdidReady =  0 ;
	u8 bVsync  =  0;
	u8 HdcpAuth = 0;
	u8 tx_state = 0;
	u8 hpd_state = 0;
	u8 monsen_state = 0;
	u8 i = 0, j = 0;	
	unsigned int k = 0;
	u8 *edid_ptr = NULL;
	u8 *orignal_edid_ptr = NULL;
	unsigned int bHasMoreEdidSegments = 0;
	if (!ctx) return -1;
	
	interrupt = (u8)adv_smbus_read_byte_data(ctx->tx_main, 0x96);
	bHpd = (u8) (interrupt & TX_HPD_CHNG_INT_BIT);
	bMonSen =  (u8) (interrupt & TX_MSEN_CHNG_INT_BIT);
	bEdidReady =  (u8) (interrupt &  TX_EDID_READY_INT_BIT);
	bVsync  =    (u8) (interrupt &  TX_VSYNC_EDGE_INT_BIT);
	HdcpAuth = 	(u8) (interrupt &  TX_HDCP_AUTH_INT_BIT);

	tx_state = (u8)adv_smbus_read_byte_data(ctx->tx_main, 0x42);
	hpd_state = (tx_state & 0x40) >> 0x6;
	if (hpd_state == 0) {
		printk(KERN_INFO"[zynq_adv762x] (%d) TX HPD state is 0x%02x\n", __LINE__, hpd_state);
		adv_smbus_write_byte_data(ctx->io, 0xb2, 0x00);
		adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x50);//Power down TX
		//adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x10);//Power up TX
	}
	if (bHpd) {
			adv_smbus_write_byte_data(ctx->tx_main, 0x96, TX_HPD_CHNG_INT_BIT);
			tx_state = (u8)adv_smbus_read_byte_data(ctx->tx_main, 0x42);
			hpd_state = (tx_state & 0x40) >> 0x6;
			//EdidSegCount = 0;
			mutex_lock(&ctx->lock);
			ctx->edid_segment_index = 0;
			ctx->edid_segment_num = 1;
			mutex_unlock(&ctx->lock);
			
			if (hpd_state) {
				adv_smbus_write_byte_data(ctx->tx_main, 0x41, 0x10 );//Powe up tx
					adv_smbus_write_byte_data(ctx->io, 0xb2, 0x1f);
				adv762x_delay(80) ;//wait 80ms for EDID to be ready			
			}else {
				bEdidReady =0;
				goto re_init;
			}
			printk(KERN_INFO"[zynq_adv762x] (%d) jeff TX HPD state is 0x%02x\n", __LINE__, hpd_state);
	}
	
	if (bMonSen) {
			adv_smbus_write_byte_data(ctx->tx_main, 0x96, TX_MSEN_CHNG_INT_BIT);
			tx_state = (u8)adv_smbus_read_byte_data(ctx->tx_main, 0x42);
			monsen_state = (tx_state & 0x20) >> 0x5; 
			printk(KERN_INFO"[zynq_adv762x] (%d)jeff  Monsen_state  = 0x%02x\n", __LINE__,  monsen_state);
			if (monsen_state == 1) {
				//NOTE: When monsen_state is 0x01, should fire the hpd interrupt.
				adv762x_check_tx_monitor_present_status(ctx); 
			}
	}
	
	 if (bEdidReady) {	 	
			i =  (u8)adv_smbus_read_byte_data(ctx->tx_main, 0xc4);
			
			 if ((i != ctx->edid_segment_index ) || (i > (TX_SUPPORTED_EDID_SEGMENTS - 1))){
				// printk(KERN_INFO"[zynq_adv762](%d)i  = 0x%02x, ctx->edid_segment_index  = 0x%02x \n",  __LINE__,  i, ctx->edid_segment_index );
				 adv_smbus_write_byte_data(ctx->tx_main, 0x96, TX_EDID_READY_INT_BIT);
			} else {
				//printk(KERN_INFO"[zynq_adv762](%d)i  = 0x%02x, ctx->edid_segment_index  = 0x%02x \n",  __LINE__,  i, ctx->edid_segment_index );
				
				adv_smbus_write_byte_data(ctx->tx_main, 0xc4, ctx->edid_segment_index );
				edid_ptr =  ctx->edid.edid_buf + ( ((unsigned int)i) << 8);
				//adv762x_delay(100) ;
				mutex_lock(&ctx->lock);
				if (ctx->adv762x_use_fixed_edid == 0) {
						for (k = 0; k < 256; k++) {
							val = (u8)adv_smbus_read_byte_data(ctx->tx_edid, k);
							* (edid_ptr + k)  = val;
						}
				}
				if (is_invalid_edid(ctx)) {
					printk(KERN_INFO"[zynq_adv762x](%d)eeexxx ee got the invalid EDID !!\n",  __LINE__);
					orignal_edid_ptr =  ctx->original_edid.edid_buf + ( ((unsigned int)i) << 8);
					if (ctx->adv762x_use_fixed_edid == 0) {
						for (k = 0; k < 256; k++) {
							* (edid_ptr + k)  = *(orignal_edid_ptr + k);
						}
					}
				}
				ctx->edid_segment_index++;
				mutex_unlock(&ctx->lock);
				adv_smbus_write_byte_data(ctx->tx_main, 0xc9, 0x1a);
				 j = edid_ptr[0x7E] >> 1;     /* j = N of additional segments */
            	if ((j > i) && (ctx->edid_segment_index  < TX_SUPPORTED_EDID_SEGMENTS)) {
                	adv_smbus_write_byte_data(ctx->tx_main, 0xc4, ctx->edid_segment_index ); /* mbw must toggle 10 times */
					printk(KERN_INFO"[zynq_adv762x](%d)Has more segments...\n",  __LINE__);
					bHasMoreEdidSegments = 1;
					mutex_lock(&ctx->lock);
					ctx->edid_segment_num = 2;
					mutex_unlock(&ctx->lock);
            	}
				adv_smbus_write_byte_data(ctx->tx_main, 0x96, TX_EDID_READY_INT_BIT); 
			}
#if 1
			for (k = 0; k < EDID_SEGMENT_SIZE; k++) {
				if (k < 16) 
					printk(KERN_INFO"[zynq_adv762x ]jeff123 edid[%d] = 0x%02x\n",  k,   ctx->edid.edid_buf[k]);
				else if (k == 0xe0)
					printk(KERN_INFO"[zynq_adv762x ]jeff123 edid[%d] = 0x%02x\n",  k,   ctx->edid.edid_buf[k]);
				else if (k == 0xff)
					printk(KERN_INFO"[zynq_adv762x ]jeff123 edid[%d] = 0x%02x\n",  k,   ctx->edid.edid_buf[k]);
			}
#endif
			if (bHasMoreEdidSegments == 0){
				//adv_smbus_write_byte_data(ctx->io, 0xb2, 0x1f);
				goto re_init;
			}
	}
	
no_re_init:

	return 0;
	
re_init:

	return 2;
	
}

static void adv762x_interrupt_service(struct work_struct *work)
{
	struct adv762x_ctx  *ctx = container_of(work,struct adv762x_ctx, interrupt_service);
    u8 val = 0;
    u8 cable_det_e_st = 0;
    u8 cable_det_e_raw = 0; 
    u8 int_txa_st = 0;
    u8 int_txb_st = 0;
	
	if (!ctx) return;
	
	atomic_set(&g_interrupt_sevice_done, -1);
   
    //  HDMI input plug/unplug process
    val = (u8)adv_smbus_read_byte_data(ctx->io, 0x53);
    cable_det_e_st = (val & 0x40) >> 6;
 
    val = (u8)adv_smbus_read_byte_data(ctx->io, 0x49);
    int_txa_st = (val & 0x80) >> 7;
    int_txb_st = (val & 0x40) >> 6;

    //printk(KERN_CRIT "[zynq_adv762x] %s()%d. e_st=%d, txa_st=%d, txb_st=%d\n", 
    //       __func__, __LINE__, cable_det_e_st, int_txa_st, int_txb_st);
   
    if (cable_det_e_st == 1) 
    {
        // clear RxE interrupt
        val  = (u8)adv_smbus_read_byte_data(ctx->io, 0x54);
        val = (val & 0xbf) | 0x40;
        adv_smbus_write_byte_data(ctx->io, 0x54, val);

        // RxE interrupt action
        if (ctx->hdmi_src_id == CON18HDMI)
        {
            val = (u8)adv_smbus_read_byte_data(ctx->io, 0x52);
            cable_det_e_raw = (val & 0x40) >> 6;
            //printk(KERN_CRIT"[zynq_adv762x] %s()%d. cable_det_e_raw=%d\n", __func__, __LINE__, cable_det_e_raw);
            if (cable_det_e_raw == 0) 
            {
                // No cable detected on HDMI RxE
           	    atomic_set(&g_interrupt_sevice_done, 0);
                mux_mode_init(ctx);
            } 
            else 
            {
                // Cable detected on HDMI RxE (High level on RXE_5V)
           	    atomic_set(&g_interrupt_sevice_done, 0);
                init_tx_rx(ctx); 
            }
        }
    }
    else if ((int_txa_st == 1) || (int_txb_st == 1))
    {
        if (int_txa_st == 1)
        {
            // clear TxA interrupt
            val = (u8)adv_smbus_read_byte_data(ctx->io, 0x4a);
            val = (val & 0x7f) | 0x80;
            adv_smbus_write_byte_data(ctx->io, 0x4a, val);
        }
        
        if (int_txb_st == 1)
        {
            // clear TxB interrupt
            val = (u8)adv_smbus_read_byte_data(ctx->io, 0x4a);
            val = (val & 0xbf) | 0x40;
            adv_smbus_write_byte_data(ctx->io, 0x4a, val);
        }

        val = (u8)adv_smbus_read_byte_data(ctx->io, 0x52);
        cable_det_e_raw = (val & 0x40) >> 6;
        //printk(KERN_CRIT "[zynq_adv762x] %s()%d. hdmi_src_id=%d, cable_det_e_raw=%d\n", 
        //       __func__, __LINE__, ctx->hdmi_src_id, cable_det_e_raw);
        if ((ctx->hdmi_src_id == CON18HDMI) && (cable_det_e_raw == 0))
        {
            // HDMI input is configured as BYOC(CON18) and this cable does exist.
            // Do nothing
        }
        else
        {
            if (process_tx_intrrupt(ctx) == 2)
            {
	            atomic_set(&g_interrupt_sevice_done, 0);
                init_tx_rx(ctx); 
            }
        }
    }

    if (atomic_read(&g_interrupt_sevice_done) == -1)
	    atomic_set(&g_interrupt_sevice_done, 0);

    //printk(KERN_CRIT "[zynq_adv762x] %s()%d. DONE\n", __func__, __LINE__);
    return;
}

static irqreturn_t adv762x_irq_handler(int irq, void *devid)
{
    struct adv762x_ctx *ctx = (struct adv762x_ctx  *)devid;

    if (ctx) {
        if (atomic_read(&g_interrupt_sevice_done) == -1) {
            queue_work(ctx->work_queue, &ctx->interrupt_service);
            atomic_set(&g_interrupt_sevice_first_run, 0);
        }
        if (atomic_read(&g_interrupt_sevice_done) == 0)
        {
            queue_work(ctx->work_queue, &ctx->interrupt_service);
        }
    }
    return IRQ_HANDLED;
}


static int interrupt_init(struct adv762x_ctx  *ctx) {
	
	int irq = -1;
	int ret = -1;
	
	if (!ctx) return -1;
	/*
	 This bit is the INT2 interrupt mask for the TxB interrupt. When set, the TxB interrupt will trigger the INT2 interrupt and
	 INT_TXB_ST will indicate the interrupt status.
	0 - Disables TxB interrupt for INT2
	1 - Enables TxB interrupt for INT2 
	 */
	adv_smbus_write_byte_data(ctx->io, 0x41, 0xf6);
	adv_smbus_write_byte_data(ctx->io, 0x4b, 0x40);
	
	adv_smbus_write_byte_data(ctx->io, 0x40, 0xc2);
	adv_smbus_write_byte_data(ctx->io, 0x4c, 0x40);
	
	set_tx_interrupt(ctx);
	//adv_smbus_write_byte_data(ctx->tx_main, 0x94, 0x80);
	//adv_smbus_write_byte_data(ctx->tx_main, 0x96, 0x80);

    // CABLE_DET_E_MB1:
    //   This bit is the INT1 interrupt mask for the HDMI RxE +5V cable detection interrupt. 
    //   When set, the HDMI RxE +5V cable detection interrupt will trigger the INT1 interrupt 
    //   and CABLE_DET_E_ST will indicate the interrupt status.
	adv_smbus_write_byte_data(ctx->io, 0x56, 0x40);

	if (ctx->irq == -1) {
		ctx->work_queue = create_singlethread_workqueue(ADV762X_DRIVER_NAME);
		if (!ctx->work_queue) {
			printk(KERN_INFO"[zynq_adv762x] create_singlethread_workqueue() for %u is failed!! \n",ADV762x_INTX_PIN );
			goto exit;
		}
		INIT_WORK(&ctx->interrupt_service, adv762x_interrupt_service);
		irq = gpio_to_irq(ADV762x_INTX_PIN);
		if (irq <= 0) {
			printk(KERN_INFO"[zynq_adv762x] Could not set the %u  pin as interrupt pin!! \n",ADV762x_INTX_PIN );
			goto exit;
		}
		//adv762x_delay(100);
		
		ret = request_irq(irq, adv762x_irq_handler, IRQF_SHARED | IRQF_TRIGGER_RISING, ADV762X_DRIVER_NAME, ctx);
		if (ret) {
			printk(KERN_INFO"[zynq_adv762x] request_irq() for %u is failed!! \n",ADV762x_INTX_PIN );
			goto exit;
		}
	
		printk(KERN_INFO"[zynq_adv762x] jeff Set GPIO %u as interrupt %d.\n", ADV762x_INTX_PIN, irq);
	
		ctx->irq = irq;
		

	}
	return 0;
	
exit:
	if (ctx->irq != -1){
		free_irq(ctx->irq, ctx);
		ctx->irq = -1;
	}
	if (ctx->work_queue) {
		cancel_work_sync(&ctx->interrupt_service);
		destroy_workqueue(ctx->work_queue);
		ctx->work_queue = NULL;
	}
	
	return -1;
}

static int interrupt_rls(struct adv762x_ctx  *ctx) {
	if (!ctx) return -1;
	
	adv_smbus_write_byte_data(ctx->io, 0x4b, 0x00);
	adv_smbus_write_byte_data(ctx->io, 0x41, 0x30);
	
	if (ctx->irq != -1){
		free_irq(ctx->irq, ctx);
		ctx->irq = -1;
	}
	if (ctx->work_queue) {
		cancel_work_sync(&ctx->interrupt_service);
		destroy_workqueue(ctx->work_queue);
		ctx->work_queue = NULL;
	}
	
	return 0;
}
////////////////////////////////////////////////////////////////////////

typedef struct {
	unsigned int bIsInitialed;
	unsigned int bIsChanged;
	struct mutex  lock;
} ctrl_status_t;

static ctrl_status_t  g_ctrl_status;
#if  1
static unsigned int ctrl_set_change_status(unsigned int val) {
	mutex_lock(&g_ctrl_status.lock);
	g_ctrl_status.bIsChanged = val;
	mutex_unlock(&g_ctrl_status.lock);
	return 0;
}
static unsigned int ctrl_get_change_status() {
	return  g_ctrl_status.bIsChanged;
}
#endif
static unsigned int ctrl_set_initial_status(unsigned int val) {
	mutex_lock(&g_ctrl_status.lock);
	g_ctrl_status. bIsInitialed = val;
	mutex_unlock(&g_ctrl_status.lock);
	return 0;
}
static unsigned int ctrl_get_initial_status() {
	return  g_ctrl_status. bIsInitialed;

}


static ssize_t b_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {

	const char *name =  attr->attr.name;
	unsigned int set_val = 0;

	if (name && buf)  {
		    if (strcmp(name, "HPD") == 0) { 
				 set_val = simple_strtoul(buf, NULL, 16);
				 printk(KERN_INFO "[zynq_adv762x] control sysfs  (name, buf, count, strlen(buf), val) = (%s, %s, %u, %u, %u)\n", name, buf, count,  strlen(buf), set_val);
				if (set_val == 1) adv762x_check_tx_monitor_present_status(&g_ctx);
			}
	}
	 return count;
};

static ssize_t b_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    const char *name =  attr->attr.name;
    int val = 0;
    if (name)  {
        if (strcmp(name, "HDMI_SOURCE") == 0) { 
            mutex_lock(&g_ctx.lock);
            val = (int)g_ctx.hdmi_src_id;
            mutex_unlock(&g_ctx.lock);
            return sprintf(buf, "%d\n", val);
        }
        else if (strcmp(name, "CABLE_DET") == 0) {
            struct adv762x_ctx *ctx = &g_ctx;
            u8 val = (u8) adv_smbus_read_byte_data(ctx->io, 0x52);
            return sprintf(buf, "0x%02x\n", val);
        }
        else if (strcmp(name, "SYSTEM_PD") == 0) {
            struct adv762x_ctx *ctx = &g_ctx;
            u8 val = (u8) adv_smbus_read_byte_data(ctx->tx_main, 0x41);
            return sprintf(buf, "0x%02x\n", val);
        }
    }
    return  0;
};

static struct kobj_attribute hdmi_source_attribute = __ATTR(HDMI_SOURCE,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute hpd_attribute = __ATTR(HPD,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute cable_det_attribute = __ATTR(CABLE_DET,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute system_pd_attribute = __ATTR(SYSTEM_PD,  S_IRUGO |  S_IWUGO, b_show, b_store);


static struct attribute *attrs[] = {
	    (struct attribute *)&hdmi_source_attribute,
	    (struct attribute *)&hpd_attribute,
	    (struct attribute *)&cable_det_attribute,
	    (struct attribute *)&system_pd_attribute,
	    NULL
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

static int create_control_sysfs(struct kobject *kobj) {
	   int retval  = -1;
	
	if (kobj != NULL) {
        retval = sysfs_create_group(kobj, &attr_group);
		mutex_init(&g_ctrl_status.lock);
		ctrl_set_change_status(0);
		ctrl_set_initial_status(0) ;
	}
	  return retval;
}

static int destroy_control_sysfs(struct kobject *kobj) {
    
	if (!kobj) return -1;

	mutex_destroy(&g_ctrl_status.lock);
    sysfs_remove_group(kobj, &attr_group);
	ctrl_set_change_status(0);
	ctrl_set_initial_status(0) ;
    return 0;
}
