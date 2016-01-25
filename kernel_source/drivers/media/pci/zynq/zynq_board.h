#ifndef ZYNQ_BOARD_H
#define ZYNQ_BOARD_H

#include <linux/i2c.h>
#include "zynq_types.h"

#define ADV7611_0_I2C_BUS 0x2
#define ADV7611_1_I2C_BUS 0x1
#define M10MO_I2C_BUS 0x1
#define ADV7511_0_I2C_BUS 0x2
#define ADV7511_1_I2C_BUS 0x1
#define TC358746A_I2C_BUS 0x1

#define SUBDEV_CH0_I2C_BUS   ADV7611_0_I2C_BUS // ADV7611
#define SUBDEV_CH1_I2C_BUS  ADV7611_1_I2C_BUS //ADV7611
#define SUBDEV_CH2_I2C_BUS  M10MO_I2C_BUS	//m10m0
#define SUBDEV_CH3_I2C_BUS  ADV7511_0_I2C_BUS //ADV7511
#define SUBDEV_CH4_I2C_BUS  ADV7511_1_I2C_BUS 	//ADV7511
#define SUBDEV_CH5_I2C_BUS  TC358746A_I2C_BUS 	//TC358746A

#define SUBDEV_CH0_I2C_ADDR (0x98 >> 1) //ADV7611
#define SUBDEV_CH1_I2C_ADDR (0x98 >> 1) //ADV7611
#define SUBDEV_CH2_I2C_ADDR (0x3e >> 1) //M10MO
#define SUBDEV_CH3_I2C_ADDR (0x7a >> 1) //ADV7511
#define SUBDEV_CH4_I2C_ADDR (0x72 >> 1) //ADV7511
#define SUBDEV_CH5_I2C_ADDR (0x1c >> 1) //TC358746A (0x07 << 1) or (0x0e << 1)

#define M10MO_ID  "FujistuM10MO"
#define TC358746A_ID "tc358746"

#define SUBDEV_CH0		"adv7611-0"
#define SUBDEV_CH1		"adv7611-1"
#define SUBDEV_CH2		M10MO_ID 
#define SUBDEV_CH3 	"adv7511-0"
#define SUBDEV_CH4		"adv7511-1"
#define SUBDEV_CH5 	TC358746A_ID

#define ADV7611_I2C_ID_NAME "adv761x"
#define M10MO_I2C_ID_NAME  "FujistuM10MO"
#define ADV7511_I2C_ID_NAME "adv7511"
#define TC358746A_I2C_ID_NAME "tc358746"

struct i2c_bus_adapter
{
	unsigned int bus_num;
	struct i2c_adapter *i2c_adap;
};
void zynq_setup_i2c_adapter(void) ;
struct i2c_adapter *zynq_get_i2c_adapter_by_bus_num(unsigned int  bus_num);
void  zynq_rls_i2c_adapter(void);

void zynq_setup_interrupt(void);
unsigned  zynq_get_irq(int channel_id);

extern struct vpif_subdev_info board_subdev_info[];
extern  unsigned int board_subdev_info_num;

extern vpif_vidoe_pipelie_entity_t  board_video_pipeline_entities[];
extern unsigned int board_video_pipeline_entity_num;

vpif_vidoe_pipelie_entity_t *board_find_video_pipeline_entity(vpif_vidoe_pipelie_entity_id_t id);
const char *to_video_pipelin_entity_name(vpif_vidoe_pipelie_entity_id_t  id);
const char *to_video_data_pin_name(vpif_video_data_pin_t pin);
#endif