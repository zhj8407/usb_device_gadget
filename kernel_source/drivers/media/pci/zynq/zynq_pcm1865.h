#ifndef _ZYNQ_PCM1865_H_
#define _ZYNQ_PCM1865_H_
#include <sound/control.h>

#define	PCM1865_PSEL_REG		0x00

#define PCM1865_DEV_CONF_PAGE 0x00
#define PCM1865_DSP_COEF_PAGE 0x01
#define PCM1865_LOW_POWER_CONF1_PAGE 0x03
#define PCM1865_LOW_POWER_CONF2_PAGE 0xfd

struct pcm1865_platform_data {
    unsigned int id;
    unsigned int mode; //0:slave mode, 1:master mode
};

int  pcm1865_write(struct i2c_client *i2c, unsigned int reg, unsigned int val, unsigned int *cur_page_ptr, unsigned int new_page);
int pcm1865_read(struct i2c_client *i2c, unsigned int reg, unsigned int *cur_page_ptr, unsigned int new_page, unsigned int cached);
void pcm1865_rmw_clr(struct i2c_client *i2c, unsigned int reg, unsigned int *cur_page_ptr, unsigned int new_page,  u32 val);
void pcm1865_rmw_set(struct i2c_client *i2c, unsigned int reg, unsigned int *cur_page_ptr, unsigned int new_page,  u32 val);

int pcm1865_resume(struct i2c_client *i2c);
int pcm1865_suspend(struct i2c_client *i2c);

unsigned int pcm1865_get_mixter_ctrl_num(void);


extern struct snd_kcontrol_new pcm1865_audio_mixter_ctrls[];

#endif	/*_ZYNQ_PCM1865_H_ */