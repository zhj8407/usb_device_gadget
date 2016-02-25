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

#include "zynq_audio.h"
#include "zynq_pcm1865.h"
#include "zynq_pcm1865_regs.h"

#define PCM1865_DRIVER_NAME "pcm1865"

#define MAX_CHANNEL_NUM 0x4

#define PCM1865_UNKWON_GAIN_VALUE 0xff

#define MAX_DEV_NUM  0x3

//#define VIRTUAL_PCM1865_DEVICE 1

typedef struct {

    cfg_reg page0[1024];
    unsigned int page0_len;

    cfg_reg page1[1024];
    unsigned int page1_len;

    cfg_reg page3[1024];
    unsigned int page3_len;

    cfg_reg page253[1024];
    unsigned int page253_len;

} regs_map_t;

static regs_map_t g_reg_maps[MAX_DEV_NUM];

static struct mutex g_mutex_locks[MAX_DEV_NUM];

static 	int  g_audio_mute[MAX_DEV_NUM][MAX_CHANNEL_NUM];
static  int  g_analog_gain[MAX_DEV_NUM][MAX_CHANNEL_NUM];
static  int  g_digital_gain[MAX_DEV_NUM][MAX_CHANNEL_NUM];
static unsigned int g_current_reg_page[MAX_DEV_NUM] = {0x00, 0x00, 0x00};
/////////////////////////////////////////////////////////////////////////////////////////
static char g_page0_regs_dump_str[MAX_DEV_NUM][PAGE_SIZE];
static char g_page1_regs_dump_str[MAX_DEV_NUM][PAGE_SIZE];
static char g_page3_regs_dump_str[MAX_DEV_NUM][PAGE_SIZE];
static char g_page253_regs_dump_str[MAX_DEV_NUM][PAGE_SIZE];

static int dump_registers(unsigned int codec_id, unsigned int page, char *regs_dump_str, int len)
{
    unsigned int i = 0;
    unsigned int size = 0;
    cfg_reg *cfg_reg_ptr = NULL;
    unsigned int offset = 0;
    char *str = regs_dump_str;

    if (page == PCM1865_DEV_CONF_PAGE ) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page0[0];
        size = g_reg_maps[codec_id].page0_len;
    } else if (page == PCM1865_DSP_COEF_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page1[0];
        size = g_reg_maps[codec_id].page1_len;
    } else if (page == PCM1865_LOW_POWER_CONF1_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page3[0];
        size = g_reg_maps[codec_id].page3_len;
    } else if (page == PCM1865_LOW_POWER_CONF2_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page253[0];
        size = g_reg_maps[codec_id].page253_len;
    }

    if ((cfg_reg_ptr == NULL) || (str == NULL) ) return -1;

    for (i = 0; i < size; i++) {
        offset = snprintf(str, len, "[0x%02x]:0x%02x\n", cfg_reg_ptr[i].offset, cfg_reg_ptr[i].value);
        str += offset;
    }
    return 0;
}

static ssize_t b_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    char *str = NULL;
    unsigned int page = 0;
    unsigned int codec_id = 0;

    if (strcmp(attr->attr.name, "0_0") == 0) {
        page = 0;
        codec_id = 0;
        str = g_page0_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "0_1") == 0) {
        page = 1;
        codec_id = 0;
        str = g_page1_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "0_3") == 0) {
        page = 3;
        codec_id = 0;
        str = g_page3_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "0_253") == 0) {
        page =253;
        codec_id = 0;
        str = g_page253_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "1_0") == 0) {
        page = 0;
        codec_id = 1;
        str = g_page0_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "1_1") == 0) {
        page = 1;
        codec_id = 1;
        str = g_page1_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "1_3") == 0) {
        page = 3;
        codec_id = 1;
        str = g_page3_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "1_253") == 0) {
        page =253;
        codec_id = 1;
        str = g_page253_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "2_0") == 0) {
        page = 0;
        codec_id = 2;
        str = g_page0_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "2_1") == 0) {
        page = 1;
        codec_id = 2;
        str = g_page1_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "2_3") == 0) {
        page = 3;
        codec_id = 2;
        str = g_page3_regs_dump_str[codec_id];
    } else if (strcmp(attr->attr.name, "2_253") == 0) {
        page =253;
        codec_id = 2;
        str = g_page253_regs_dump_str[codec_id];
    }
    if (str == NULL) {
        return  0;
    } else {
        dump_registers(codec_id, page, str, PAGE_SIZE);
        return sprintf(buf, "%s", str);
    }
}

static struct kobj_attribute codec0_page0_attribute = __ATTR(0_0,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec0_page1_attribute = __ATTR(0_1,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec0_page3_attribute = __ATTR(0_3,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec0_page253_attribute = __ATTR(0_253,  S_IRUGO, b_show, NULL);

static struct kobj_attribute codec1_page0_attribute = __ATTR(1_0,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec1_page1_attribute = __ATTR(1_1,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec1_page3_attribute = __ATTR(1_3,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec1_page253_attribute = __ATTR(1_253,  S_IRUGO, b_show, NULL);


static struct kobj_attribute codec2_page0_attribute = __ATTR(2_0,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec2_page1_attribute = __ATTR(2_1,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec2_page3_attribute = __ATTR(2_3,  S_IRUGO, b_show, NULL);
static struct kobj_attribute codec2_page253_attribute = __ATTR(2_253,  S_IRUGO, b_show, NULL);


static struct attribute *attrs[] = {
    (struct attribute *)&codec0_page0_attribute,
    (struct attribute *)&codec0_page1_attribute,
    (struct attribute *)&codec0_page3_attribute,
    (struct attribute *)&codec0_page253_attribute,
    (struct attribute *)&codec1_page0_attribute,
    (struct attribute *)&codec1_page1_attribute,
    (struct attribute *)&codec1_page3_attribute,
    (struct attribute *)&codec1_page253_attribute,
    (struct attribute *)&codec2_page0_attribute,
    (struct attribute *)&codec2_page1_attribute,
    (struct attribute *)&codec2_page3_attribute,
    (struct attribute *)&codec2_page253_attribute,
    NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

static int create_register_dump_sysfs(struct kobject *kobj,  unsigned int codec_id)
{
    int retval  = -1;

    printk(KERN_INFO"[pcm1865] Enter create_register_dump_sysfs()\n");

    memset(g_page0_regs_dump_str[codec_id], 0x0, PAGE_SIZE);
    memset(g_page1_regs_dump_str[codec_id], 0x0, PAGE_SIZE);
    memset(g_page3_regs_dump_str[codec_id], 0x0, PAGE_SIZE);
    memset(g_page253_regs_dump_str[codec_id], 0x0, PAGE_SIZE);

    if (kobj != NULL)
        retval = sysfs_create_group(kobj, &attr_group);

    printk(KERN_INFO"[pcm1865] Leave create_register_dump_sysfs()\n");

    return retval;
}

static int destroy_register_dump_sysfs(struct kobject *kobj)
{
    printk(KERN_INFO"[pcm1865] Enter destroy_register_dump_sysfs()\n");

    if (!kobj) return -1;

    sysfs_remove_group(kobj, &attr_group);

    printk(KERN_INFO"[pcm1865] Leave destroy_register_dump_sysfs()\n");
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////
/*I2C configuration related work*/

static int vpl_audio_analog_gain_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *info)
{
    // Actually  the analog gain could be -12 dB ~ +12 dB (1 dB steps), 20 dB, 32 dB (see p.11 of http://www.ti.com/lit/ds/slas831c/slas831c.pdf)
    info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    info->count = 3;
    info->value.integer.min = 0x00; //0x01 (+0.5 dB)~ 0x50 (+40.0 dB)
    info->value.integer.max = 0xff;//0xe8 (-12.0 dB) ~0xff(-0.5 dB)
    info->value.integer.step = 1;
    return 0;
}

static int vpl_audio_analog_gain_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *value)
{
    struct i2c_client *capture_client = NULL ;
    int device_num = value->value.integer.value[0] ;
    int channel_num = value->value.integer.value[1] ;
    int ori_volume = -1 ;
    unsigned int reg = 0x00;

    zynq_audio_card_info_t  *card_info = ( zynq_audio_card_info_t  *)snd_kcontrol_chip(kcontrol);

    if ((device_num < 0) || (device_num >=  card_info->data_len)) {
        printk(KERN_INFO "[pcm1865] Set device %d failed !! (0~%d)\n", device_num, (card_info->data_len-1)) ;
        return -1;
    }

    if ((channel_num < 0) || (channel_num >=  MAX_CHANNEL_NUM)) {
        printk(KERN_INFO "[pcm1865] Set channel %d failed !! (0~%d)\n", channel_num, (MAX_CHANNEL_NUM-1)) ;
        return -1;
    }

    capture_client = (struct i2c_client *)card_info->data[device_num];

    if (!capture_client) {
        printk(KERN_INFO "[pcm1865] I2C client obj is NULL!!\n") ;
        return  -1;
    }

    if (g_analog_gain[device_num][channel_num] != PCM1865_UNKWON_GAIN_VALUE) {
        ori_volume = g_analog_gain[device_num][channel_num];
    } else {

        if (channel_num == 0) reg = 0x01;
        else  if (channel_num == 1) reg = 0x02;
        else  if (channel_num == 2) reg = 0x03;
        else  if (channel_num == 3) reg = 0x04;
        if (reg != 0x00) {
            ori_volume = pcm1865_read(capture_client, reg, &g_current_reg_page[device_num], 0, 0);
            mutex_lock(&g_mutex_locks[device_num]);
            g_digital_gain[device_num][channel_num] = ori_volume;
            mutex_unlock(&g_mutex_locks[device_num]);
        }
    }
    value->value.integer.value[0] = ori_volume;
    return 0 ;
}

static int vpl_audio_analog_gain_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *value)
{
    struct i2c_client *capture_client = NULL ;
    int device_num = value->value.integer.value[0] ;
    int channel_num = value->value.integer.value[1] ;
    int new_volume = value->value.integer.value[2] ;
    unsigned int reg = 0x00;

    zynq_audio_card_info_t  *card_info = ( zynq_audio_card_info_t  *)snd_kcontrol_chip(kcontrol);

    if ((device_num < 0) || (device_num >=  card_info->data_len)) {
        printk(KERN_INFO "[pcm1865] Set device %d failed !! (0~%d)\n", device_num, (card_info->data_len-1)) ;
        return -1;
    }

    if ((channel_num < 0) || (channel_num >=  MAX_CHANNEL_NUM)) {
        printk(KERN_INFO "[pcm1865] Set channel %d failed !! (0~%d)\n", channel_num, (MAX_CHANNEL_NUM-1)) ;
        return -1;
    }

    capture_client = (struct i2c_client *)card_info->data[device_num];

    if (!capture_client) {
        printk(KERN_INFO "[pcm1865] I2C client obj is NULL!!\n") ;
        return  -1;
    }

    if (g_analog_gain[device_num][channel_num] != new_volume) {

        mutex_lock(&g_mutex_locks[device_num]);

        g_analog_gain[device_num][channel_num] = new_volume;

        if (channel_num == 0) reg =0x01;
        else  if (channel_num == 1) reg = 0x2;
        else  if (channel_num == 2)  reg =0x3;
        else  if (channel_num == 3)  reg = 0x4;

        if (reg != 0x0) pcm1865_write(capture_client, reg, new_volume, &g_current_reg_page[device_num], 0x0);

        mutex_unlock(&g_mutex_locks[device_num]);
    }
    return 0;
}

static int vpl_audio_digital_gain_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *info)
{
    //0x28 - 0x3F in 0.5 dB steps
    // Gain steps between 12dB and 20dB are all done in the digital domain. (for example, 18dB gain = 12dB analog + 6dB digital).(see p.22 of http://www.ti.com/lit/ds/slas831c/slas831c.pdf)
    info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    info->count = 3;
    info->value.integer.min = 0x28; // 0.0 dB
    info->value.integer.max =0x37; //7.5 dB
    info->value.integer.step = 1;
    return 0;
}

static int vpl_audio_digital_gain_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *value)
{
    struct i2c_client *capture_client = NULL ;
    int device_num = value->value.integer.value[0] ;
    int channel_num = value->value.integer.value[1] ;
    int ori_volume = -1 ;
    unsigned int reg  = 0x00;

    zynq_audio_card_info_t  *card_info = ( zynq_audio_card_info_t  *)snd_kcontrol_chip(kcontrol);

    if ((device_num < 0) || (device_num >=  card_info->data_len)) {
        printk(KERN_INFO "[pcm1865] Set device %d failed !! (0~%d)\n", device_num, (card_info->data_len-1)) ;
        return -1;
    }

    if ((channel_num < 0) || (channel_num >=  MAX_CHANNEL_NUM)) {
        printk(KERN_INFO "[pcm1865] Set channel %d failed !! (0~%d)\n", channel_num, (MAX_CHANNEL_NUM-1)) ;
        return -1;
    }

    capture_client = (struct i2c_client *)card_info->data[device_num];

    if (!capture_client) {
        printk(KERN_INFO "[pcm1865] I2C client obj is NULL!!\n") ;
        return  -1;
    }

    //printk(KERN_INFO "[pcm1865] Card info (info, len, i2c)(%p, %d, %p)\n", card_info, card_info->data_len, capture_client);

    if (g_digital_gain[device_num][channel_num] != PCM1865_UNKWON_GAIN_VALUE) {
        ori_volume = g_digital_gain[device_num][channel_num];
    } else {

        if (channel_num == 0) reg =0x0f ;
        else  if (channel_num == 1) reg = 0x16;
        else  if (channel_num == 2) reg = 0x17;
        else  if (channel_num == 3) reg = 0x18;

        if (reg != 0x00) {
            ori_volume = pcm1865_read(capture_client, reg,  &g_current_reg_page[device_num], 0, 0);
            mutex_lock(&g_mutex_locks[device_num]);
            g_digital_gain[device_num][channel_num] = ori_volume;
            mutex_unlock(&g_mutex_locks[device_num]);
        }

    }
    value->value.integer.value[0] = ori_volume;
    return 0 ;
}

static int vpl_audio_digital_gain_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *value)
{
    struct i2c_client *capture_client = NULL ;
    int device_num = value->value.integer.value[0] ;
    int channel_num = value->value.integer.value[1] ;
    int new_volume = value->value.integer.value[2] ;
    unsigned reg  = 0x00;

    zynq_audio_card_info_t  *card_info = ( zynq_audio_card_info_t  *)snd_kcontrol_chip(kcontrol);


    if ((device_num < 0) || (device_num >=  card_info->data_len)) {
        printk(KERN_INFO "[pcm1865] Set device %d failed !! (0~%d)\n", device_num, (card_info->data_len-1)) ;
        return -1;
    }

    if ((channel_num < 0) || (channel_num >=  MAX_CHANNEL_NUM)) {
        printk(KERN_INFO "[pcm1865] Set channel %d failed !! (0~%d)\n", channel_num, (MAX_CHANNEL_NUM-1)) ;
        return -1;
    }

    capture_client = (struct i2c_client *)card_info->data[device_num];

    if (!capture_client) {
        printk(KERN_INFO "[pcm1865] I2C client obj is NULL!!\n") ;
        return  -1;
    }

    if (g_digital_gain[device_num][channel_num] != new_volume) {
        mutex_lock(&g_mutex_locks[device_num]);
        g_digital_gain[device_num][channel_num] = new_volume;

        /*
        	Digital Gain setting when the device is used in the following two scenarios (4 channel device only, values from 0x28 to 0x37):
        	i. Analog PGA gain and digital PGA are set separately
        	ii. Digital Microphone Interface is used (4-channel device only, when Manual Gain Mapping is enabled in register 0x19)
         */
        if (channel_num == 0) reg = 0x0f;
        else  if (channel_num == 1) reg = 0x16;
        else  if (channel_num == 2) reg = 0x17;
        else  if (channel_num == 3) reg = 0x18;

        if (reg != 0x00)  pcm1865_write(capture_client, reg , new_volume, &g_current_reg_page[device_num], 0x0);
        mutex_unlock(&g_mutex_locks[device_num]);
    }
    return 0;
}

static int vpl_audio_mute_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *info)
{
    //Page 0 / Register 113 (Hex 0x71).(see p.100 of http://www.ti.com/lit/ds/slas831c/slas831c.pdf)
    info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    info->count = 3;
    info->value.integer.min = 0x00;
    info->value.integer.max =0x01;
    info->value.integer.step = 1;
    return 0;
}

static int vpl_audio_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *value)
{
    struct i2c_client *capture_client = NULL ;
    int device_num = value->value.integer.value[0] ;
    int channel_num = value->value.integer.value[1] ;
    int ori_mute = -1 ;
    int reg_val = 0;

    zynq_audio_card_info_t  *card_info = ( zynq_audio_card_info_t  *)snd_kcontrol_chip(kcontrol);

    if ((device_num < 0) || (device_num >=  card_info->data_len)) {
        printk(KERN_INFO "[pcm1865] Set device %d failed !! (0~%d)\n", device_num, (card_info->data_len-1)) ;
        return -1;
    }

    if ((channel_num < 0) || (channel_num >=  MAX_CHANNEL_NUM)) {
        printk(KERN_INFO "[pcm1865] Set channel %d failed !! (0~%d)\n", channel_num, (MAX_CHANNEL_NUM-1)) ;
        return -1;
    }

    capture_client = (struct i2c_client *)card_info->data[device_num];

    if (!capture_client) {
        printk(KERN_INFO "[pcm1865] I2C client obj is NULL!!\n") ;
        return  -1;
    }

    if (g_audio_mute[device_num][channel_num] != PCM1865_UNKWON_GAIN_VALUE) {
        ori_mute = g_audio_mute[device_num][channel_num];
    } else {

        reg_val = pcm1865_read(capture_client, 0x71,&g_current_reg_page[device_num], 0, 0);
        ori_mute = (reg_val & 0x0f) >> channel_num;
        mutex_lock(&g_mutex_locks[device_num]);
        g_audio_mute[device_num][channel_num] = ori_mute;
        mutex_unlock(&g_mutex_locks[device_num]);
    }
    value->value.integer.value[0] = ori_mute;
    return 0 ;
}

static int vpl_audio_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *value)
{
    struct i2c_client *capture_client = NULL ;
    int device_num = value->value.integer.value[0] ;
    int channel_num = value->value.integer.value[1] ;
    int new_mute = value->value.integer.value[2] ;

    zynq_audio_card_info_t  *card_info = ( zynq_audio_card_info_t  *)snd_kcontrol_chip(kcontrol);


    if ((device_num < 0) || (device_num >=  card_info->data_len)) {
        printk(KERN_INFO "[pcm1865] Set device %d failed !! (0~%d)\n", device_num, (card_info->data_len-1)) ;
        return -1;
    }

    if ((channel_num < 0) || (channel_num >=  MAX_CHANNEL_NUM)) {
        printk(KERN_INFO "[pcm1865] Set channel %d failed !! (0~%d)\n", channel_num, (MAX_CHANNEL_NUM-1)) ;
        return -1;
    }

    if ( (new_mute < 0 ) || (new_mute > 1)) {
        printk(KERN_INFO "[pcm1865] Set mute %d failed !! (0,1)\n", new_mute) ;
        return -1;
    }

    capture_client = (struct i2c_client *)card_info->data[device_num];

    if (!capture_client) {
        printk(KERN_INFO "[pcm1865] I2C client obj is NULL!!\n") ;
        return  -1;
    }

    if (g_audio_mute[device_num][channel_num] != new_mute) {
        unsigned int value = 0;

        mutex_lock(&g_mutex_locks[device_num]);
        g_audio_mute[device_num][channel_num] = new_mute;
        value = 1 << channel_num;

        if (new_mute == 1)
            pcm1865_rmw_set(capture_client, 0x71, &g_current_reg_page[device_num], 0x0,  value);
        else if (new_mute == 0)
            pcm1865_rmw_clr(capture_client, 0x71, &g_current_reg_page[device_num], 0x0,  value);
        mutex_unlock(&g_mutex_locks[device_num]);
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////
/*Public interfaces*/

struct snd_kcontrol_new pcm1865_audio_mixter_ctrls[] = {
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Analog Gain",
        .info = vpl_audio_analog_gain_info,
        .get = vpl_audio_analog_gain_get,
        .put = vpl_audio_analog_gain_put,
    },
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Digital Gain",
        .info = vpl_audio_digital_gain_info,
        .get = vpl_audio_digital_gain_get,
        .put = vpl_audio_digital_gain_put,
    },
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Audio Mute",
        .info = vpl_audio_mute_info,
        .get = vpl_audio_mute_get,
        .put = vpl_audio_mute_put,
    }
} ;
EXPORT_SYMBOL_GPL(pcm1865_audio_mixter_ctrls);

unsigned int pcm1865_get_mixter_ctrl_num()
{
    return ARRAY_SIZE(pcm1865_audio_mixter_ctrls);
}
EXPORT_SYMBOL_GPL(pcm1865_get_mixter_ctrl_num);

/*
 NOTE:
 The following notes are refered from http://www.ti.com/lit/ds/symlink/pcm1865.pdf :
(1) The register map of pcm1865 is separetd into four pages: 0,  1,  3, and 253 (0xfd).
(2) The length of each registers in these page is 1 byte.
(3) The registers of all page is not all exported. Should be careful to  check the datasheet.
(4) The page 0 handle all of the devices conirguration.
(5) The page 1 is used to indirectly program coefficients into two fixed function DSP on the IC.
(6) The page 3 contains some additional registers for lower power usage along with page 253.
(7) All undocumented registers should be consitered reserved and should not be written.
(8) Changing between pages is done by writting to register 0x00 with the page that you wnat.
(9) Resetting register is done by writting 0xff to the register 0x00.
(8)
 */

static int set_cached_regs(struct i2c_client *client, unsigned int page, unsigned int offset, unsigned char  val)
{
    int ret  = -1;
    unsigned int i = 0;
    unsigned int size = 0;
    cfg_reg *cfg_reg_ptr = NULL;

    struct pcm1865_platform_data* pdata = NULL;
    int codec_id = -1;

    if (!client) return  -1;

    pdata = client->dev.platform_data;

    if (!pdata) return  -1;

    codec_id= pdata->id;

    if (page == PCM1865_DEV_CONF_PAGE ) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page0[0];
        size = g_reg_maps[codec_id].page0_len;
    } else if (page == PCM1865_DSP_COEF_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page1[0];
        size = g_reg_maps[codec_id].page1_len;
    } else if (page == PCM1865_LOW_POWER_CONF1_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page3[0];
        size = g_reg_maps[codec_id].page3_len;
    } else if (page == PCM1865_LOW_POWER_CONF2_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page253[0];
        size = g_reg_maps[codec_id].page253_len;
    }

    if (cfg_reg_ptr == NULL) return -1;

    for (i = 0; i < size; i++) {
        if (cfg_reg_ptr[i].offset == offset) {
            ret  = 0;
            cfg_reg_ptr[i].value = val;
            break;
        }
    }
    return ret;
}

static int get_cached_regs(struct i2c_client *client, unsigned int page, unsigned int offset, unsigned char*val)
{
    int ret  = -1;
    unsigned int i = 0;
    unsigned int size = 0;
    cfg_reg *cfg_reg_ptr = NULL;

    struct pcm1865_platform_data* pdata = NULL;
    int codec_id = -1;

    if (!client) return  -1;

    pdata = client->dev.platform_data;

    if (!pdata) return  -1;

    codec_id= pdata->id;

    if (page == PCM1865_DEV_CONF_PAGE ) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page0[0];
        size = g_reg_maps[codec_id].page0_len;
    } else if (page == PCM1865_DSP_COEF_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page1[0];
        size = g_reg_maps[codec_id].page1_len;
    } else if (page == PCM1865_LOW_POWER_CONF1_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page3[0];
        size = g_reg_maps[codec_id].page3_len;
    } else if (page == PCM1865_LOW_POWER_CONF2_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page253[0];
        size = g_reg_maps[codec_id].page253_len;
    }

    if (cfg_reg_ptr == NULL) return -1;

    for (i = 0; i < size; i++) {
        if (cfg_reg_ptr[i].offset == offset) {
            ret  = 0;
            *val = cfg_reg_ptr[i].value;
            break;
        }
    }
    return ret;
}

static int pcm1865_change_page(struct i2c_client *i2c, unsigned int new_page, unsigned int *cur_page_ptr)
{
    u8 data[2];
    int ret =2;

    data[0] = 0x00;
    data[1] = new_page & 0xff;

#ifndef VIRTUAL_PCM1865_DEVICE
    ret = i2c_master_send(i2c, data, 2);
#endif
    if (ret == 2) {
        *cur_page_ptr = new_page;
        return 0;
    } else {
        return ret;
    }
}
static unsigned int is_valided_page(unsigned int page)
{
    if (page == PCM1865_DEV_CONF_PAGE )
        return 1;
    else if (page == PCM1865_DSP_COEF_PAGE)
        return 1;
    else if (page == PCM1865_LOW_POWER_CONF1_PAGE)
        return 1;
    else if (page == PCM1865_LOW_POWER_CONF2_PAGE)
        return 1;
    else
        return 0;

}

int pcm1865_write(struct i2c_client *i2c, unsigned int reg, unsigned int val, unsigned int *cur_page_ptr, unsigned int new_page)
{
    u8 data[2];
    int ret = 2;

    if (reg == 	PCM1865_PSEL_REG) {
        if ( is_valided_page(val) == 0)
            return -1;
        else
            return pcm1865_change_page(i2c, val, cur_page_ptr);
    }

    if (*cur_page_ptr != new_page) {

        if ( is_valided_page(new_page) == 0) return -1;

        ret = pcm1865_change_page(i2c, new_page, cur_page_ptr);
        if (ret != 0)
            return ret;
    }

    data[0] = reg & 0xff;
    data[1] = val & 0xff;
#ifndef VIRTUAL_PCM1865_DEVICE
    ret = i2c_master_send(i2c, data, 2);
#endif
    if (ret == 2) {
        set_cached_regs(i2c, new_page, (reg & 0xff), (val & 0xff));
        return 0;
    } else {
        return ret;
    }

}
EXPORT_SYMBOL_GPL(pcm1865_write);


int pcm1865_read(struct i2c_client *i2c, unsigned int reg, unsigned int *cur_page_ptr, unsigned int new_page, unsigned int cached)
{
    int  ret = 0;
    unsigned char val = 0x0;

    if (*cur_page_ptr != new_page) {
        if ( is_valided_page(new_page) == 0) return -1;
        ret = pcm1865_change_page(i2c, new_page, cur_page_ptr);
        if (ret != 0)
            return ret;
    }
    if (cached) {
        ret = get_cached_regs(i2c, new_page, (reg & 0xff), &val);
        if (ret == 0)
            return val;
    }
#ifndef VIRTUAL_PCM1865_DEVICE
    ret = i2c_smbus_read_byte_data(i2c, reg & 0xff);
    if (ret >= 0) set_cached_regs(i2c, new_page, (reg & 0xff), (ret & 0xff));
#endif
    return ret;
}
EXPORT_SYMBOL_GPL(pcm1865_read);

static void pcm1865_rmw(struct i2c_client *i2c, unsigned int reg, unsigned int *cur_page_ptr, unsigned int new_page, u32 clr_bits, u32 set_bits)
{
    u8 tmp = (u8)pcm1865_read(i2c, reg, cur_page_ptr,  new_page, 1);
    tmp &= ~clr_bits;
    tmp |= set_bits;
    pcm1865_write(i2c, reg, tmp,  cur_page_ptr, new_page);
}

void pcm1865_rmw_clr(struct i2c_client *i2c, unsigned int reg, unsigned int *cur_page_ptr, unsigned int new_page,  u32 val)
{
    pcm1865_rmw(i2c, reg, cur_page_ptr, new_page, val, 0);
}
EXPORT_SYMBOL_GPL(pcm1865_rmw_clr);

void pcm1865_rmw_set(struct i2c_client *i2c, unsigned int reg, unsigned int *cur_page_ptr, unsigned int new_page,  u32 val)
{
    pcm1865_rmw(i2c, reg, cur_page_ptr, new_page, 0, val);
}
EXPORT_SYMBOL_GPL(pcm1865_rmw_set);

#ifdef CONFIG_PM_SLEEP
int pcm1865_suspend(struct i2c_client *i2c)
{
    int ret = 0;
    return ret;
}
EXPORT_SYMBOL_GPL(pcm1865_suspend);

int pcm1865_resume(struct i2c_client *i2c)
{
    int ret = 0;
    return ret;
}
EXPORT_SYMBOL_GPL(pcm1865_resume);
#endif


//////////////////////////////////////////////////////////////////////////////////////
static int initialize_register_offset(unsigned int codec_id, unsigned int page)
{
    unsigned int i = 0;
    unsigned int size = 0;
    cfg_reg *cfg_reg_ptr = NULL;
    cfg_reg *cfg_reg_offset_ptr = NULL;

    if (page == PCM1865_DEV_CONF_PAGE ) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page0[0];
        size = g_reg_maps[codec_id].page0_len;
        cfg_reg_offset_ptr = &page0_registers[0];
    } else if (page == PCM1865_DSP_COEF_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page1[0];
        size = g_reg_maps[codec_id].page1_len;
        cfg_reg_offset_ptr = &page1_registers[0];
    } else if (page == PCM1865_LOW_POWER_CONF1_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page3[0];
        size = g_reg_maps[codec_id].page3_len;
        cfg_reg_offset_ptr = &page3_registers[0];
    } else if (page == PCM1865_LOW_POWER_CONF2_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page253[0];
        size = g_reg_maps[codec_id].page253_len;
        cfg_reg_offset_ptr = &page253_registers[0];
    }

    if (cfg_reg_ptr == NULL) return -1;

    for (i = 0; i < size; i++) {
        cfg_reg_ptr[i].offset = cfg_reg_offset_ptr[i].offset;
    }
    return 0;
}

static int clear_cached_resgisters(unsigned int codec_id, unsigned int page)
{
    unsigned int i = 0;
    unsigned int size = 0;
    cfg_reg *cfg_reg_ptr = NULL;

    if (page == PCM1865_DEV_CONF_PAGE ) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page0[0];
        size = g_reg_maps[codec_id].page0_len;
    } else if (page == PCM1865_DSP_COEF_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page1[0];
        size = g_reg_maps[codec_id].page1_len;
    } else if (page == PCM1865_LOW_POWER_CONF1_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page3[0];
        size = g_reg_maps[codec_id].page3_len;
    } else if (page == PCM1865_LOW_POWER_CONF2_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page253[0];
        size = g_reg_maps[codec_id].page253_len;
    }

    if (cfg_reg_ptr == NULL) return -1;

    for (i = 0; i < size; i++) {
        cfg_reg_ptr[i].value = 0x00;
    }
    return 0;

}

static int update_cached_registers(struct i2c_client *client, unsigned int codec_id, unsigned int page)
{
    unsigned int i = 0;
    unsigned int size = 0;
    cfg_reg *cfg_reg_ptr = NULL;

    if (page == PCM1865_DEV_CONF_PAGE ) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page0[0];
        size = g_reg_maps[codec_id].page0_len;
    } else if (page == PCM1865_DSP_COEF_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page1[0];
        size = g_reg_maps[codec_id].page1_len;
    } else if (page == PCM1865_LOW_POWER_CONF1_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page3[0];
        size = g_reg_maps[codec_id].page3_len;
    } else if (page == PCM1865_LOW_POWER_CONF2_PAGE) {
        cfg_reg_ptr = &g_reg_maps[codec_id].page253[0];
        size = g_reg_maps[codec_id].page253_len;
    }

    if (cfg_reg_ptr == NULL) return -1;

    for (i = 0; i < size; i++) {
        cfg_reg_ptr[i].value =  pcm1865_read(client, cfg_reg_ptr[i].offset, &g_current_reg_page[codec_id], page, 0);
    }
    return 0;

}

static int board_specific_init(struct i2c_client *client, unsigned int codec_id, unsigned int mode)
{
    int ret  = 0;

    pcm1865_rmw_clr(client, 0x0b, &g_current_reg_page[codec_id], 0x0,  0x0c);
    pcm1865_rmw_set(client, 0x0b, &g_current_reg_page[codec_id], 0x0,  0x0c);//16 bits : w 94 0b 4f => w 94  0b ( 0c | old)

    pcm1865_rmw_clr(client, 0x0b, &g_current_reg_page[codec_id], 0x0,  0x03);
    pcm1865_rmw_set(client, 0x0b, &g_current_reg_page[codec_id], 0x0,  0x03);//TDM/DSP (256Fs BCK is required): w 94 0b 47 => w 94 0b (03 | old)

    pcm1865_rmw_clr(client, 0x0c, &g_current_reg_page[codec_id], 0x0,  0x03);
    pcm1865_rmw_set(client, 0x0c, &g_current_reg_page[codec_id], 0x0,  0x01); // 4ch TDM : w 94 0c  01

    if (codec_id == 0) {
        //bus:0x2,  addr:0x4b
        pcm1865_write(client, 0x0d, 0x00, &g_current_reg_page[codec_id], 0x0); //0 bits :  w 94 0d 00

        pcm1865_rmw_clr(client, 0x08, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x06, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x07, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x09, &g_current_reg_page[codec_id], 0x0,  0x3f);

        /*
         i2cset -y 2 0x4b 0x08 0x50 --> 01 01 0000
        i2cset -y 2 0x4b 0x06 0x60  --> 01 10  0000
        i2cset -y 2 0x4b 0x07 0x50  --> 01 01  0000
        i2cset -y 2 0x4b 0x09 0x60 -->  01 10 0000

         */

        pcm1865_rmw_set(client, 0x08, &g_current_reg_page[codec_id], 0x0,  0x10);//Left ch2 : 1P-1M (differential) : w 96 08 50  =>  w 96  08 ( 10 | old)  (0x08 : ADC 2 Input Channel Select (ADC2L))
        pcm1865_rmw_set(client, 0x06, &g_current_reg_page[codec_id], 0x0,  0x20);//Left ch1 : 4P-4M (differential)  :  w 96 06 60 =>  w 96 06 ( 20 | old)  (0x06 : ADC 1 Input Channel Select (ADC1L))
        pcm1865_rmw_set(client, 0x07, &g_current_reg_page[codec_id], 0x0,  0x10);//Right ch1 : 2P-2M (differential)  : w 96 07 50 => w 96 07 (10 | old)   (0x07 : ADC 1 Input Channel Select (ADC1R))
        pcm1865_rmw_set(client, 0x09, &g_current_reg_page[codec_id], 0x0,  0x20);//Right ch2 : 3P-3M (differentila)  : w 96 09 60 => w 96 09 (20 | old )  (0x09 : ADC 2 Input Channel Select (ADC2R))
    } else if (codec_id == 1) {
        //bus:0x2, addr:0x4a
        //pcm1865_write(client, 0x0d, 0x10, &g_current_reg_page[codec_id], 0x0); //16 bits : w 94 0d 10
        pcm1865_write(client, 0x0d, 0x00, &g_current_reg_page[codec_id], 0x0); //0 bits : w 94 0d  00

        pcm1865_rmw_clr(client, 0x08, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x06, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x07, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x09, &g_current_reg_page[codec_id], 0x0,  0x3f);

        /*
         i2cset -y 2 0x4a 0x08 0x50 --> 01 01 0000
        i2cset -y 2 0x4a 0x06 0x48 --> 01 00 1000
        i2cset -y 2 0x4a 0x07 0x50 --> 01 01 0000
        i2cset -y 2 0x4a 0x09 0x48  --> 01 00 1000

         */
        pcm1865_rmw_set(client, 0x08, &g_current_reg_page[codec_id], 0x0,  0x10);//Left ch2 : 1P-1M (differential) : w 94 08 50  =>  w 94  08 ( 10 | old)  (0x08 : ADC 2 Input Channel Select (ADC2L))
        pcm1865_rmw_set(client, 0x06, &g_current_reg_page[codec_id], 0x0,  0x08);//Left ch1 : L4 (single end) : w 94 06 48 => w 96 06 ( 08 | old)  (0x06 : ADC 1 Input Channel Select (ADC1L))
        pcm1865_rmw_set(client, 0x07, &g_current_reg_page[codec_id], 0x0,  0x10);//Right ch1 : 2P-2M (differential)  : w 94 07 50 => w 94 07 (10 | old)   (0x07 : ADC 1 Input Channel Select (ADC1R))
        pcm1865_rmw_set(client, 0x09, &g_current_reg_page[codec_id], 0x0,  0x08);//Right ch2 : R4 (single end) :w 94 09 48 =>w 96 09 ( 08  | old )  (0x09 : ADC 2 Input Channel Select (ADC2R))
    } else if (codec_id == 2) {
        //bus:0x0, addr:0x4a
        //pcm1865_write(client, 0x0d, 0x20, &g_current_reg_page[codec_id], 0x0); //32 bits : w 94 0d 20
        pcm1865_write(client, 0x0d, 0x00, &g_current_reg_page[codec_id], 0x0); //0  bits : w 94 0d 00

        pcm1865_rmw_clr(client, 0x08, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x06, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_clr(client, 0x07, &g_current_reg_page[codec_id], 0x0,  0x3f);
        pcm1865_rmw_set(client, 0x09, &g_current_reg_page[codec_id], 0x0,  0x3f);
        /*
         	i2cset -y 0 0x4a 0x08 0x50  --> 01 01 0000
        	i2cset -y 0 0x4a 0x06 0x60  --> 01 10  0000
        	i2cset -y 0 0x4a 0x07 0x50  --> 01 01 0000
        	i2cset -y 0 0x4a 0x09 0x60  --> 01 10  0000
         */
        pcm1865_rmw_set(client, 0x08, &g_current_reg_page[codec_id], 0x0,  0x10);//Left ch2 : 1P-1M (differential) : w 94 08 50  =>  w 94  08 ( 10 | old)  (0x08 : ADC 2 Input Channel Select (ADC2L))
        pcm1865_rmw_set(client, 0x06, &g_current_reg_page[codec_id], 0x0,  0x20);//Left ch1 : 4P-4M (differential)  :  w 94 06 60 =>  w 94 06 ( 20 | old)  (0x06 : ADC 1 Input Channel Select (ADC1L))
        pcm1865_rmw_set(client, 0x07, &g_current_reg_page[codec_id], 0x0,  0x10);//Right ch1 : 2P-2M (differential)  : w 94 07 50 => w 94 07 (10 | old)   (0x07 : ADC 1 Input Channel Select (ADC1R))
        pcm1865_rmw_set(client, 0x09, &g_current_reg_page[codec_id], 0x0,  0x20);//Right ch2 : 3P-3M (differentila)  : w 94 09 60 => w 94 09 (20 | old )  (0x09 : ADC 2 Input Channel Select (ADC2R))
    }

    if (mode == 1) {
        /*
         i2cset -y 0 0x4a 0x20 0x11
         i2cset -y 0 0x4a 0x20 0x50
         i2cset -y 0 0x4a 0x28 0x0
        i2cset -y 0 0x4a 0x25 0xb
        i2cset -y 0 0x4a 0x26 0x5
        i2cset -y 0 0x4a 0x27 0x3f
         */
        printk(KERN_INFO"[pcm1865]Set codec_id %u as master mode!!\n", codec_id);
        pcm1865_rmw_clr(client, 0x20, &g_current_reg_page[codec_id], 0x0,  0x10);
        pcm1865_rmw_set(client, 0x20, &g_current_reg_page[codec_id], 0x0,  0x10);//Masetr : w 94  20 13 => w 94  20  (10 | old)

        /*
         SCK/Xtal selection:
        Default value: 00
        00: SCK or Xtal (Default)
        01: SCK
        10: Xtal
        11: (Reserved)
        */
        pcm1865_rmw_clr(client, 0x20, &g_current_reg_page[codec_id], 0x0,  0xc0);
        pcm1865_rmw_set(client, 0x20, &g_current_reg_page[codec_id], 0x0,  0x40);//sck

        /*
         Enable Auto Clock Detector Configuration
        Default value: 1
        0: Disable
        1: Enable (Default)
         */
        pcm1865_rmw_clr(client, 0x20, &g_current_reg_page[codec_id], 0x0,  0x01);
        pcm1865_rmw_set(client, 0x20, &g_current_reg_page[codec_id], 0x0,  0x00);//sck

        /*
         i2cset -y 0 0x4a 0x28 0x0
        i2cset -y 0 0x4a 0x25 0xb
        i2cset -y 0 0x4a 0x26 0x5
        i2cset -y 0 0x4a 0x27 0x3f
         */
        pcm1865_write(client, 0x28, 0x00, &g_current_reg_page[codec_id], 0x0);
        pcm1865_write(client, 0x25, 0x0b, &g_current_reg_page[codec_id], 0x0);
        pcm1865_write(client, 0x26, 0x05, &g_current_reg_page[codec_id], 0x0);
        pcm1865_write(client, 0x27, 0x3f, &g_current_reg_page[codec_id], 0x0);

    } else {
        pcm1865_write(client, 0x20, 0x01, &g_current_reg_page[codec_id], 0x0);

    }
#if 0
    pcm1865_write(client, 0x25, 0x07, &g_current_reg_page[codec_id], 0x0); //1/8: w 94 25 07 (default)
    pcm1865_write(client, 0x26, 0x01, &g_current_reg_page[codec_id], 0x0); //1/2: w 94 26 01
    pcm1865_write(client, 0x27, 0xff, &g_current_reg_page[codec_id], 0x0); //1/256: w 94 27 ff
#endif
    return  ret;
}

static int codec_init(struct i2c_client *client, unsigned int codec_id, unsigned int mode)
{
    int ret = 0 ;
    unsigned int size = sizeof(reset_registers)/sizeof(reset_registers[0]);
    cfg_reg *cfg_reg_ptr = &reset_registers[0];
    int  i  = 0;
    if (!client) return -1;

    for (i = 0; i < size; i++) {
        if (pcm1865_write(client, cfg_reg_ptr[i].offset, cfg_reg_ptr[i].value, &g_current_reg_page[codec_id], g_current_reg_page[codec_id]) != 0) return -1;
    }

#ifndef VIRTUAL_PCM1865_DEVICE
    initialize_register_offset(codec_id, PCM1865_DEV_CONF_PAGE);
    initialize_register_offset(codec_id, PCM1865_DSP_COEF_PAGE);
    initialize_register_offset(codec_id, PCM1865_LOW_POWER_CONF1_PAGE);
    initialize_register_offset(codec_id, PCM1865_LOW_POWER_CONF2_PAGE);

    clear_cached_resgisters(codec_id, PCM1865_DEV_CONF_PAGE);
    clear_cached_resgisters(codec_id, PCM1865_DSP_COEF_PAGE);
    clear_cached_resgisters(codec_id, PCM1865_LOW_POWER_CONF1_PAGE);
    clear_cached_resgisters(codec_id, PCM1865_LOW_POWER_CONF2_PAGE);

    update_cached_registers(client, codec_id, PCM1865_LOW_POWER_CONF2_PAGE);
    update_cached_registers(client, codec_id, PCM1865_LOW_POWER_CONF1_PAGE);
    update_cached_registers(client, codec_id, PCM1865_DSP_COEF_PAGE);
    update_cached_registers(client, codec_id, PCM1865_DEV_CONF_PAGE);
#endif


    ret  = board_specific_init(client, codec_id, mode);

    return ret ;
}

static int pcm1865_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
    int ret = 0;
    int i = 0;
    struct pcm1865_platform_data* pdata = NULL;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    printk(KERN_INFO"[pcm1865]Enter probe()\n");

    if (adapter == NULL) {
        printk(KERN_INFO"[pcm1865]I2C adpter object is NULL!!\n");
        return -ENODEV;
    }

    if (!client->dev.platform_data) {
        printk(KERN_INFO"[pcm1865]No platform data found\n");
        return -ENODEV;
    }

    pdata =client->dev.platform_data;

    printk(KERN_INFO "[pcm1865]Chip found @ 0x%02x (bus:%d)(id:%d)(ptr:%p)\n", client->addr, adapter->nr, pdata->id, client);

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
        g_analog_gain[pdata->id][i] = PCM1865_UNKWON_GAIN_VALUE;

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
        g_digital_gain[pdata->id][i] = PCM1865_UNKWON_GAIN_VALUE;

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
        g_audio_mute[pdata->id][i] = PCM1865_UNKWON_GAIN_VALUE;

    g_reg_maps[pdata->id].page0_len = PCM1865_PAGE0_REG_COUNT;
    g_reg_maps[pdata->id].page1_len = PCM1865_PAGE1_REG_COUNT;
    g_reg_maps[pdata->id].page3_len = PCM1865_PAGE3_REG_COUNT;
    g_reg_maps[pdata->id].page253_len = PCM1865_PAGE253_REG_COUNT;

    ret = codec_init(client, pdata->id, pdata->mode) ;
    if(ret < 0) {
        printk(KERN_INFO "[pcm1865]failed to  call codec_init() !!\n");
        goto exit;
    }

    mutex_init(&g_mutex_locks[pdata->id]);

    create_register_dump_sysfs(&client->dev.kobj,  pdata->id);

    printk(KERN_INFO"[pcm1865]Leave probe()\n");

exit:
    return ret;
}

static int pcm1865_remove(struct i2c_client *client)
{
    struct pcm1865_platform_data* pdata = NULL;

    if (!client) return  -1;

    pdata =client->dev.platform_data;

    if (!pdata) return  -1;

    mutex_destroy(&g_mutex_locks[pdata->id]);

    printk(KERN_INFO"[pcm1865]Enter remove()\n");
    destroy_register_dump_sysfs(&client->dev.kobj);
    printk(KERN_INFO"[pcm1865]Leave remove()\n");
    return 0;
}

static const struct i2c_device_id pcm1865_id[] = {
    { "pcm1865", 0 },
    { },
};

MODULE_DEVICE_TABLE(i2c, pcm1865_id);

static struct i2c_driver pcm1865_driver = {
    .driver = {
        .owner	= THIS_MODULE,
        .name	= PCM1865_DRIVER_NAME,
    },
    .probe		= pcm1865_probe,
    .remove		= pcm1865_remove,
    .id_table	= pcm1865_id,
};

module_i2c_driver(pcm1865_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PCM1865 driver");
MODULE_AUTHOR("Jeff Liao <qustion@gmail.com>");