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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>

#include <linux/vmalloc.h>
#include <linux/pci.h>

#include <asm/cacheflush.h>

#include "zynq_gpio.h"
#include "zynq_pcm1865.h"
#include "zynq_debug.h"
#include "zynq_audio.h"
#include "modules/zynq_regs.h"
#include "zynq_fpga_verify.h"

#define PRE_ALLOCATE_SIZE  (72*1024*4)

#define PERIOD_BYTES_MAX	16380

#ifndef FPGA_VERIF	
	#define  VIRTUAL_AUDIO_DEVICE 1
#endif


#ifdef  VIRTUAL_AUDIO_DEVICE
#include "zynq_audio_virtual_dev.h"
#endif

#define MAX_AUDIO_CODEC_NUM 0x3

///////////////////////////////////////////////////////////////////////////////////////
extern void __iomem *zynq_reg_base ;
static u8 is_first_enable_audio1 = 1;
static u8 is_first_enable_audio = 1;
static void __iomem *zynq_audio_reg_base = NULL;
///////////////////////////////////////////////////////////////////////////////////////
//There is one transfer channel of PCI from audio swtich module on the Zynq platfom.
enum {
	MOZART_I2S0 = 0,
	MOZART_I2S_NUM
};
enum {
	DMA_CHN_0 = 0,
	DMA_CHN_1 =1,
	DMA_CHN_NUM
};
//////////////////////////////////////////////////////////////////////////////////////

/* The actual rates supported by the card */
static unsigned int pcm1865_supported_samplerates[] = {
	8000, 16000, 32000, 44100, 48000, 64000, 88200, 96000, 176400, 192000
};
static struct snd_pcm_hw_constraint_list constraints_rates = {
	.count = ARRAY_SIZE(pcm1865_supported_samplerates),
	.list = pcm1865_supported_samplerates,
	.mask = 0,
};
#if 0
 static unsigned int pcm1865_supported_periodsize[] = {
	384,    448,    512,    640,
	384*2,  448*2,  512*2,  640*2,	
	384*4,  448*4,  512*4,  640*4,
	384*8,  448*8,  512*8,  640*8,
	384*16, 448*16, 512*16, 640*16,
	384*32, 448*32, 512*32, 640*32,
	384*64, 448*64, 512*64, 640*64,
	384*128,448*128,512*128
 }; 

static struct snd_pcm_hw_constraint_list constraints_period_size = {
	.count = ARRAY_SIZE(pcm1865_supported_periodsize),
	.list =pcm1865_supported_periodsize,
	.mask = 0,
};
#endif

static 	unsigned int dummy_buffer_size = 0;
static	u32 dummy_buffer_handle = 0;
static 	char *dummy_buffer = 0;

typedef struct {
	int dev_num;
	int dma_chn;
}snd_i2s_dev_private;

struct audio_stream {
	int stream_id;		/* numeric identification */
	unsigned int active:1;	/* we are using this stream for transfer now */
	int dma_pos;			/* dma processed position in frames */
	struct snd_pcm_substream *stream;
	unsigned int ring_buf_size; // bytes
	unsigned int ring_buf_pos;  //position in buffer
	unsigned int dma_size;
};

struct snd_i2s {
	struct snd_card *card;
	struct snd_pcm *pcm[MOZART_I2S_NUM];
	long samplerate[MOZART_I2S_NUM];
	long vvdc_param[MOZART_I2S_NUM];
	int dma_irq;
	int dma_irq_acks;
	struct mutex dma_irq_mutex;
	unsigned int dma_requested;
	struct audio_stream s[MOZART_I2S_NUM][2];	/* I2S0 playback/capture */
#ifdef VIRTUAL_AUDIO_DEVICE
	struct minivosc_device *virtual_dev;
#endif
	
};

struct audio_context
{
	 struct i2c_client *i2c_clients[MAX_AUDIO_CODEC_NUM];
	 struct i2c_adapter *i2c_adapters[MAX_AUDIO_CODEC_NUM];
	 struct snd_i2s  *chip;
	 unsigned int cur_page;
};

static struct audio_context  g_audio_ctx;

zynq_audio_card_info_t g_card_info_for_codec = {NULL, 0};

///////////////////////////////////////////////////////////////////////////////////////
#define AUDIO_CH0		"pcm1865-0"
#define AUDIO_CH1		"pcm1865-1"
#define AUDIO_CH2		"pcm1865-2"

/*
 PCM1865 #0 (addr : 0x96, bus : I2C3)
 PCM1865 #1 (addr : 0x94, bus : I2C3)
 PCM1865 #2 (addr:0x94, bus: GEN1_I2C)
 
 
pin: CAM_I2C (I2C3)
bus:2
devices:
name: PCM1865 (audio codec) 	addr:0x96
name: PCM1865 (audio codec) 	addr:0x94

pin: GEN1_I2C
bus: 0
devices
name: PCM1865 (audio codec)	 addr:0x94


 */
#define AUDIO_CH0_I2C_BUS 0x2
#define AUDIO_CH1_I2C_BUS 0x2
#define AUDIO_CH2_I2C_BUS 0x0
#define AUDIO_CH0_I2C_ADDR (0x96 >> 1)//0x4b
#define AUDIO_CH1_I2C_ADDR (0x94 >> 1)//0x4a 
#define AUDIO_CH2_I2C_ADDR (0x94 >> 1)//0x4a 

#define AUDIO_IRQ_PIN GPIO_PU4/*GPIO_PU1*//*GPIO_PU4*/

#define PCM1865_I2C_ID_NAME "pcm1865"

//////////////////////////////////////////////////////////////////////////////////////
/*I2C slave and I2C bus mapping*/

struct audio_i2c_bus_adapter
{
	unsigned int bus_num;
	struct i2c_adapter *i2c_adap;
};


static struct audio_i2c_bus_adapter  audio_i2c_adapters[] = {
	[0] = {
		.bus_num = AUDIO_CH0_I2C_BUS 
	},
	[1] = {
		.bus_num = AUDIO_CH1_I2C_BUS 
	},
	[2] = {
		.bus_num = AUDIO_CH2_I2C_BUS 
	}
};


static void audio_setup_i2c_adapter(void) 
{
	int i = 0;
	unsigned int  bus_num  = (unsigned int)-1;
	struct i2c_adapter *i2c_adap = NULL;
	for (i = 0;  i < ARRAY_SIZE(audio_i2c_adapters); i ++)
	{
		if (audio_i2c_adapters[i].bus_num != bus_num)
		{
		
			audio_i2c_adapters[i].i2c_adap = i2c_get_adapter(audio_i2c_adapters[i].bus_num);
			i2c_adap  = 	audio_i2c_adapters[i].i2c_adap;
			bus_num = audio_i2c_adapters[i].bus_num;
		}
		else{
			audio_i2c_adapters[i].i2c_adap = i2c_adap;
		}
	}
	return;
}

static  struct i2c_adapter *audio_get_i2c_adapter_by_bus_num(unsigned int  bus_num)
{
	int i = 0;
	for (i = 0;  i < ARRAY_SIZE(audio_i2c_adapters); i ++)
	{
		if (bus_num == audio_i2c_adapters[i].bus_num)
			return audio_i2c_adapters[i].i2c_adap;
	}
	
	return NULL;
}

static void  audio_rls_i2c_adapter(void)
{
	int i = 0;
	unsigned int  bus_num  = (unsigned int)-1;
	for (i = 0;  i < ARRAY_SIZE(audio_i2c_adapters); i ++)
	{
		if (audio_i2c_adapters[i].bus_num != bus_num)
		{
			i2c_put_adapter(audio_i2c_adapters[i].i2c_adap);
			bus_num = audio_i2c_adapters[i].bus_num;
		}
	}
	return;
}

//////////////////////////////////////////////////////////////////////////////////////
/*definition for IRQ and I2C  slave address*/

struct audio_codec_info{
	unsigned int enable;
	const char *name;
	struct i2c_board_info board_info;
};

static struct pcm1865_platform_data pcm1865_pdata_0 = {
	.id = 0,
	.mode = 0
};

static struct pcm1865_platform_data pcm1865_pdata_1= {
	.id = 1, 
	.mode = 0
};

static struct pcm1865_platform_data pcm1865_pdata_2 = {
	.id = 2,
	.mode =1
};


static struct resource audio_interrupt_resources[] ={
	[0] = {
		.name = "interrupt0",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE
	}
};

static void zynq_audio_setup_interrupt(void)
{
		audio_interrupt_resources[0].start =  gpio_to_irq(AUDIO_IRQ_PIN);
		audio_interrupt_resources[0].end =  gpio_to_irq(AUDIO_IRQ_PIN);
		return;
}

static struct audio_codec_info  zynq_audio_codec_info[] = {
	{
		.enable = 1,
		.name	= AUDIO_CH0,
		.board_info = {
			I2C_BOARD_INFO( PCM1865_I2C_ID_NAME, AUDIO_CH0_I2C_ADDR ),
			.platform_data = &pcm1865_pdata_0
		}
	},
	{
		.enable = 1,
		.name	= AUDIO_CH1,
		.board_info = {
			I2C_BOARD_INFO( PCM1865_I2C_ID_NAME, AUDIO_CH1_I2C_ADDR ),
			.platform_data = &pcm1865_pdata_1
		}
	},
	{
		.enable = 1,
		.name	= AUDIO_CH2,
		.board_info = {
			I2C_BOARD_INFO( PCM1865_I2C_ID_NAME, AUDIO_CH2_I2C_ADDR ),
			.platform_data = &pcm1865_pdata_2
		}
	}
};

/////////////////////////////////////////////////////////////////////////////////////////
/*isr  related work*/


static irqreturn_t audio_dma_isr(int irq, void *dev_id)
{
	struct snd_i2s *chip =  (struct snd_i2s *)dev_id;
	struct audio_stream *s1 = NULL;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	snd_i2s_dev_private *dev_private;
	 snd_pcm_uframes_t  available = 0 ;
	 
	s1 = &chip->s[ MOZART_I2S0][SNDRV_PCM_STREAM_CAPTURE];
	if (s1) {
		if (s1->active && s1->stream) {
				substream = s1->stream;
				runtime = substream->runtime;
				dev_private = runtime->private_data;
				//Get the available (readable) space for capture
				available = snd_pcm_capture_avail(runtime);
				//zynq_printk(0, "snd_pcm_capture_avail()--->%d\n",snd_pcm_capture_avail(runtime));
				if (available == 0) {
					s1->ring_buf_pos += s1->dma_size;
					s1->ring_buf_pos  %= s1->ring_buf_size;
					snd_pcm_period_elapsed(s1->stream);
				}
				
				{
					void __iomem *base =   zynq_audio_reg_base;
					u32 reg = 0;
					u32 val = 0;
					
					reg = FPGA_INTERRUPT_REG;
					val = FPGA_AUDIO_INTERRUPT_MASK ;
					fpga_reg_write(base, reg, val);	
		
					reg = FPGA_AUDIO_PCM_BUFFER_REG; 
					if (available == 0){
						val = runtime->dma_addr + s1->ring_buf_pos;
					}else {
						val = dummy_buffer_handle;
					}
					fpga_reg_write(base, reg, val);
					
					reg = FPGA_PCI_PUSH_READY_REG;
					val = FPGA_AUDIO_PCI_PUSH_READY_MASK;
					fpga_reg_write(base, reg, val);	
				
				}
		
				return IRQ_HANDLED;
		}
	}
	return IRQ_NONE;
}

static int audio_dma_irq_request(struct snd_i2s *chip)
{
	zynq_audio_setup_interrupt();
	mutex_lock(&chip->dma_irq_mutex);
	if (chip->dma_irq < 0) {
		chip->dma_irq = audio_interrupt_resources[0].start;
		if(request_irq (chip->dma_irq, audio_dma_isr, IRQF_ONESHOT   | IRQF_TRIGGER_HIGH/*IRQF_TRIGGER_RISING*//*IRQF_ONESHOT  |IRQF_TRIGGER_HIGH  *//*IRQF_ONESHOT   | IRQF_TRIGGER_HIGH*//*IRQF_SHARED | IRQF_TRIGGER_RISING*/, "zynq_audio", chip)) 	{
			zynq_printk(0,  "[zynq_audio] IRQ request fail\n");
			mutex_unlock(&chip->dma_irq_mutex);
			return -EBUSY;
		}
		// set Level Trigger for LLP interrupts
		//fLib_SetIntTrig(chip->dma_irq, LEVEL, H_ACTIVE);
	}
	chip->dma_irq_acks++;
	mutex_unlock(&chip->dma_irq_mutex);
	return 0;
}

static void audio_dma_irq_free(struct snd_i2s *chip)
{	
	zynq_printk(0,  "[zynq_audio] Enter audio_dma_irq_free()\n");
	mutex_lock(&chip->dma_irq_mutex);
	if (chip->dma_irq_acks > 0) {
		chip->dma_irq_acks--;
	}
	if (chip->dma_irq_acks == 0 && chip->dma_irq >= 0) {
		zynq_printk(0,  "[zynq_audio]Call free_irq()\n");
		free_irq(chip->dma_irq, chip);
		{
					void __iomem *base =   zynq_audio_reg_base;
					u32 reg = 0;
					u32 val = 0;
					
					reg = FPGA_INTERRUPT_REG;
					val = FPGA_AUDIO_INTERRUPT_MASK ;
					fpga_reg_write(base, reg, val);	
					
					val = fpga_reg_read(base, reg);
					while(1) {
						if (val == 0) break;
						zynq_printk(0,  "[zynq_audio] Interrupt is not cleared!!\n");
					}
					reg = FPGA_AUDIO_READY_REG ;
					val = 0;
					fpga_reg_write(base, reg, val);
					is_first_enable_audio1 = 1;
		}
		chip->dma_irq = -1;
	}
	mutex_unlock(&chip->dma_irq_mutex);
	zynq_printk(0,  "[zynq_audio] Leave audio_dma_irq_free()\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
/*pcm1865 control functions*/

int config_pcm1865(int codec_id, unsigned int page, unsigned int  reg, unsigned int val)
{
	struct i2c_client *i2c =  g_audio_ctx.i2c_clients[codec_id];
	
	if (!i2c) return -1;
	
	return pcm1865_write(i2c, reg, val, &g_audio_ctx.cur_page, page);
}

int check_pcm1865(int codec_id,  unsigned int page, unsigned int reg, unsigned int *retval)
{
	int ret = 0;
	struct i2c_client *i2c =  g_audio_ctx.i2c_clients[codec_id];
	
	if (!i2c) return -1;
	
	ret = pcm1865_read(i2c, reg, &g_audio_ctx.cur_page, page, 1);
	
	if (ret >= 0) 
	{
		*retval = (unsigned int)ret;
		return  0;
	}
	
	return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////
/*some HW control functions*/

static void audio_enable_device(struct audio_stream *s, int enable)
{
	return;
}

 static void audio_enable_stream(struct audio_stream *s, int enable, int stream_id)
{
	return;
}


static void audio_start_dma(struct audio_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_i2s_dev_private *dev_private = runtime->private_data;
	int dev_num = 0;
	unsigned int dma_size = 0;
	
	if (!dev_private) return;
	
	if (is_first_enable_audio) {
		
		void __iomem *base =   zynq_audio_reg_base;
		u32 reg = 0;
		u32 val = 0;
		
		dev_num = dev_private->dev_num;
		dma_size = frames_to_bytes(runtime, runtime->period_size); // transfer one period size data each dma move
		//TODO: Let the HW know the period size.
		zynq_printk(0, "[zynq_audio]dma_size = %u(bytes),  period_size = %u (frames),  frame_size =%u(bytes), buffer_size = %u(frames),  dma_addr=0x%x\n", (unsigned int)dma_size,   (unsigned int)runtime->period_size,  (unsigned int)(runtime->frame_bits / 8),  (unsigned int)runtime->buffer_size, (unsigned int)runtime->dma_addr);
	
		s->ring_buf_pos = 0;
		s->ring_buf_size = frames_to_bytes(runtime, runtime->buffer_size);
		s->dma_size =  frames_to_bytes(runtime, runtime->period_size);
		
		is_first_enable_audio = 0;
				
		reg = FPGA_AUDIO_PCM_BUFFER_REG; 
		val = runtime->dma_addr;
		fpga_reg_write(base, reg, val);
		
		reg = FPGA_AUDIO_PERIOD_SIZE_REG;
		val = runtime->period_size;
		fpga_reg_write(base, reg, val);	
		
		reg = FPGA_PCI_PUSH_READY_REG;
		val = FPGA_AUDIO_PCI_PUSH_READY_MASK;
		fpga_reg_write(base, reg, val);	
		
		if (is_first_enable_audio1 == 1) {
			is_first_enable_audio1 = 0;
			reg = FPGA_AUDIO_READY_REG ;
			val = 1;
			fpga_reg_write(base, reg, val);
		}
	}
	return;
}

static void audio_stop_dma(struct audio_stream *s)
{
	return;
}

static u_int audio_get_dma_pos(struct audio_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset = 0;
	
	offset = s->ring_buf_pos;
	
	offset = bytes_to_frames(runtime, offset);
	if (offset >= runtime->buffer_size) {
		offset = 0;
	}
	return offset;
}

static void snd_i2s_set_samplerate(struct snd_i2s *chip,  long rate, int device)
{
	/*Set the pre-defined samplerate to I2S controller  and audio codec through I2C*/
	return;
}
/////////////////////////////////////////////////////////////////////////////////////////
/*PCM related work*/

#ifndef VIRTUAL_AUDIO_DEVICE
static struct snd_pcm_hardware snd_i2s_playback =
{
	.info =		 (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_NONINTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats =	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |SNDRV_PCM_FMTBIT_U24_LE,
	.rates =		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
                                   SNDRV_PCM_RATE_32000 |SNDRV_PCM_RATE_44100 |\
                                   SNDRV_PCM_RATE_48000 |SNDRV_PCM_RATE_KNOT),
	.rate_min =		8000,
	.rate_max =		48000,
	.channels_min =		1,
	.channels_max =	16,
	.periods_min =		1,
	.periods_max =		16,
	.buffer_bytes_max =	16*2*16*1024*16,
	.period_bytes_min =		2*16*1, 
	.period_bytes_max =	2*16*1024*16 
};

static struct snd_pcm_hardware snd_i2s_capture =
{
	.info =		 (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_NONINTERLEAVED |
				   SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats =	SNDRV_PCM_FMTBIT_S16_LE  |  SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_U24_LE,
	.rates =		 (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
				   SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
				   SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_KNOT),
	.rate_min =		8000,
	.rate_max =		48000,
	.channels_min =		1,
	.channels_max =	16,
	.periods_min =		1,
	.periods_max =		16,
	.buffer_bytes_max = 16*2*16*1024*16, 
	.period_bytes_min =	 2*16*1,
	.period_bytes_max =	2*16*1024*16
};
#endif

static int snd_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_i2s *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_i2s_dev_private *dev_private = runtime->private_data;
	struct audio_stream *s, *s1;
	int err = 0;
	
	s = &chip->s[dev_private->dev_num][substream->pstr->stream];
	s1 = &chip->s[dev_private->dev_num][substream->pstr->stream ^ 1];
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if ((runtime->dma_addr % 8) != 0) {
			zynq_printk(0, "runtime->dma_addr not aligned with APBC DMA data size!\n");
			return -EPERM;
		}
		s->active = 1;
		audio_start_dma(s);
		audio_enable_device(s, 1);
		audio_enable_stream(s, 1, substream->pstr->stream);
#ifdef VIRTUAL_AUDIO_DEVICE
		minivosc_timer_start(chip->virtual_dev, jiffies, substream->stream);
#endif		
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		s->active = 0;
		audio_stop_dma(s);
		audio_enable_stream(s, 0, substream->pstr->stream);
		if (s1->active == 0) {
			audio_enable_device(s, 0);
		}
#ifdef VIRTUAL_AUDIO_DEVICE
		minivosc_timer_stop(chip->virtual_dev, substream->stream);
#endif
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		zynq_printk(0, "[zynq_audio] The SNDRV_PCM_TRIGGER_SUSPEND has been called!!\n");
#ifdef VIRTUAL_AUDIO_DEVICE
		minivosc_timer_stop(chip->virtual_dev, substream->stream);
#endif		
		break;
	case SNDRV_PCM_TRIGGER_RESUME:	
		zynq_printk(0, "[zynq_audio] The SNDRV_PCM_TRIGGER_RESUME  has been called!!\n");
#ifdef VIRTUAL_AUDIO_DEVICE
		minivosc_timer_start(chip->virtual_dev, jiffies, substream->stream);
#endif			
		break;
	default:
		err = -EINVAL;
		break;
	}
	return err;
}

static int snd_i2s_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_i2s *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_i2s_dev_private *dev_private = runtime->private_data;
	struct audio_stream *s = NULL; 
	struct audio_stream *s1 = NULL;

	if (!dev_private) return -1;
	
	s =  &chip->s[dev_private->dev_num][substream->pstr->stream];
	s1 = &chip->s[dev_private->dev_num][substream->pstr->stream^1];
	
	if (s1->active != 1) {
		snd_i2s_set_samplerate(chip, runtime->rate, dev_private->dev_num);
	}
	else if (chip->samplerate[dev_private->dev_num] != runtime->rate) {
		zynq_printk(0, "[zynq_audio] WARNING : The samplerate %d is not set successfully into I2S%d, because the other stream is running under %ld samplerate.\n", runtime->rate, dev_private->dev_num, chip->samplerate[dev_private->dev_num]);
		return -EINVAL;
	}
 	s->dma_pos = bytes_to_frames(runtime, runtime->dma_addr);
	
#ifdef VIRTUAL_AUDIO_DEVICE
	minivosc_prepare(chip->virtual_dev, substream);
#endif
	return 0;
}

static snd_pcm_uframes_t snd_i2s_pointer(struct snd_pcm_substream *substream)
{
	struct snd_i2s *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_i2s_dev_private *dev_private = runtime->private_data;
	struct audio_stream *s = NULL;
	snd_pcm_uframes_t ret = 0;

	if (dev_private == NULL) return 0;
	
	s = &chip->s[dev_private->dev_num][substream->pstr->stream];
	
	ret = audio_get_dma_pos(s);
#ifdef VIRTUAL_AUDIO_DEVICE
	ret = bytes_to_frames(runtime, minivosc_get_pos(chip->virtual_dev));
#endif	
	return ret;
}

static int snd_i2s_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int snd_i2s_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static void snd_i2s_free_private_data(struct snd_pcm_runtime *runtime)
{
	if (runtime->private_data) 	{
		kfree(runtime->private_data);
	}
}

static int snd_i2s_open(struct snd_pcm_substream *substream)
{
	struct snd_i2s *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_i2s_dev_private *dev_private = NULL;
	int stream_id = substream->pstr->stream;
	int err = 0;
	
	if ((err = audio_dma_irq_request(chip)) < 0) {
		return err;
	}
	
	chip->s[MOZART_I2S0][stream_id].stream = substream;
	dev_private = kmalloc(sizeof(*dev_private), GFP_ATOMIC);
	dev_private->dev_num = MOZART_I2S0;
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		dev_private->dma_chn = DMA_CHN_0;
		zynq_printk(1, "[zynq_audio] Opne under playback mode!\n");
#ifndef VIRTUAL_AUDIO_DEVICE 	
		runtime->hw = snd_i2s_playback;
#else
		zynq_printk(1, "[zynq_audio]Use the minivosc_pcm_playback_hw !!\n");
		runtime->hw = minivosc_pcm_playback_hw;
#endif
	}
	else {
		dev_private->dma_chn = DMA_CHN_1;
		zynq_printk(1, "[zynq_audio]Opne under capture mode!\n");
#ifndef VIRTUAL_AUDIO_DEVICE 		
		runtime->hw = snd_i2s_capture;
#else
		zynq_printk(1, "[zynq_audio]Use the minivosc_pcm_capture_hw !!\n");
		runtime->hw = minivosc_pcm_capture_hw;
#endif		
	}

	if ((err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE, &constraints_rates)) < 0)
		return err;
	
	runtime->private_data = dev_private;
	runtime->private_free = snd_i2s_free_private_data;
#ifdef VIRTUAL_AUDIO_DEVICE	
	minivosc_setup_timer(chip->virtual_dev, substream);
#endif
	
	is_first_enable_audio = 1;
	return 0;
}

static int snd_i2s_close(struct snd_pcm_substream *substream)
{
	struct snd_i2s *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_i2s_dev_private *dev_private = runtime->private_data;
	
	audio_dma_irq_free(chip);
	
	if (dev_private != NULL) chip->s[dev_private->dev_num][substream->pstr->stream].stream = NULL;
	
	return 0;
}

static struct snd_pcm_ops snd_i2s0_playback_ops = {
	.open = snd_i2s_open,
	.close = snd_i2s_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_i2s_pcm_hw_params,
	.hw_free = snd_i2s_pcm_hw_free,
	.prepare = snd_i2s_pcm_prepare,
	.trigger = snd_i2s_trigger,
	.pointer = snd_i2s_pointer,
};
static struct snd_pcm_ops snd_i2s0_capture_ops = {
	.open = snd_i2s_open,
	.close = snd_i2s_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_i2s_pcm_hw_params,
	.hw_free = snd_i2s_pcm_hw_free,
	.prepare = snd_i2s_pcm_prepare,
	.trigger = snd_i2s_trigger,
	.pointer = snd_i2s_pointer,
};

/////////////////////////////////////////////////////////////////////////////////////////
/*card resource  initialization*/
static void snd_i2s_audio_init(struct snd_i2s *chip, int device)
{
	/*Set the pre-defined samplerate to I2S controller  and audio codec through I2C*/
	snd_i2s_set_samplerate(chip, chip->samplerate[device], device);
	return;
}
static int snd_i2s_pcm_init(struct snd_i2s *chip, int dev)
{
	struct snd_pcm *pcm;
	int err;

	if((err = snd_pcm_new(chip->card, "PCM1865 PCM", dev, 1, 1, &pcm))<0) {
		zynq_printk(0,"[zynq_audio]Failed to call snd_pcm_new()!!\n");
		return err;
	}
#ifndef VIRTUAL_AUDIO_DEVICE	
	err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, NULL, PRE_ALLOCATE_SIZE, PRE_ALLOCATE_SIZE);
#else
	err = snd_pcm_lib_preallocate_pages_for_all(pcm,
	        SNDRV_DMA_TYPE_CONTINUOUS,
	        snd_dma_continuous_data(GFP_KERNEL),
	        32*48, 32*48); 
#endif	
	if (err < 0) {
		zynq_printk(0,"[zynq_audio]Preallocate memory fail.\n");
		return err;
	}
	switch (dev) {
	case MOZART_I2S0:
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_i2s0_playback_ops);
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_i2s0_capture_ops);
		break;
 	default:
		break;
	}
	pcm->private_data = chip;
	pcm->info_flags = 0;
	strcpy(pcm->name, chip->card->shortname);

	chip->pcm[dev] = pcm;
	chip->samplerate[dev] = 8000; // DEFAULT_SAMPLERATE

	/* init audio dev */
	snd_i2s_audio_init(chip, dev);
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////

//Refence from : http://lxr.free-electrons.com/source/drivers/i2c/i2c-core.c#L2195
static int i2c_clients_probe(struct i2c_adapter * adap, unsigned short addr) {

	int err;
	union i2c_smbus_data dummy;
	
	err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &dummy);
	
	return err >= 0;
}


static int  audio_codec_init(void)
{
	int err = 0;
	int index = 0;
	unsigned int bus_num = (unsigned int)-1;
	unsigned short  probe_addrs =  (unsigned short)-1;
	
	memset(&g_audio_ctx, 0x0, sizeof(struct audio_context));
	
	audio_setup_i2c_adapter(); 
	
	for (index = 0 ; index < MAX_AUDIO_CODEC_NUM; index++)
	{
		if (index == 0)
		{
			bus_num = AUDIO_CH0_I2C_BUS;
			probe_addrs =AUDIO_CH0_I2C_ADDR;
		}
		else if (index == 1) 
		{
			bus_num = AUDIO_CH1_I2C_BUS;
			probe_addrs =AUDIO_CH1_I2C_ADDR;
		}
		else if (index == 2) 
		{
			bus_num = AUDIO_CH2_I2C_BUS;
			probe_addrs =AUDIO_CH2_I2C_ADDR;
		}
		if ((bus_num != (unsigned int) -1) && (probe_addrs != (unsigned short)-1))
		{
			struct i2c_board_info *info = &zynq_audio_codec_info[index].board_info;
			if (zynq_audio_codec_info[index].enable == 1)
			{
				g_audio_ctx.i2c_adapters[index] = audio_get_i2c_adapter_by_bus_num(bus_num);
				if (g_audio_ctx.i2c_adapters[index]) g_audio_ctx.i2c_clients[index]  = i2c_new_probed_device(g_audio_ctx.i2c_adapters[index], info, &probe_addrs,  i2c_clients_probe);
			}
		}
	}
	return err;
}

static int audio_codec_release(void)
{
	int err = 0;
	int index = 0;
	for (index = 0 ; index < MAX_AUDIO_CODEC_NUM; index++)
		if(g_audio_ctx.i2c_clients[index])	i2c_unregister_device(g_audio_ctx.i2c_clients[index]);
	
	audio_rls_i2c_adapter(); 
	
	return err;
}
/////////////////////////////////////////////////////////////////////////////////////////
static void snd_i2s_free(struct snd_card *card)
{
	struct snd_i2s *chip = card->private_data;
#if 1
	audio_dma_irq_free(chip);
#endif
}
static unsigned is_sucess_audio_codec_init = 0;
static int snd_i2s_probe(struct pci_dev *pdev)
{
	int err = 0, i = 0;
	struct snd_card *card = NULL;
	struct snd_i2s *chip = NULL;
	struct snd_kcontrol_new *kctl = NULL;
	
#if 0	
	//1. initialize the pcm1865
	err = audio_codec_init();
	if (err != 0) {
		zynq_printk(0,"[zynq_audio]Failed to audio_codec_init()!!\n");
		goto rls_audio_codec;
	}
	
	is_sucess_audio_codec_init = 1;
#endif
	/* 2. register the soundcard */
	err= snd_card_create(SNDRV_DEFAULT_IDX1, /* use first available id */
						"zynq", /* xid from end of shortname*/
						THIS_MODULE, sizeof(struct snd_i2s), &card);
	if ((!card) || (err))
		return  -ENOMEM;
	
	card->dev = &pdev->dev;
	strncpy(card->driver, pdev->dev.driver->name, sizeof(card->driver));
	sprintf(card->shortname, "pcm1865");
	sprintf(card->longname, "zynq-pcm1865");
	card->private_free = snd_i2s_free;
	
	chip = card->private_data;
	chip->card = card;
	chip->dma_irq = -1;
	mutex_init(&chip->dma_irq_mutex);
#ifdef VIRTUAL_AUDIO_DEVICE
	if (minivosc_init(&chip->virtual_dev) != 0) 
	{
			zynq_printk(0,"[zynq_audio]Failed to minivosc_init()!!\n");
			goto rls_card;
	}
#endif	
	
	g_audio_ctx.chip = chip;
	
	for (i = MOZART_I2S0; i < MOZART_I2S_NUM; i++) 	{
		if ((err = snd_i2s_pcm_init(chip, i)) != 0)
		{
			zynq_printk(0,"[zynq_audio]Failed to snd_i2s_pcm_init()!!\n");
			goto rls_card;
		}
	}
	
	 g_card_info_for_codec.data = (void **)vmalloc(sizeof(struct i2c_client *)*MAX_AUDIO_CODEC_NUM);
	 
	 if ( g_card_info_for_codec.data == NULL) {
		 zynq_printk(0,"[zynq_audio]Cannot allocate memory !!\n");
		goto rls_card;
	}
	 g_card_info_for_codec.data_len = MAX_AUDIO_CODEC_NUM;
	 memset( (struct i2c_client **)g_card_info_for_codec.data, 0x0, sizeof(struct i2c_client *)*MAX_AUDIO_CODEC_NUM);
	 
	for (i = 0; i < MAX_AUDIO_CODEC_NUM; i++)
	{
		 g_card_info_for_codec.data [i]  =  g_audio_ctx.i2c_clients[i];
		 zynq_printk(1, "[zynq_audio](%d) ---> i2c (%p, %p) (%p)\n", i, g_card_info_for_codec.data [i], g_audio_ctx.i2c_clients[i], &g_card_info_for_codec);
	}
	

	/* Mixer controls */
	zynq_printk(1, "[zynq_audio]  set audio mixer!!\n");
	strcpy(card->mixername, "zynq-mixer");
	for(i = 0 ; i < pcm1865_get_mixter_ctrl_num() ; i++) {
		kctl = &pcm1865_audio_mixter_ctrls[i] ;
		kctl->count = 1;
		err = snd_ctl_add(card, snd_ctl_new1(kctl, &g_card_info_for_codec));
		if (err !=  0)
		{	
			zynq_printk(0,"[zynq_audio]Failed to snd_ctl_add()!!\n");
			goto rls_card;
		}
	}
	
	if ((err = snd_card_register(card)) != 0)  
	{
		zynq_printk(0,"[zynq_audio]Failed to  call snd_card_register()!!\n");
		goto rls_card;
	}
	///////////////////////////////////////
	dummy_buffer_size = 1920 * 1080 * 2;
	dummy_buffer = dma_alloc_coherent(NULL, 	dummy_buffer_size , &dummy_buffer_handle, GFP_KERNEL);
	////////////////////////////////////////
	return 0;

rls_card:
	snd_card_free(card);
#if 0	
rls_audio_codec:
	audio_codec_release();
#endif	
	if (g_card_info_for_codec.data  != NULL)
	{
		vfree(g_card_info_for_codec.data );
		g_card_info_for_codec.data  = NULL;
	}	
	///////////////////////////////////////////
	if (dummy_buffer  != NULL)
				dma_free_coherent(NULL, dummy_buffer_size, dummy_buffer, dummy_buffer_handle);
	///////////////////////////////////////////
	return err;
}


static int  snd_i2s_remove(struct pci_dev *pdev)
{
	struct snd_i2s *chip = NULL;
	struct snd_card *card = NULL;
	
	chip = g_audio_ctx.chip;
	
	if (!chip ) goto rls_audio_codec;

	mutex_destroy(&chip->dma_irq_mutex);
	
#ifdef VIRTUAL_AUDIO_DEVICE
	minivosc_rls(chip->virtual_dev);
#endif	
	card = chip->card;
	
	if (card != NULL) snd_card_free(card);

rls_audio_codec:

	if (is_sucess_audio_codec_init) audio_codec_release();
	
	if (g_card_info_for_codec.data  != NULL)
	{
		vfree(g_card_info_for_codec.data );
		g_card_info_for_codec.data  = NULL;
	}
	///////////////////////////////////////////
	if (dummy_buffer  != NULL)
				dma_free_coherent(NULL, dummy_buffer_size, dummy_buffer, dummy_buffer_handle);
	///////////////////////////////////////////
	
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////

int zynq_audio_init (struct pci_dev *pdev)
{
	int ret = 0;
	
	if (zynq_reg_base == NULL) return -1;
	
	zynq_audio_reg_base = (void __iomem *)(zynq_reg_base + 0x0);
	
	if (zynq_audio_reg_base == NULL) return -1;
	
	zynq_printk(0,"[zynq_audio] Zynq Audio Switch Register Base Address : 0x%p\n", zynq_audio_reg_base);
	
	ret =snd_i2s_probe(pdev);
	
	return ret;
}

int zynq_audio_release(struct pci_dev *pdev)
{
	int ret  = 0;
	
	ret =snd_i2s_remove(pdev);
	
	return ret;
}

#ifdef CONFIG_PM_SLEEP
int zynq_audio_suspend(void)
{
	int  i = 0;
	struct i2c_client *i2c = NULL;
	zynq_printk(0,"[zynq_audio]Enter zynq_audio_suspend().\n");
	for (i = 0; i < MAX_AUDIO_CODEC_NUM;  i++){
		i2c =  g_audio_ctx.i2c_clients[i];
		if (i2c != NULL) pcm1865_suspend(i2c);
	}
	snd_power_change_state(g_audio_ctx.chip->card, SNDRV_CTL_POWER_D3hot);
	snd_pcm_suspend_all(g_audio_ctx.chip->pcm[MOZART_I2S0]);
	zynq_printk(0,"[zynq_audio]Leave zynq_audio_suspend().\n");
	return  0;
}
int zynq_audio_resume(void)
{
	int  i = 0;
	struct i2c_client *i2c = NULL;
	zynq_printk(0,"[zynq_audio]Enter zynq_audio_resume().\n");
	for (i = 0; i < MAX_AUDIO_CODEC_NUM;  i++){
		i2c =  g_audio_ctx.i2c_clients[i];
		if (i2c != NULL) pcm1865_suspend(i2c);
	}
	snd_power_change_state(g_audio_ctx.chip->card, SNDRV_CTL_POWER_D0);
	zynq_printk(0,"[zynq_audio]Leave zynq_audio_resume().\n");
	return 0;
}
#endif