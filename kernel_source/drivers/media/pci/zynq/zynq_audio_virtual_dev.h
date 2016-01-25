#ifndef ZYNQ_AUDIO_VIRTUAL_DEV_H
#define ZYNQ_AUDIO_VIRTUAL_DEV_H

#include <linux/time.h>
#include <linux/wait.h>
#include <sound/pcm.h>
#include <linux/mm.h>

extern struct snd_pcm_hardware minivosc_pcm_playback_hw;
extern struct snd_pcm_hardware minivosc_pcm_capture_hw;

struct minivosc_device
{
	const struct minivosc_pcm_ops *timer_ops;
	/*
	* we have only one substream, so all data in this struct
	*/
	/* copied from struct loopback: */
	struct mutex cable_lock;
	/* copied from struct loopback_cable: */
	/* PCM parameters */
	unsigned int pcm_period_size;
	unsigned int pcm_bps;		/* bytes per second */
	/* flags */
	unsigned int valid;
	unsigned int running;
	unsigned int period_update_pending :1;
	/* timer stuff */
	unsigned int irq_pos;		/* fractional IRQ position */
	unsigned int period_size_frac;
	unsigned long last_jiffies;
	struct timer_list timer;
	/* copied from struct loopback_pcm: */
	struct snd_pcm_substream *substream;
	unsigned int pcm_buffer_size;
	unsigned int buf_pos;	/* position in buffer */
	unsigned long ms;
	const char *pcm_buffer;
	unsigned int pcm_buffer_offset;
	unsigned int pcm_buffer_size1;
};

int minivosc_init(struct minivosc_device **phandle);
int minivosc_rls(struct minivosc_device *handle);
void minivosc_timer_start(struct minivosc_device *mydev, unsigned long jiffies, unsigned int stream_id);//be called at 'trigger()'
void minivosc_timer_stop(struct minivosc_device *mydev, unsigned int stream_id); //be called at 'trigger()'
void minivosc_pos_update(struct minivosc_device *mydev);//be called at 'pointer()'
unsigned int minivosc_get_pos(struct minivosc_device *mydev);//be called at 'pointer()'
void minivosc_setup_timer(struct minivosc_device *mydev, struct snd_pcm_substream *ss);//be called at 'open()'
int minivosc_prepare(struct minivosc_device *mydev, struct snd_pcm_substream *ss);//be called at "prepare"

#endif