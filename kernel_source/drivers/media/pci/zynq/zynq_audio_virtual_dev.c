#include <linux/slab.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>

#include "pcm_data.c"

#include "zynq_audio_virtual_dev.h"

#if  0
static int debug = 1;
/* Use our own dbg macro http://www.n1ywb.com/projects/darts/darts-usb/darts-usb.c*/
#undef dbg
#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg); } while (0)
#define dbg2(format, arg...) do { if (debug) printk( ": " format "\n" , ## arg); } while (0)
#endif

#define byte_pos(x)	((x) / HZ)
#define frac_pos(x)	((x) * HZ)

static void minivosc_xfer_buf(struct minivosc_device *mydev, unsigned int count);
static void minivosc_fill_capture_buf(struct minivosc_device *mydev, unsigned int bytes);
static void minivosc_timer_function(unsigned long data);
static void minivosc_internal_timer_start(struct minivosc_device *mydev);
static void minivosc_internal_timer_stop(struct minivosc_device *mydev);
static int load_pcm_data(const char **phandle, unsigned int *buffer_size);

#define CABLE_PLAYBACK	(1 << SNDRV_PCM_STREAM_PLAYBACK)
#define CABLE_CAPTURE	(1 << SNDRV_PCM_STREAM_CAPTURE)
#define CABLE_BOTH	(CABLE_PLAYBACK | CABLE_CAPTURE)

//////////////////////////////////////////////////////////////////////////////////////////////////////
struct snd_pcm_hardware minivosc_pcm_playback_hw = {
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
    .periods_min =		2,
    .periods_max =		16,
    .buffer_bytes_max =	16*4*16*1024,
    .period_bytes_min =	 	4*16*1,
    .period_bytes_max =	4*16*1024,
};
struct snd_pcm_hardware minivosc_pcm_capture_hw = {
    .info =		 (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_NONINTERLEAVED |
    SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
    .formats =	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE| SNDRV_PCM_FMTBIT_U24_LE,
    .rates =		 (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
    SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_KNOT),
    .rate_min =		8000,
    .rate_max =		48000,
    .channels_min =		1,
    .channels_max =	16,
    .periods_min =		2,
    .periods_max =		16,
    .buffer_bytes_max =	16*4*16*1024,
    .period_bytes_min =	 4*16*1,
    .period_bytes_max =	4*16*1024,
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
/*public functions*/

int minivosc_init(struct minivosc_device **phandle)
{

    struct minivosc_device *dev = NULL;
    dev =  kzalloc(sizeof(struct minivosc_device), GFP_KERNEL);

    if (!dev) return -ENOMEM;

    *phandle = dev;

    memset(dev, 0x0, sizeof(struct minivosc_device));

    dev->pcm_buffer_offset = 0;

    if (load_pcm_data(&dev->pcm_buffer, &dev->pcm_buffer_size1) != 0) return -1;

    mutex_init(&dev->cable_lock);

    return 0;
}

int minivosc_rls(struct minivosc_device *handle)
{
    if (handle != NULL) {
        mutex_destroy(&handle->cable_lock);
        kfree(handle);
    }

    return 0;
}

void minivosc_timer_start(struct minivosc_device *mydev, unsigned long jiffies,unsigned int stream_id)
{

    if (!mydev->running) {
        mydev->last_jiffies = jiffies;
        minivosc_internal_timer_start(mydev);
    }
    mydev->running |= (1 << stream_id);
}

void minivosc_timer_stop(struct minivosc_device *mydev,unsigned int stream_id)
{
    mydev->running &= ~(1 << stream_id);
    if (!mydev->running) minivosc_internal_timer_stop(mydev);
}

unsigned int minivosc_get_pos(struct minivosc_device *mydev)
{
    return mydev->buf_pos;
}

void minivosc_pos_update(struct minivosc_device *mydev)
{
    unsigned int last_pos, count;
    unsigned long delta;

    if (!mydev->running)
        return;

    delta = jiffies - mydev->last_jiffies;

    if (!delta)
        return;

    mydev->last_jiffies += delta;

    last_pos = byte_pos(mydev->irq_pos);
    mydev->irq_pos += delta * mydev->pcm_bps;
    count = byte_pos(mydev->irq_pos) - last_pos;

    if (!count)
        return;

#if 0
    {
        unsigned long ms = 0;
        char str[100];
        mydev->ms += jiffies_to_msecs(delta);
        ms = mydev->ms;
        memset(str, 0x0, sizeof(str));
        snprintf(str, sizeof(str), "%02lu:%02lu:%02lu:%03lu ",
                 (ms / (60 * 60 * 1000)) % 24,
                 (ms / (60 * 1000)) % 60,
                 (ms / 1000) % 60,
                 ms % 1000);
        printk(KERN_INFO"[zynq_audio]gg timestamp: %s (%u)", str, mydev->period_size_frac);
    }
#endif

    // FILL BUFFER HERE
    minivosc_xfer_buf(mydev, count);

    if (mydev->irq_pos >= mydev->period_size_frac) {
        mydev->irq_pos %= mydev->period_size_frac;
        mydev->period_update_pending = 1;
    }
}

void minivosc_setup_timer(struct minivosc_device *mydev, struct snd_pcm_substream *ss)
{
    mutex_lock(&mydev->cable_lock);

    mydev->substream = ss; 	//save (system given) substream *ss, in our structure field

    // SETUP THE TIMER HERE:
    setup_timer(&mydev->timer, minivosc_timer_function,
                (unsigned long)mydev);

    mutex_unlock(&mydev->cable_lock);
}


int minivosc_prepare(struct minivosc_device *mydev, struct snd_pcm_substream *ss)
{
    struct snd_pcm_runtime *runtime = ss->runtime;
    unsigned int bps = 0;

    bps = runtime->rate * runtime->channels; // params requested by user app (arecord, audacity)
    bps *= snd_pcm_format_width(runtime->format);
    bps /= 8;
    if (bps <= 0)
        return -EINVAL;

    mydev->buf_pos = 0;
    mydev->pcm_buffer_size = frames_to_bytes(runtime, runtime->buffer_size);

    if (ss->stream == SNDRV_PCM_STREAM_CAPTURE)  memset(runtime->dma_area, 45, mydev->pcm_buffer_size);

    if (!mydev->running) {
        mydev->irq_pos = 0;
        mydev->period_update_pending = 0;
    }

    mutex_lock(&mydev->cable_lock);
    if (!(mydev->valid & ~(1 << ss->stream))) {
        mydev->pcm_bps = bps;
        mydev->pcm_period_size = frames_to_bytes(runtime, runtime->period_size);
        mydev->period_size_frac = frac_pos(mydev->pcm_period_size);

    }
    mydev->valid |= 1 << ss->stream;
    mutex_unlock(&mydev->cable_lock);

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/*private functions*/

static void minivosc_internal_timer_stop(struct minivosc_device *mydev)
{
    del_timer(&mydev->timer);
}
static void minivosc_internal_timer_start(struct minivosc_device *mydev)
{
    unsigned long tick = 0;
    tick = mydev->period_size_frac - mydev->irq_pos;
    tick = (tick + mydev->pcm_bps - 1) / mydev->pcm_bps;
    mydev->timer.expires = jiffies + tick;
    add_timer(&mydev->timer);
}
static void minivosc_timer_function(unsigned long data)
{
    struct minivosc_device *mydev = (struct minivosc_device *)data;

    if (!mydev->running)
        return;

    minivosc_pos_update(mydev);

    minivosc_internal_timer_start(mydev);

    if (mydev->period_update_pending) {
        mydev->period_update_pending = 0;

        if (mydev->running) {
            snd_pcm_period_elapsed(mydev->substream);
        }
    }
}

static void minivosc_xfer_buf(struct minivosc_device *mydev, unsigned int count)
{

    switch (mydev->running) {
        case CABLE_CAPTURE:
            minivosc_fill_capture_buf(mydev, count);
            break;
    }

    if (mydev->running) {
        mydev->buf_pos += count;
        mydev->buf_pos %= mydev->pcm_buffer_size;
    }
}


static int  load_pcm_data(const char **phandle, unsigned int *buffer_size)
{

    *phandle = &pcm_data[0];
    *buffer_size = pcm_data_len;
    return 0;
}

static void minivosc_fill_capture_buf(struct minivosc_device *mydev, unsigned int bytes)
{
    char *dst = mydev->substream->runtime->dma_area;
    unsigned int size1= bytes;
    unsigned int size = 0;
    unsigned int dst_off = mydev->buf_pos; // buf_pos is in bytes, not in samples !

    if (mydev->pcm_buffer_offset + bytes > mydev->pcm_buffer_size1)
        size1 = 	mydev->pcm_buffer_size1-mydev->pcm_buffer_offset;

    size = size1;

    memcpy(dst + dst_off, mydev->pcm_buffer + mydev->pcm_buffer_offset, size);

    mydev->pcm_buffer_offset  += size;

    if (mydev->pcm_buffer_offset >= mydev->pcm_buffer_size1) mydev->pcm_buffer_offset  %= mydev->pcm_buffer_size1;

}