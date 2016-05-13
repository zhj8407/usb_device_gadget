#include <math.h>
#include <err.h>
#include "audioutils.h"
#include "audiosource.h"
#include "alsa/asoundlib.h"
#include <sys/time.h>

//#define TEST_MODE
#define DEBUG

static int setSwParams( snd_pcm_t *handle,
                        snd_pcm_uframes_t startFrame,	/* auto start transfer threshold */
                        snd_pcm_uframes_t stopFrame,	/* auto stop transfer threshold */
                        snd_pcm_uframes_t period_size	/* PCM ready size */
                      )
{
    int err;
    snd_pcm_sw_params_t *swparams;

    snd_pcm_sw_params_alloca(&swparams);

    /* get the current swparams */
    err = snd_pcm_sw_params_current(handle, swparams);
    RETERRIFNEG(err);

    /* auto start transfer threshold */
    err = snd_pcm_sw_params_set_start_threshold(handle, swparams, startFrame);
    RETERRIFNEG(err);

    /* auto stop transfer threshold */
    err = snd_pcm_sw_params_set_stop_threshold(handle, swparams, stopFrame);
    RETERRIFNEG(err);

    /* allow the transfer when at least period_size samples can be processed */
    err = snd_pcm_sw_params_set_avail_min(handle, swparams, period_size);
    RETERRIFNEG(err);

    /* write the parameters to the playback device */
    err = snd_pcm_sw_params(handle, swparams);
    RETERRIFNEG(err);

    return 0;
}


static int setHwParams(snd_pcm_t *pcm_handle,
                       snd_pcm_uframes_t buffer_frames,	/* ring buffer length */
                       snd_pcm_uframes_t period_frames,    /* period time */
                       unsigned int channels,
                       unsigned int rate
                      )
{
    int rc;
    unsigned int val;
    int dir;
    snd_pcm_hw_params_t* params;

    /* Allocate a hardware parameters object */
    snd_pcm_hw_params_alloca(&params);

    /* First set to default values */
    rc = snd_pcm_hw_params_any(pcm_handle, params);
    RETERRIFNEG(rc);

    /* Set the desired hardware parameters. */

    /* Set access mode - Interleaved/non-interleaved */
    rc = snd_pcm_hw_params_set_access(pcm_handle, params, AUDIO_ACCESS_MODE);
    RETERRIFNEG(rc);

    /* Set PCM format */
    rc = snd_pcm_hw_params_set_format(pcm_handle, params, AUDIO_PCM_FORMAT);
    RETERRIFNEG(rc);

    /* Set # channels, 1 -> mono, 2 -> stereo */
    rc = snd_pcm_hw_params_set_channels(pcm_handle, params, channels);
    RETERRIFNEG(rc);

    /* Set sampling rate */
    val = rate;
    dir = 0;
    rc = snd_pcm_hw_params_set_rate_near(pcm_handle, params, &val, &dir);
    RETERRIFNEG(rc);
    if( val != rate)
        printf("warnning: rate is not accurate, want %d Hz, got %d Hz\n", rate, val);

    if( buffer_frames)
    {
        /* set the buffer size */
        rc = snd_pcm_hw_params_set_buffer_size_near(pcm_handle, params, &buffer_frames);
        RETERRIFNEG(rc);
    }

    if( period_frames)
    {
        /* set the period size */
        rc = snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &period_frames, &dir);
        RETERRIFNEG(rc);
    }

    /* Write the parameters to the driver */
    rc = snd_pcm_hw_params(pcm_handle, params);
    RETERRIFNEG(rc);

    return 0;
}

static int xrun_recovery(snd_pcm_t *handle, int err)
{
    printf("stream recovery from ...");
    printf("xrun_recovery: %s xrun, err %d",
           (snd_pcm_stream(handle) == SND_PCM_STREAM_PLAYBACK) ? "playback" : "capture", err);

    if (err == -EPIPE)
    {
        if (snd_pcm_stream(handle) == SND_PCM_STREAM_PLAYBACK)
            printf("underrun...");
        else
            printf("overrun...");

        err = snd_pcm_prepare(handle);
        if (err < 0)
        {
            printf("Can't recovery from underrun, prepare failed: %s\n", snd_strerror(err));
            return err;
        }
        printf("ready \n");
        return 0;
    }
    else if (err == -ESTRPIPE)
    {
        int sleepcnt=0;
        printf("suspend...");
        while ((err = snd_pcm_resume(handle)) == -EAGAIN)
        {
            sleep(1);       /* wait until the suspend flag is released */
            if(++sleepcnt > 5)
                break;
        }

        if (err < 0)
        {
            err = snd_pcm_prepare(handle);
            if (err < 0)
            {
                printf("Can't recovery from suspend, prepare failed: %s\n", snd_strerror(err));
                return err;
            }
        }
        printf("ready \n");
        return 0;
    }

    printf("unknown \n");
    return err;
}

snd_pcm_t* audio_snd_card_open(char *pcm_name)
{
    int err;
    snd_pcm_t *handle = NULL;
    err = snd_pcm_open(&handle, pcm_name, SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0)
    {
        printf("[audio_playback] unable to open pcm device: %s\n", snd_strerror(err));
    }
    return handle;
}

int audio_snd_card_close(snd_pcm_t *pcm_handle)
{
    snd_pcm_t *handle = pcm_handle;
    if (handle)
    {
        snd_pcm_abort(handle);
        snd_pcm_close(handle);
    }
}

/*init once at the beginning of writing data to the playback sound card*/
int audio_snd_card_parm_set(snd_pcm_t *pcm_handle)
{
    int err;
    snd_pcm_t *handle = pcm_handle;
    if(handle)
    {
        err = setHwParams(handle, AUDIO_BUFFER_SIZE, AUDIO_PERIOD_SIZE, PCM_NUM_CH, AUDIO_SAMPLING_RATE);
        RETERRIFNEG(err);
        err = setSwParams(handle, AUDIO_PERIOD_SIZE, AUDIO_BUFFER_SIZE*2, AUDIO_PERIOD_SIZE);
        RETERRIFNEG(err);
    }
}

int audio_snd_card_write_dummy(snd_pcm_t *pcm_handle, char* buffers, int size)
{
    printf("[audio_playback] dummy write!\n");
}
/*
SND_PCM_STATE_OPEN
Open
SND_PCM_STATE_SETUP
Setup installed
SND_PCM_STATE_PREPARED
Ready to start
SND_PCM_STATE_RUNNING
Running
SND_PCM_STATE_XRUN
Stopped: underrun (playback) or overrun (capture) detected
SND_PCM_STATE_DRAINING
Draining: running (playback) or stopped (capture)
SND_PCM_STATE_PAUSED
Paused
SND_PCM_STATE_SUSPENDED
Hardware is suspended
SND_PCM_STATE_DISCONNECTED
Hardware is disconnected*/

static int audio_snd_card_trans(snd_pcm_t *pcm_handle, char* buffers, int size)
{
    int rc = 0;
    snd_pcm_t *pPcm = pcm_handle;
    static int pcmStallCnt=0;
    static char silence[AUDIO_PERIOD_SIZE*(PLAY_MIN_PERIOD+1)];

    if ((snd_pcm_state(pPcm) == SND_PCM_STATE_SETUP) ||
            (snd_pcm_state(pPcm) == SND_PCM_STATE_PREPARED) )
    {
        /* fill some zeros to kick start*/
        rc = snd_pcm_writei(pPcm, silence, AUDIO_PERIOD_SIZE*(PLAY_MIN_PERIOD+1));
        if (rc < 0) {
            printf("snd pcm write: %s\n", snd_strerror(rc));
            return -1;
        }
    }

    rc = snd_pcm_wait(pPcm, 2*AUDIO_PERIOD_MS);
    if (rc == 1)	/* data ready */
    {
        rc = snd_pcm_writei(pPcm, buffers, AUDIO_PERIOD_SIZE);
        if (rc < 0)
        {
            /* check if xrun and try recovery */
            if (rc == -EPIPE || rc == -ESTRPIPE)
            {
                rc = xrun_recovery(pPcm, rc);
                if (rc < 0)
                {
                    printf("Can't recovery from underrun for playback or overrun for capture: %s\n", snd_strerror(rc));
                }
            }
            else if (rc == -ENOTTY || rc ==-ENODEV)
            {
                printf("device is physically removed: %s\n", snd_strerror(rc));
                return -1;
            }
            else
            {
                printf("error happened: %s\n", snd_strerror(rc));
                return -1;
            }
        }

        if (pcmStallCnt)
        {
            printf("clear timeout, pcmStallCnt=%d\n", pcmStallCnt);
            pcmStallCnt = 0;
        }
    }
    else	/* 0:timeout, no data, < 0: error */
    {
        /* check if xrun and try recovery */
        if (rc == -EPIPE || rc == -ESTRPIPE)
        {
            rc = xrun_recovery(pPcm, rc);
            if (rc < 0)
            {
                printf("ALSA xrun recovery failed (%s)\n", snd_strerror(rc));
            }
        }
        else
        {
            if (pcmStallCnt == 0)
            {
                printf("timeout0, rc=%s\n", snd_strerror(rc));
            }
            if (pcmStallCnt > PCM_STALL_MAX)
            {
                printf("stopping, too many wait timeout, rc=%d\n", rc);
                return -1;
            }
            ++pcmStallCnt;
            return 0;
        }
    }

    if (rc<0) RETERRIFNEG(rc);
    return rc;
}

int audio_snd_card_write(snd_pcm_t *pcm_handle, char* buffers, int size)
{
    snd_timestamp_t tstamp;
    struct timespec tps, tpsend, tpsdiff;

    unsigned int frame_sec = 0;
    unsigned int frame_usec = 0;
    unsigned int pre_frame_sec = 0;
    unsigned int pre_freme_usec = 0;
    unsigned int read_periods =1;// period_num_of_buffer;
    int rc = 0;
    unsigned int read_buffer_size = 0;
    unsigned int sample_size = 2;
    unsigned int channel_number=1;
    unsigned int frame_size = sample_size * channel_number;
    unsigned int index = 0;
    snd_pcm_t *phandle = pcm_handle;
    char *wbuffers = NULL;
    wbuffers = (char *) malloc(size);
    memset(wbuffers, 0x0, size);
    read_buffer_size = size;
#ifdef DEBUG
    clock_gettime(CLOCK_REALTIME, &tps);
#endif
#ifdef TEST_MODE
    int p_mode;

    audio_snd_sysfs_read(SOURCE_TEST_MODE_SYS_PATH, "%d\n", &p_mode);
    if(p_mode > 0)
    {
        audio_snd_buffer_fill(wbuffers, read_buffer_size, 1);
        /*Here's some example code that makes a 1 kHz sample at a 32 kHz sample rate and with 16 bit samples (that is, not floating point)*/
    } else {
        audio_snd_buffer_endian(wbuffers, buffers, read_buffer_size, FALSE);
    }
#else
    audio_snd_buffer_endian(wbuffers, buffers, read_buffer_size, FALSE);
#endif
    audio_snd_card_trans(phandle, wbuffers, AUDIO_PERIOD_SIZE);

#ifdef DEBUG
    clock_gettime(CLOCK_REALTIME, &tpsend);
    audio_snd_ops_latency(&tps,&tpsend,&tpsdiff);
    printf("[audio_write]wsize =%d %lu s %lu ms\n",read_buffer_size, tpsdiff.tv_sec, tpsdiff.tv_nsec/1000000);
//rc = fwrite(wbuffers, sizeof(char), size, fps);
//if (rc != size) fprintf(stderr, "[audio_capture]short write: wrote %d bytes\n", rc);

#if 0
    unsigned int tmp_frame_sec = 0;
    unsigned int tmp_frame_usec = 0;
    signed int  diff  = 0;
    audio_snd_get_timestamp(phandle, &tstamp);
    frame_sec  = (unsigned int)(tstamp.tv_sec);
    frame_usec = (unsigned int)(tstamp.tv_usec );
    tmp_frame_sec = frame_sec;
    tmp_frame_usec = frame_usec ;
    diff  = ((signed int)tmp_frame_sec  - (signed int)pre_frame_sec) * 1000000+ ((signed int)tmp_frame_usec  - (signed int)pre_freme_usec);
    printf("[audio_write]frame interval = %f (msec) (%u)\n",(float)diff/1000.0, index);
    pre_frame_sec = tmp_frame_sec ;
    pre_freme_usec = tmp_frame_usec;
    index ++;
#endif
#endif
    free(wbuffers);
}
