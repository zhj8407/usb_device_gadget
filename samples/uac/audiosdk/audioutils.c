#include <math.h>
#include "audioutils.h"

int audio_snd_vol_set()
{
}

int audio_snd_vol_get()
{
}

int audio_snd_card_get()
{

}

int audio_snd_card_find(const char *name, int card)
{
    snd_ctl_t *handle;
    snd_ctl_card_info_t *info;
    int err;

    snd_ctl_card_info_alloca(&info);

    /* card == -1, return first card.  Return -1 if no more card */
    if (snd_card_next(&card) < 0 || card < 0) {
        return card;
    }

    while (card >= 0) {
        char name[32];
        sprintf(name, "hw:%d", card);
        if ((err = snd_ctl_open(&handle, name, 0)) < 0) {
            printf("control open (%i): %s\n", card, snd_strerror(err));
            goto next_card;
        }
        if ((err = snd_ctl_card_info(handle, info)) < 0) {
            printf("control hardware info (%i): %s\n", card, snd_strerror(err));
            snd_ctl_close(handle);
            goto next_card;
        }
        snd_ctl_close(handle);

        err = strncmp( name, snd_ctl_card_info_get_driver(info), strlen(name));
        if( err == 0) {
            break;
        }
next_card:
        if (snd_card_next(&card) < 0) {
            printf("snd_card_next\n");
            break;
        }
    }

    return card;
}
/*channel adjust, support short(16bit) pcm data buffer*/
void audio_snd_channel_adjust(short *buf, int inchn, int outchn)
{
    int i=0;
    short *inptr=NULL;
    short *outptr=NULL;
    if(inchn == outchn)
        return;

    if(inchn == 2 && outchn == 1)
    {
        inptr = buf;
        outptr = buf;
        for(i=0; i<AUDIO_PERIOD_SIZE; i++)
        {
            *outptr = *inptr;
            inptr += 2;
            outptr ++;
        }
        return;
    }

    if(inchn == 1 && outchn == 2)
    {
        inptr = &buf[AUDIO_PERIOD_SIZE-1];
        outptr = &buf[(AUDIO_PERIOD_SIZE*2)-1];
        for(i=0; i<AUDIO_PERIOD_SIZE; i++)
        {
            *outptr = *inptr;
            outptr--;
            *outptr = *inptr;
            outptr--;
            inptr--;
        }
        return;
    }

    printf("unsupported: inchn=%d, outchn=%d\n", inchn, outchn);
}

#define TWO_PI (3.1415926 * 2)
int audio_snd_buffer_fill(char* buf, int size, int type)
{
    int i;
    int sampleRate = 32000;
    float rate = 1;
    short amplitude = 128;
    float frequency = 1000;
    int n=0;
    int sample=0;
    int frame_size=AUDIO_FRAME_SIZE;

    /*Here's some example code that makes a 1 kHz sample at a 32 kHz sample rate and with 16 bit samples (that is, not floating point)*/
    for (i = 0, n=0; i < size; n++) {
        sample = rate*amplitude * sin((TWO_PI * n * frequency) / sampleRate);
        buf[i] = (char)(sample&0xff);
        buf[i+1]= (char)(sample>>8&0xff);
        i+=frame_size;
    }
}

int audio_snd_buffer_endian(char* dstbuf, char* srcbuf, int size, bool swap)
{
    int i,j;
    int frame_size=AUDIO_FRAME_SIZE;
    if(swap)
    {
        for (i = 0, j = 0; i < size;) {
            dstbuf[j+1] = srcbuf[i];
            dstbuf[j]= srcbuf[i+1];
            i+=frame_size;
            j+=frame_size;
        }
    }
    else
    {
        for (i = 0, j = 0; i < size;) {
            dstbuf[j] = srcbuf[i];
            dstbuf[j+1]= srcbuf[i+1];
            i+=frame_size;
            j+=frame_size;
        }
    }
}

int audio_snd_sysfs_read(const char* filename, const char* format, int *value)
{
    int fd = 0;
    char buf[8];
    int len;

    fd = open(filename, O_RDONLY);

    if (fd < 0) {
        fprintf(stderr, "Can not open the file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        return -1;
    }

    len = read(fd, buf, sizeof(buf));

    if (len <= 0) {
        fprintf(stderr, "Can not read data from file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        close(fd);
        return -1;
    }

    len = sscanf(buf, format, value);

    if (len <= 0) {
        fprintf(stderr, "Can not parse the value from %s\n", filename);
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

void audio_snd_get_timestamp(snd_pcm_t *handle, snd_timestamp_t *timestamp)
{
    int i_snd_rc = -1;

    snd_pcm_status_t *p_status;

    /* Get the status */
    snd_pcm_status_alloca(&p_status);
    i_snd_rc = snd_pcm_status(handle, p_status );
    if( i_snd_rc < 0 )
    {
        printf("[audio_capture]Stream status error: %s\n", snd_strerror(i_snd_rc));
        return;
    }
    /* Handle buffer underruns and get the status again */
    if( snd_pcm_status_get_state( p_status ) == SND_PCM_STATE_XRUN )
    {
        /* Prepare the device */
        i_snd_rc = snd_pcm_prepare( handle );
        if( i_snd_rc )
        {
            printf("[audio_capture]cannot recover from buffer underrun\n");
            return;
        }
        printf("[audio_capture]recovered from buffer underrun\n" );
        /* Get the new status */
        i_snd_rc = snd_pcm_status( handle, p_status );
        if( i_snd_rc < 0 )
        {
            printf("[audio_capture]cannot get device status after recovery: %s\n", snd_strerror(i_snd_rc) );
            return;
        }
    }
    else
    {
        snd_pcm_status_get_tstamp(p_status, timestamp);
    }
    return;
}

/*clock_gettime used to get latency, diff(time1,time2).tv_sec diff(time1,time2).tv_nsec*/
int audio_snd_ops_latency(struct timespec *start, struct timespec *end, struct timespec *diff)
{
    if(!diff)
        return -1;
    if ((end->tv_nsec-start->tv_nsec)<0) {
        diff->tv_sec = end->tv_sec-start->tv_sec-1;
        diff->tv_nsec = 1000000000+end->tv_nsec-start->tv_nsec;
    } else {
        diff->tv_sec = end->tv_sec-start->tv_sec;
        diff->tv_nsec = end->tv_nsec-start->tv_nsec;
    }
    return 0;
}


