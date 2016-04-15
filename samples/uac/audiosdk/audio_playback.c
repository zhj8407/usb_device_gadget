#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <assert.h>
#include <fcntl.h>           /* For O_* constants */

#include "alsa/asoundlib.h"

#define DEFAULT_CHANNELS_NUM 1

#define MAX_SUPPORTED_CHANNEL_NUM 20

#define INTERLEAVED_MODE 0x0
#define NON_INTERLEAVED_MODE 0x1

//The device file of alsa could be checked through : /proc/asound/cards .
#define DEVFILE "hw:2,0"

static int bTerminate = 0;
static int bSleep = 0;

static void sig_kill(int signo)
{
    printf("Receive SIGNAL: %d\n", signo);
    if ((signo == SIGTERM) || (signo == SIGINT)) bTerminate = 1;
    else if (signo == SIGHUP) {
        if (bSleep == 0)
            bSleep = 1;
        else
            bSleep =0;
        //bIssueEvent = 1;
    }
}
#define INTERLEAVED_FILE_NAME "playback"

static unsigned int g_count = (unsigned int)-1;
static unsigned int g_is_splite = (unsigned int)-1;
static unsigned int g_bShowFrameRate = (unsigned int)-1;

#define RETERRIFNEG(x)	if(x<0) { printf("error at %s@%d: %s=%d\n", __func__, __LINE__, #x, x ); return x;}

#define AUDIO_SAMPLING_RATE	32000
#define PCM_NUM_CH			1
#define FIFO_NUM_CH			2

#define AUDIO_BUFFER_PERIOD     10
#define AUDIO_PERIOD_PER_SEC	100
#define AUDIO_PERIOD_MAX_SIZE   (AUDIO_SAMPLING_RATE/AUDIO_PERIOD_PER_SEC)
#define AUDIO_PERIOD_MS         (1000/AUDIO_PERIOD_PER_SEC)
#define AUDIO_PERIOD_SIZE		(AUDIO_SAMPLING_RATE / AUDIO_PERIOD_PER_SEC)

#define PCM_FRAME_BYTES			(PCM_NUM_CH * 2)
#define FIFO_FRAME_BYTES		(FIFO_NUM_CH * 2)

#define PCM_PERIOD_BYTES	(AUDIO_PERIOD_SIZE * PCM_FRAME_BYTES)
#define FIFO_PERIOD_BYTES	(AUDIO_PERIOD_SIZE * FIFO_FRAME_BYTES)
#define AUDIO_BUFFER_SIZE	(AUDIO_PERIOD_SIZE * AUDIO_BUFFER_PERIOD)
#define PLAY_MIN_PERIOD         3
#define AUDIO_ACCESS_MODE       SND_PCM_ACCESS_RW_INTERLEAVED
#define AUDIO_PCM_FORMAT        SND_PCM_FORMAT_S16_LE

int main(int argc, char* argv[]) {
    int rc;
    FILE *fps = NULL;

    short *buffers = NULL;
    int size = PCM_PERIOD_BYTES;

    buffers = (short *) malloc(size);
    memset(buffers, 0x0, size);

    signal(SIGTERM, sig_kill);
    signal(SIGINT, sig_kill);
    signal(SIGHUP, sig_kill);

    fps = fopen ( "/root/playback.pcm" , "rb" );
    int linkid;
    linkid = audio_linx_open("audiosocket");
    if(linkid<0)
        goto exit;

    while(fps!=NULL && !feof(fps))
    {
        if (bTerminate == 1) goto exit;
        if(fps!=NULL)
            rc = fread(buffers, sizeof(short), AUDIO_PERIOD_SIZE, fps);
        RETERRIFNEG(rc);
        /*mono channel 16bit 32kHz sample period 10ms*/
        audio_linx_write(linkid,buffers,PCM_PERIOD_BYTES);
        //printf("[audio_playback]write!%s \n", __TIME__);
        usleep(9*1000);
    }
    audio_linx_close(linkid);
exit:
    printf("[audio_playback]The playback loop is exit!!\n");
    if (fps != NULL) fclose(fps);
    free(buffers);
    return 0;
}