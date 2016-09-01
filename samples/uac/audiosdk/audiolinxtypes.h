/* Mon May 21 2016 alex */

#ifdef __cplusplus
extern "C" {
#endif
#ifndef _AUDIOLINXTYPES_H
#define _AUDIOLINXTYPE_H

#define AUDIO_SAMPLING_RATE	32000
#define PCM_NUM_CH			1
#define FIFO_NUM_CH			2

#define AUDIO_BUFFER_PERIOD     10
#define AUDIO_PERIOD_PER_SEC	50
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
#define AUDIO_FRAME_SIZE    2
#define PCM_STALL_MAX			100	// 2 second

#define RETERRIFNEG(x)	if(x<0) { printf("error at %s@%d: %s=%d\n", __func__, __LINE__, #x, x ); return x;}
#define DEFAULT_FIFO_NAME "/tmp/musicfifo"

typedef int bool;
#define TRUE  1
#define FALSE 0

struct linxed_struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    pid_t pid;
    int state;
    int* sock;
};

#endif

#ifdef __cplusplus
}				/* extern "C" */
#endif

