#ifndef __AUDIOUTILS_H__
#define __AUDIOUTILS_H___

#include <alsa/asoundlib.h>
#include <time.h>
#include "audiolinxtypes.h"

typedef int bool;
//int audio_snd_card_find(const char *name, int card);
int audio_snd_card_find(int card);

void audio_snd_channel_adjust(short *buf, int inchn, int outchn);
int audio_snd_sysfs_read(const char* filename, const char* format, int *value);
void audio_snd_get_timestamp(snd_pcm_t *handle, snd_timestamp_t *timestamp);
/*fill with 1 kHz sample at a 32 kHz sample rate and with 16 bit samples*/
int audio_snd_buffer_fill(char* buf, int size);

int audio_snd_buffer_endian(char* dstbuf, char* srcbuf, int size, bool swap);
int audio_snd_ops_latency(struct timespec *start, struct timespec *end, struct timespec *diff);

#endif /* __AUDIOUTILS_H__ */
