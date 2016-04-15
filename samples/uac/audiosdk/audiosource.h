#ifndef __AUDIOSOURCE_H__
#define __AUDIOSOURCE_H__
#include <alsa/asoundlib.h>
#define AUDIO_USB "/sys/class/plcm_usb/f_audio_dual/pcm"
#define SOURCE_TEST_MODE_SYS_PATH "/sys/class/plcm_usb/plcm0/f_audio_dual/audio_tmode"

snd_pcm_t* audio_snd_card_open(char *pcm_name);
int audio_snd_card_close(snd_pcm_t *pcm_handle);
int audio_snd_card_parm_set(snd_pcm_t *pcm_handle);
int audio_snd_card_write(snd_pcm_t *pcm_handle, char* buffers, int size);
int audio_snd_card_write_dummy(snd_pcm_t *pcm_handle, char* buffers, int size);
#endif /* __AUDIOSOURCE_H__ */