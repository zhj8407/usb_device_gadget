/* Mon May 21 2016 alex */

#ifdef __cplusplus
extern "C" {
#endif
#ifndef _AUDIOLINX_H
#define _AUDIOLINX_H

int audio_linx_open(const char *name);

int audio_linx_get();

int audio_linx_write(int linkid, short *buffers, int size);

int audio_linx_close(int linkid);
#endif

#ifdef __cplusplus
}				/* extern "C" */
#endif

