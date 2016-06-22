#ifndef __VISAGE_SHARED_MEM_H__
#define __VISAGE_SHARED_MEM_H__
#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>

#define USB_SHM_VIDEO_IN_BUFFER "/usb_v_in"
#define USB_SHM_VIDEO_IN_MUTEX "/usb_v_in_mutex"
#define USB_SHM_MAX_NUM 1



/*******************Main and related begin*********/
extern uint8_t * allocSharedMem(const char * shm_name, uint32_t shmSize);

extern void freeSharedMem(const char * shm_name, uint8_t * shm_addr, size_t shmSize);
extern pthread_mutex_t * allocSharedMemMutex();
extern void freeSharedMemMutex(pthread_mutex_t * shm_mutex, const char * shm_name);


#endif
