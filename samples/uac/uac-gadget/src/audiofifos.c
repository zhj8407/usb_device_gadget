#define _GNU_SOURCE
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <stdlib.h>
#include <limits.h>
#include <pthread.h> //for threading , link with lpthread
#include "audiolinxtypes.h"
#include "audiosource.h"
void *connection_handler(void *);
#define DEVFILE "hw:2,0"
#define DUMP 1
#define DUMP_FILE_NAME "dump"
char file_name[128] = {0x0};
FILE *fps = NULL;


#if 0
static void wait(int timeInMs)
{
    struct timespec timeToWait;
    struct timeval now;
    int rt;

    gettimeofday(&now, NULL);

    timeToWait.tv_sec = now.tv_sec;
    timeToWait.tv_nsec = (now.tv_usec + 1000UL * timeInMs) * 1000UL;

    pthread_mutex_lock(&fakeMutex);
    rt = pthread_cond_timedwait(&fakeCond, &fakeMutex, &timeToWait);
    pthread_mutex_unlock(&fakeMutex);
    printf("\nDone\n");
}
#endif
/*pthread_mutex_lock(&mutex);   pthread_cond_signal(&cond);   pthread_mutex_unlock(&mutex);*/

/*
 * This will handle connection for each client
 * */
extern int UacDevState;
void *connection_handler(void *arguments)
{
    int rval;
    char buf[PCM_PERIOD_BYTES];
    snd_pcm_t* handle = NULL;
    struct linxed_struct *args = (struct linxed_struct *)arguments;
    int sock = *(int*)args->sock;

#if 0    //Get the socket descriptor
    snprintf(file_name, sizeof(file_name), "%s-%d.pcm", DUMP_FILE_NAME, sock);
    fps = fopen(file_name, "wb");

    if (fps == NULL) {
        fprintf(stderr, "[audio_capture] unable to open file: %s\n", file_name);
        return 0;
    }

    chmod(file_name, S_IRUSR | S_IRGRP | S_IROTH);
#endif
    handle = audio_snd_card_open(DEVFILE);

    if (handle == NULL)
        return 0;

    audio_snd_card_parm_set(handle);
#if 0

    while (1) {
        pthread_mutex_lock(&args->mutex);
        pthread_cond_wait(&args->cond, &args->mutex);
        //printf("####%d\n", args->state);
        pthread_mutex_unlock(&args->mutex);

        if (0 == args->state)
            break;
    };

#endif
    printf("open recving stream %d", PCM_PERIOD_BYTES);

    for (;;) {
        bzero(buf, sizeof(buf));

        //printf("args->state:####%d\n", args->state);
        if ((rval = read(sock, buf, PCM_PERIOD_BYTES)) < 0) {
            printf("reading stream message");
        } else if (rval == 0) {
            printf("Ending connection\n");
            usleep(20 * 1000);
        } else {
            pthread_mutex_lock(&args->mutex);
            args->state = UacDevState;

            if (UAC_DEVICE_ACTIVE == args->state)
                audio_snd_card_write(handle, buf, PCM_PERIOD_BYTES);
            else {
                audio_snd_card_write_dummy();
            }

            pthread_mutex_unlock(&args->mutex);
        }
    }

    audio_snd_card_close(handle);
    fclose(fps);
    return 0;
}

static int fifo_create()
{
    int fd = -1;
    struct stat st;
    mkfifo(DEFAULT_FIFO_NAME, O_RDWR);

    if ((fd = open(DEFAULT_FIFO_NAME, O_RDWR)) < 0) {
        fprintf(stderr, ": open: %s\n", strerror(errno));
        return -1;
    }

    if (fstat(fd, &st) < 0) {
        fprintf(stderr, __FILE__": fstat: %s\n", strerror(errno));
        goto fail;
    }

    if (!S_ISFIFO(st.st_mode)) {
        fprintf(stderr, __FILE__": is not a FIFO.\n");
        goto fail;
    }

    fcntl(fd, F_SETPIPE_SZ, FIFO_PERIOD_BYTES);
    return fd;
fail:

    if (fd >= 0)
        close(fd);

    return -1;
}

int audio_fifo_process(struct linxed_struct *args)
{
    int rval;
    //struct linxed_struct *args = (struct linxed_struct *)st;

    pthread_t sniffer_thread;
    //struct sched_param param;
    //pthread_attr_t attr;
    //pthread_attr_init(&attr);
    //param.sched_priority = 21;
    //pthread_attr_setschedpolicy(&attr, SCHED_RR);
    //pthread_attr_setschedparam(&attr, &param);
    //pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    //pthread_cond_init(&args.cond, NULL);

    pthread_mutex_init(&args->mutex, NULL);
    args->pid = getpid();

    args->fifofd = fifo_create();

    if (args->fifofd <= 0) {
        printf("Error[%d] when create...\n", errno);
        return -1;
    }

    args->sock = &(args->fifofd);

    //audio_snd_event_get(&args);
    rval = pthread_create(&sniffer_thread, NULL,  connection_handler, args);

    if (rval < 0) {
        printf("could not create thread");
        return rval;
    }

    //Now join the thread , so that we dont terminate before the thread
#if 0
    //pthread_attr_destroy(&attr);
    //pthread_join(sniffer_thread, NULL);

    close(fifofd);
#endif
    return 0;
}
