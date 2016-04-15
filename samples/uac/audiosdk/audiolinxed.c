#include <stdio.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h> //for threading , link with lpthread
#include "audiolinxtypes.h"
#include "audiosource.h"

void *connection_handler(void *);
#define NAME "audiosocket"
#define DEVFILE "hw:3,0"
#define DUMP 1
#define DUMP_FILE_NAME "dump"
char file_name[128] = {0x0};
FILE *fps = NULL;

pthread_mutex_t fakeMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t fakeCond = PTHREAD_COND_INITIALIZER;

static void wait(int timeInMs)
{
    struct timespec timeToWait;
    struct timeval now;
    int rt;

    gettimeofday(&now,NULL);

    timeToWait.tv_sec = now.tv_sec;
    timeToWait.tv_nsec = (now.tv_usec+1000UL*timeInMs)*1000UL;

    pthread_mutex_lock(&fakeMutex);
    rt = pthread_cond_timedwait(&fakeCond, &fakeMutex, &timeToWait);
    pthread_mutex_unlock(&fakeMutex);
    printf("\nDone\n");
}

/*pthread_mutex_lock(&mutex);   pthread_cond_signal(&cond);   pthread_mutex_unlock(&mutex);*/

/*
 * This will handle connection for each client
 * */
void *connection_handler(void *arguments)
{
    int rval;
    char buf[PCM_PERIOD_BYTES];
    struct linxed_struct *args = (struct linxed_struct *)arguments;
    //Get the socket descriptor
    int sock = *(int*)args->sock;
    int state=-1;
    snd_pcm_t* handle=NULL;
    snprintf(file_name, sizeof(file_name),"%s-%d.pcm", DUMP_FILE_NAME, sock);
    fps = fopen(file_name, "wb");
    if (fps == NULL)
    {
        fprintf(stderr, "[audio_capture] unable to open file: %s\n", file_name);
        return 0;
    }

    handle = audio_snd_card_open(DEVFILE);
    if(handle == NULL)
        return 0;
    audio_snd_card_parm_set(handle);
    chmod(file_name, S_IRUSR|S_IRGRP|S_IROTH);
#if 0
    while(1) {
        pthread_mutex_lock(&args->mutex);
        pthread_cond_wait(&args->cond, &args->mutex);
        //printf("####%d\n", args->state);
        pthread_mutex_unlock(&args->mutex);
        if(0 == args->state)
            break;
    };
#endif
    printf("open recving stream");

    for (;;) {
        bzero(buf, sizeof(buf));
        //printf("args->state:####%d\n", args->state);
        if ((rval = recv(sock, buf, PCM_PERIOD_BYTES, 0)) < 0)
        {
            printf("reading stream message");
        }
        else if (rval == 0)
        {
            printf("Ending connection\n");
            break;
        }
        else
        {
            pthread_mutex_lock(&args->mutex);
            if(0 == args->state)
                audio_snd_card_write(handle, buf, PCM_PERIOD_BYTES);
            else
            {
                audio_snd_card_write_dummy(handle, buf, PCM_PERIOD_BYTES);
            }
            pthread_mutex_unlock(&args->mutex);
        }
    }

    //Free the socket pointer
    audio_snd_card_close(handle);
    fclose(fps);
    free(args->sock);
    return 0;
}

// the max connection number of the server
#define MAX_CONNECTION_NUMBER 5

/* * Create a server endpoint of a connection. * Returns fd if all OK, <0 on error. */
int unix_socket_listen(const char *servername)
{
    int fd;
    struct sockaddr_un un;
    if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        return(-1);
    }
    int len, rval;
    unlink(servername);               /* in case it already exists */
    memset(&un, 0, sizeof(un));
    un.sun_family = AF_UNIX;
    strcpy(un.sun_path, servername);
    len = offsetof(struct sockaddr_un, sun_path) + strlen(servername);
    /* bind the name to the descriptor */
    if (bind(fd, (struct sockaddr *)&un, len) < 0) {
        rval = -2;
    } else {
        if (listen(fd, MAX_CONNECTION_NUMBER) < 0) {
            rval =  -3;
        } else {
            return fd;
        }
    }
    int err;
    err = errno;
    close(fd);
    errno = err;
    return rval;
}

main()
{
    int sock, msgsock, rval;
    int *new_sock;

    struct linxed_struct args;
    pthread_mutex_init(&args.mutex, NULL);
    pthread_cond_init(&args.cond, NULL);
    args.pid = getpid();
    args.state = 3;

    sock = unix_socket_listen(NAME);
    if(sock<0) {
        printf("Error[%d] when listening...\n",errno);
        return 0;
    }

    audio_snd_event_get(&args);

    for (;;) {
        msgsock = accept(sock, 0, 0);
        printf("accept %d \n", msgsock);
        if (msgsock == -1)
            printf("accept");
        else do {
                printf("accept1\n");
                pthread_t sniffer_thread;
                new_sock = malloc(1);
                *new_sock = msgsock;
                args.sock = new_sock;
                rval = pthread_create( &sniffer_thread, NULL,  connection_handler, (void *)&args);
                printf("accept2\n");
                if(rval < 0)
                {
                    printf("could not create thread");
                    free(new_sock);
                    close(msgsock);
                    break;
                }
                //Now join the thread , so that we dont terminate before the thread
                pthread_join(sniffer_thread, NULL);
            } while (rval > 0);
    }
    pthread_cond_destroy(&fakeCond);
    pthread_mutex_destroy(&fakeMutex);

    close(sock);
    unlink(NAME);
}