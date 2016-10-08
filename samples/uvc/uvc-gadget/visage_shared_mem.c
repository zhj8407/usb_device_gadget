#include "visage_shared_mem.h"
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>

#include <sys/mman.h>
#include <sys/stat.h>      /* For mode constants */
#include <fcntl.h>         /* For O_* constants */

/*******************Shared memory utility*********/
struct shared_mem_t {
    pthread_mutex_t *shm_mutex;
    uint8_t * pSharedMemory;
};


uint8_t * allocSharedMem(const char * shm_name, uint32_t shmSize)
{
    //alloc shared memory
    int shm_fd = 0;
    uint8_t * shm_addr = NULL;
    shm_fd = shm_open(shm_name, O_RDWR | O_CREAT, 0777);

    if (shm_fd == -1) {
        printf("shm_open failed: %d %s\n", errno, strerror(errno));
    }

    int ret = ftruncate(shm_fd, shmSize);

    if (ret < 0) {
        printf("ftrucate failed: %d %s\n", errno, strerror(errno));
    }

    shm_addr = mmap(0, shmSize, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    if (shm_addr == (void *) - 1) {
        fprintf(stderr, "mmap failed:%s\n", strerror(errno));
    }

    return shm_addr;
}



void freeSharedMem(const char * shm_name, uint8_t * shm_addr, size_t shmSize)
{
    printf("try free shared mem: %s at %p of size %u\n", shm_name, shm_addr, shmSize);
    int retv = munmap(shm_addr, shmSize);

    if (retv == -1) {
        printf("munmap failed: %d %s\n", errno, strerror(errno));
        //return;
    }

    retv = shm_unlink(shm_name);

    if (retv == -1) {
        printf("shm_unlink failed: %d %s\n", errno, strerror(errno));
    }
}

pthread_mutex_t * allocSharedMemMutex(const char * shm_mutex_name)
{
    int err;
    pthread_mutexattr_t attr;
    err = pthread_mutexattr_init(&attr);

    if (err) {
        printf("pthread_mutexattr_init failed: %d %s\n", errno, strerror(errno));
        return NULL;
    }

    err = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

    if (err) {
        printf("pthread_mutexattr_setpshared failed: %d %s\n", errno, strerror(errno));
        return NULL;
    }

    pthread_mutex_t *shm_mutex = (pthread_mutex_t *)allocSharedMem(shm_mutex_name, sizeof(pthread_mutex_t));
    err = pthread_mutex_init(shm_mutex, &attr);

    if (err) {
        printf("pthread_mutex_init failed: %d %s\n", errno, strerror(errno));
        return NULL;
    }

    err = pthread_mutexattr_destroy(&attr);

    if (err) {
        printf("pthread_mutexattr_destroy failed: %d %s\n", errno, strerror(errno));
        return NULL;
    }

    return (pthread_mutex_t *)shm_mutex;
}

void freeSharedMemMutex(pthread_mutex_t * shm_mutex, const char * shm_name)
{
    int err;
    err = pthread_mutex_destroy(shm_mutex);

    if (err) {
        printf("pthread_mutex_destroy failed: %d %s\n", errno, strerror(errno));
    }

    freeSharedMem(shm_name, (uint8_t*)shm_mutex, sizeof(pthread_mutex_t));
}


