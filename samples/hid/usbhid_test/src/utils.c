//================================================================
//  File Name: utils.c
//
//  Description:
//
//  Utility functions for use the driver to improve
//  portability across different operating systems.
//
//  Copyright (c) 2015 Polycom
//================================================================

#include "utils.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <semaphore.h>
#include <errno.h>

#define _MULTI_THREADED
#define UTIL_ERR printf

typedef void* HANDLE;
typedef struct {
    void *threadHandle;
    UTIL_THREAD_FUNC pFunc;
    void *pData;
} UtilThreadInfo;

static void *UtilStartUnixThread(void *pData)
{
    UtilThreadInfo *info = (UtilThreadInfo *)pData;
    (*info->pFunc)(info->pData);
    return NULL;
}

/*****************************************************************
 * Function:    UtilSemCreate
 *****************************************************************/
void UtilSemCreate(Util_SEM_HANDLE *newSem)
{
    HANDLE mySem;
    sem_t *pSemt = (sem_t*)malloc(sizeof(sem_t));
    sem_init(pSemt, 0, 0);
    mySem = (void *)pSemt;
    *newSem = mySem;
}

/*****************************************************************
 * Function:    UtilSemGive
 *****************************************************************/
void UtilSemGive(Util_SEM_HANDLE sem)
{
    sem_post((sem_t *)sem);
}

/*****************************************************************
 * Function:    UtilSemTake
 *****************************************************************/
dxErrCode UtilSemTake(Util_SEM_HANDLE sem, int milliSeconds)
{
    //wait for the interrupt
    struct timespec ts;        //timeout check for sem
    int err;
    sem_t *pSem;

    pSem  = (sem_t *)sem;

    if (milliSeconds == -1) { //INFINITE
        sem_wait((sem_t *)sem);
    } else {
        //calculate the new sem timeout time
        if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
            UTIL_ERR("ERROR: cannot get clock time\n");
            return Util_ERR_UNKNOWN;
        }

        ts.tv_sec += (milliSeconds / 1000) + 1;

        err = sem_timedwait(pSem, &ts);

        if (err) {
            if (ETIMEDOUT == errno) {
                // UTIL_ERR("ERROR: timed out waiting for semaphore\n");
                return Util_ERR_TIMEOUT;
            } else {
                UTIL_ERR("ERROR: unknown error waiting for semaphore\n");
                return Util_ERR_UNKNOWN;
            }
        }
    }

    return Util_SUCCESS;
}

/*****************************************************************
 * Function:    UtilSemDestroy
 *****************************************************************/
void UtilSemDestroy(Util_SEM_HANDLE *sem)
{

    HANDLE *hSem = *sem;
    sem_t *TSinSem = (sem_t *)hSem;
    sem_destroy(TSinSem);
    *sem = NULL;
}
/*****************************************************************
 * Function:    UtilThreadStart
 *****************************************************************/
dxErrCode UtilThreadStart(Util_THREAD_HANDLE *phThread, UTIL_THREAD_FUNC pFunc, void *pData)
{

    UtilThreadInfo *info = MemAlloc(__FILE__, __LINE__, (sizeof(UtilThreadInfo)));
    int err;
    info->threadHandle = MemAlloc(__FILE__, __LINE__, (sizeof(pthread_t)));
    info->pFunc = pFunc;
    info->pData = pData;
    err = pthread_create((pthread_t *)info->threadHandle, NULL, &UtilStartUnixThread, info);

    if (err) {
        return Util_ERR_UNKNOWN;
    }

    *phThread = (void *)info;
    return Util_SUCCESS;
}
/*****************************************************************
 * Function:    UtilThreadWait
 *****************************************************************/
void UtilThreadWait(Util_THREAD_HANDLE *phThread)
{

    UtilThreadInfo *info = (UtilThreadInfo *)(*phThread);
    void* exit_status;
    pthread_join(*((pthread_t *)info->threadHandle), &exit_status);
    MemFree(__FILE__, __LINE__, info->threadHandle);
    MemFree(__FILE__, __LINE__, info);
    *phThread = NULL;
}

/*****************************************************************
 * Function:    UtilThreadStop
 *****************************************************************/
void UtilThreadStop(Util_THREAD_HANDLE *phThread)
{
    UtilThreadInfo *info = (UtilThreadInfo *)(*phThread);
    pthread_cancel(*((pthread_t *)info->threadHandle));
    MemFree(__FILE__, __LINE__, info->threadHandle);
    MemFree(__FILE__, __LINE__, info);
    *phThread = NULL;
}

/*****************************************************************
 * Function:    UtilThreadExit
 *****************************************************************/
void UtilThreadExit(Util_THREAD_HANDLE *phThread)
{
    pthread_exit((void*)phThread);
}

/*****************************************************************
 * Function:    UtilThreadSetPriority
 *****************************************************************/
dxErrCode UtilThreadSetPriority(Util_THREAD_HANDLE hThread, int priority)
{
    return Util_ERR_NOT_IMPLEMENTED;
}

/*****************************************************************
 * Function:    UtilSleep
 *****************************************************************/
void UtilSleep(unsigned int milliSecs)
{

    usleep(milliSecs * 1000);
}

/*****************************************************************
 * Function:    UtilGetTime
 *****************************************************************/
unsigned int UtilGetTime()
{
    //dxErrCode ret = Util_SUCCESS;
    unsigned int currSysTime = 0;
    struct timespec ts;

    if (clock_gettime(CLOCK_REALTIME, &ts) == ESRCH) {
        return -1;
    } else {
        currSysTime = ts.tv_sec * 1000 + ts.tv_nsec / 1000000L; //get the system millisec
    }

    return currSysTime;
}

void *MemAlloc(char *file, int line, int size)
{
    void *handle;

    //printf("In file %s line %d malloc  %d bytes\n", file, line, size);

    handle = malloc(size);

//    printf("malloc handle=0x%x In line %d malloc  %d bytes\n", handle, line, size);

    return handle;
}

void MemFree(char *file, int line, void* buf)
{
//    printf("In line %d free 0x%x\n", line, buf);

    free(buf);
}



