//================================================================
//  File Name: utils.h
//
//  Description:
//
//  Utility functions for use the driver to improve
//  portability across different operating systems.
//
//  Copyright (c) 2015 Polycom
//================================================================

#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdio.h>
#include <pthread.h>

typedef void (*UTIL_THREAD_FUNC)(void *pData);
typedef void *Util_SEM_HANDLE;
typedef void *Util_THREAD_HANDLE;

typedef enum {
    Util_SUCCESS = 0,
    Util_ERR_RECOVERABLE,
    Util_ERR_UNSUPPORTED_ACCESS_MODE,
    Util_ERR_TIMEOUT,
    Util_ERR_BAD_PARAMETER,
    Util_ERR_RESOURCE_ALLOCATION_FAILED,
    Util_ERR_READ_FAILED,
    Util_ERR_WRITE_FAILED,
    Util_ERR_PCI_SCAN_FAILED,
    Util_ERR_DEVICE_OPEN_FAILED,
    Util_ERR_DEVICE_CLOSE_FAILED,
    Util_ERR_DEVICE_MISSING_RESOURCES,
    Util_ERR_INSUFFICIENT_BUFFERS,
    Util_ERR_SYNC_FAILED,
    Util_ERR_ARRAY_TOO_SMALL,
    Util_ERR_DRIVER_NAME_SET_FAILED,
    Util_ERR_DEBUG_SET_FAILED,
    Util_ERR_LIB_INIT_FAILED,
    Util_ERR_LIB_UNINIT_FAILED,
    Util_ERR_OPERATION_ALREADY_DONE,
    Util_ERR_INTERRUPT_SET_FAILED,
    Util_ERR_NOT_IMPLEMENTED,
    Util_ERR_UNKNOWN,
    Util_ERR_DONE,
    Util_ERR_NOT_FOUND,
    Util_ERR_WRONG_CHIP_TYPE,
    Util_ERR_NOT_SUPPORTED,
    Util_ERR_MEMORY_ALLOCATION_FAILED,
    Util_ERR_INVALID_ADDRESS,
    Util_ERR_INVALID_ID,
    Util_ERR_SDRAM_NOT_INITIALIZED,
    Util_ERR_MAX  // Maximum error value.
} dxErrCode;


/*****************************************************************
 * Function:    UtilSemCreate
 *
 * Description: returns a binary semaphore than can be attached
 *              to an interrupt from either linux or windows
 *
 * output:        newSem - sem to be returned
 *****************************************************************/
void UtilSemCreate(Util_SEM_HANDLE *newSem);

/*****************************************************************
 * Function:    UtilSemGive
 *
 * Description: releases a sem, linux or windows
 *
 * input:       sem - sem to be released
 *****************************************************************/
void UtilSemGive(Util_SEM_HANDLE sem);

/*****************************************************************
 * Function:    UtilSemTake
 *
 * Description: waits on a sem for specified # milliseconds
 *
 * input:       sem - sem to be released
 *
 * returns:     Util_SUCCESS for given, Util_ERR_TIMEOUT for timeout, Util_ERR_UNKNOWN other
 *****************************************************************/
dxErrCode UtilSemTake(Util_SEM_HANDLE sem, int milliSeconds);

/*****************************************************************
 * Function:    UtilSemDestroy
 *
 * Description: destroys a previously created sem
 *
 * input:       sem - sem to be destroyed
 *****************************************************************/
void UtilSemDestroy(Util_SEM_HANDLE *sem);

/*****************************************************************
 * Function:    UtildxErrCodeToString
 *
 * Description: returns a string with text of the error code
 *
 * input:       err - error code to find
 *              max - max number of chars to write to string
 *
 * output:      pString - dxErrCode string
 *****************************************************************/
void UtildxErrCodeToString(dxErrCode err, int max, char *pString);

/*****************************************************************
 * Function:    UtilThreadStart
 *
 * Description: returns a string with text of the error code
 *
 * input:       pFunc - func to start
 *              pData - data to pass to the function
 *
 * output:      phThread - handle to the thread
 *****************************************************************/
dxErrCode UtilThreadStart(Util_THREAD_HANDLE *phThread, UTIL_THREAD_FUNC pFunc, void *pData);

/*****************************************************************
 * Function:    UtilThreadWait
 *
 * Description: waits until a thread exits
 *
 * input:       phThread - pointer to thread handle returned from UtilThreadStart
 *****************************************************************/
void UtilThreadWait(Util_THREAD_HANDLE *phThread);

/*****************************************************************
 * Function:    UtilThreadStop
 *
 * Description: stops a thread
 *
 * input:       phThread - pointer to thread handle returned from UtilThreadStart
 *****************************************************************/
void UtilThreadStop(Util_THREAD_HANDLE *phThread);
/*****************************************************************
 * Function:    UtilThreadExit
 *
 * Description: exit self thread
 *
 * input:       phThread - pointer to thread handle returned from UtilThreadStart
 *****************************************************************/
void UtilThreadExit(Util_THREAD_HANDLE *phThread);

/*****************************************************************
 * Function:    UtilThreadSetPriority
 *
 * Description: sets the thread priority
 *
 * input:       phThread - pointer to thread handle returned from UtilThreadStart
 *              priority - the priority to set
 *****************************************************************/
dxErrCode UtilThreadSetPriority(Util_THREAD_HANDLE hThread, int priority);

/*****************************************************************
 * Function:    UtilSleep
 *
 * Description: stops a thread
 *
 * input:       microSecs
 *****************************************************************/
void UtilSleep(unsigned int microSecs);

/*****************************************************************
 * Function:    UtilGetTime
 *
 * Description: Get the current system time
 *
 * input:       the point to time variable
 *****************************************************************/
unsigned int UtilGetTime();

void *MemAlloc(char *file, int line, int size);
void MemFree(char *file, int line, void* buf);

#endif
