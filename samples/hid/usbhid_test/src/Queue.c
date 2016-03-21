//=============================================================================
//  File Name: Queue.c
//
//  Description:
//      Message Queue operation implemention.
//
//  Copyright (c) 2015 Polycom
//=============================================================================

#include "malloc.h"
#include "stdio.h"
#include "memory.h"
#include "string.h"
#include "Queue.h"
#include "utils.h"

#ifndef NULL
#define NULL 0
#endif

//#define DEBUG_LOG

typedef struct _QUEUE_STRUCT {
    int read;
    int write;
    int totalSize;
    QueueItem * pBuf;
    char name[32];
} QUEUE_STRUCT;
QueueHandle UtilCreateQueue(int size, char* nameStr)
{
    QUEUE_STRUCT *queue = (QUEUE_STRUCT *)MemAlloc(__FILE__, __LINE__,  sizeof(QUEUE_STRUCT));

    if (queue == NULL)
        return NULL;

    memset(queue, 0, sizeof(QUEUE_STRUCT));

    queue->pBuf = (QueueItem *)MemAlloc(__FILE__, __LINE__,  size * sizeof(QueueItem));

    if (queue->pBuf == NULL)
        return NULL;

    memset(queue->pBuf, 0, size * sizeof(QueueItem));

    queue->totalSize = size;
    queue->read = -1;
    queue->write = 0;

    if (nameStr != NULL)
        strncpy(queue->name, nameStr, 32);
    else
        strcpy(queue->name, "Queue");

    return (QueueHandle)queue;
}

// add an item into queue
// return value
//      0: success
//      1: fail
int UtilEnQueue(QueueHandle handle, QueueItem item)
{
    QUEUE_STRUCT *q = (QUEUE_STRUCT *)handle;

    // buffer full
    if ((q->write + 1) % (q->totalSize) == q->read) {
        printf("\n%s error!!!: buffer full, size %d\n\n", q->name, q->totalSize);
        return 1;
    }

    *(q->pBuf + q->write) = item;

    if (q->read == -1)
        q->read = q->write;

    q->write = (q->write + 1) % (q->totalSize);

#ifdef DEBUG_LOG
    printf("enQ %s r:%d,w:%d\n", q->name, q->read, q->write);
#endif
    return 0;

}

// get value of first item and remove it from queue
// return value
//      0: success
//      1: fail
int UtilDeQueue(QueueHandle handle, QueueItem *item)
{
    QUEUE_STRUCT *q = (QUEUE_STRUCT *)handle;

    // buffer empty
    if (q->read == -1) {
        //printf("Queue error!!! buffer empty\n");
        *item = NULL;
        return 1;
    }

    *item = *(q->pBuf + q->read);
    *(q->pBuf + q->read) = NULL;

    q->read = (q->read + 1) % (q->totalSize);

    if (q->read == q->write)
        q->read = -1;

#ifdef DEBUG_LOG
    printf("deQ %s r:%d,w:%d\n", q->name, q->read, q->write);
#endif
    return 0;

}

// get value of first item, do not remove it from queue
// return value
//      0: success
//      1: fail
int UtilPeekQueue(QueueHandle handle, QueueItem *item)
{
    QUEUE_STRUCT *q = (QUEUE_STRUCT *)handle;

#ifdef DEBUG_LOG
    printf("pkQ %s r:%d,w:%d\n", q->name, q->read, q->write);
#endif

    // buffer empty
    if (q->read == -1) {
        *item = 0;
        return 1;
    }

    *item = *(q->pBuf + q->read);

    return 0;


}

// Search item by compareFunc. If found, put it's value in "item" and remove it from queue
// Return value
//      0: success
//      1: fail
int UtilSearchQueue(QueueHandle handle, QueueItem *item, compareFunc function, int target)
{
    QUEUE_STRUCT *q = (QUEUE_STRUCT *)handle;
    int found = 0;
    int found_pos = -1;
    int i = q->read;
    int end = q->write;
    int next;

    // buffer empty
    if (q->read == -1)
        return 1;

    while (i != end) {
        if (function(*(q->pBuf + i), target) == 0) {
            found = 1;
            found_pos = i;
            break;
        }

        i = (i + 1) % (q->totalSize);
    }

    if (found == 0)
        return 1;

    *item = *(q->pBuf + found_pos);
    *(q->pBuf + found_pos) = NULL;

    i = found_pos;

    // remove found item from buf
    next = (i + 1) % (q->totalSize);

    if (next == end) { // this item is last one
        q->write = i;
    }


    while (next != end) {
        *(q->pBuf + i) = *(q->pBuf + next);

        i = (i + 1) % (q->totalSize);
        next = (i + 1) % (q->totalSize);
    }

    q->write = i;

    if (q->read == q->write) // buffer is empty
        q->read = -1;

    return 0;

}

//
//return value:
//  0: queue is not empty
//  1: queue is empty
//
int UtilIsQueueEmpty(QueueHandle handle)
{
    QUEUE_STRUCT *q = (QUEUE_STRUCT *)handle;

    if (q->read == -1)
        return 1;
    else
        return 0;
}

// Remove all items from queue and free memory of each queue item
// Return value
//      0: success
//      1: fail

int UtilFlushQueue(QueueHandle handle)
{
    QUEUE_STRUCT *q = (QUEUE_STRUCT *)handle;
    int i = q->read;

    if (q->read == -1)
        return 0;

    while (i != q->write) {
        if (*(q->pBuf + i) != NULL) {
            MemFree(__FILE__, __LINE__, *(q->pBuf + i));
            *(q->pBuf + i) = NULL;
        }

        i = (i + 1) % (q->totalSize);
    }

    q->read = -1;
    q->write = 0;

    return 0;

}
int UtilDestroyQueue(QueueHandle handle)
{
    QUEUE_STRUCT *q = (QUEUE_STRUCT *)handle;

    MemFree(__FILE__, __LINE__, q->pBuf);
    MemFree(__FILE__, __LINE__, q);

    return 0;
}



