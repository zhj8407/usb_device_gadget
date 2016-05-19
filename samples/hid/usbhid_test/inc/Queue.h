//=============================================================================
//  File Name: Queue.h
//
//  Description:
//      Message Queue operation implemention.
//
//  Copyright (c) 2015 Polycom
//=============================================================================

#ifndef QUEUE_H
#define QUEUE_H

typedef void * QueueHandle;
typedef void * QueueItem;
typedef int (*compareFunc)(QueueItem item, int target);

QueueHandle UtilCreateQueue(int size, char* nameStr);
int UtilEnQueue(QueueHandle handle, QueueItem item);
int UtilDeQueue(QueueHandle handle, QueueItem *item);
int UtilPeekQueue(QueueHandle handle, QueueItem *item);
int UtilSearchQueue(QueueHandle handle, QueueItem *item, compareFunc function, int target);
int UtilIsQueueEmpty(QueueHandle handle);
int UtilFlushQueue(QueueHandle handle);
int UtilDestroyQueue(QueueHandle handle);

#endif

