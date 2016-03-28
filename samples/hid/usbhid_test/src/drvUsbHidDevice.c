//================================================================
//  File Name:          drvUsbHidDevice.c
//  Description:
//
//  synchronizes mute/hold ... status with a PC connected via USB
//
//  Copyright (c) 2015 Polycom
//================================================================

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>

#include "drvUsbHidDevice.h"
#include "Queue.h"

#define DRV_USB_HID_FILENAME            "/dev/hidg0"

#define REPORT2_HEADER              0x2
#define REPORT2_DATA_ALL_MASK           0x7
#define REPORT2_LEN             2

#define REPORT3_HEADER              0x3
#define REPORT3_DATA_ALL_MASK           0xF
#define REPORT3_DATA_RING_MASK          0x1
#define REPORT3_DATA_MUTE_MASK          0x2
#define REPORT3_DATA_HOOK_MASK          0x4
#define REPORT3_DATA_HOLD_MASK          0x8
#define REPORT3_LEN             2

#ifndef INFINITE
#define INFINITE -1
#endif

/*
*******************************************************************************
* Name:         writeData - writes length bytes of buffer to a file
*                   descriptor
*
* Parameters:       fd - file descriptor for writing the message
*                  buffer - pointer to the message buffer
*                  len - buffer length
*
* Returns:      bytes written or error
*
* Notes:        None
*******************************************************************************
*/
static int writeData(int fd, char *buffer, int to_send)
{
    char *local_buffer = buffer;
    //int remaining_bytes_to_write = len;
    // int total_bytes_written = 0;

    if (fd < 0) {
        DEBUG_PRINTF(
            "[%s] file descriptor, %d, is invalid",
            __FUNCTION__,
            fd);
        return -1;
    }

    if (buffer == NULL) {
        DEBUG_PRINTF(
            "[%s] data unavailable",
            __FUNCTION__);
        return -1;
    }

    if (to_send <= 0) {
        DEBUG_PRINTF("[%s] invalid data length, length: %d",
                     __FUNCTION__,
                     to_send);
        return -1;
    }

    /* write the buffer */
    {
        int rc;
        int bytes_written;
        fd_set fdset;
        FD_ZERO(&fdset);
        FD_SET(fd, &fdset);

        rc = select(fd + 1, NULL, &fdset, NULL, NULL);

        if (rc > 0 && FD_ISSET(fd, &fdset)) {
            //DEBUG_PRINTF("local_buffer=%x %x,remaining_bytes_to_write=0x%x\n",local_buffer[0],local_buffer[1],remaining_bytes_to_write);
            bytes_written = write(fd, local_buffer, to_send);

            if (bytes_written != to_send) {
                DEBUG_PRINTF("writeData err\n");
                return -1;
            }

            memset(local_buffer + 1, 0x0, to_send - 1);
            //DEBUG_PRINTF("hold,report= %x %x,to_send=0x%x\n",local_buffer[0],local_buffer[1],to_send);
            bytes_written = write(fd, local_buffer, to_send);

            if (bytes_written != to_send) {
                DEBUG_PRINTF("writeData endup err\n");
                return -1;
            }
        } else if (rc < 0) { /* select() error */
            return errno * -1;
        }
    }
    return to_send;
}

/*
*******************************************************************************
* Name:         drvUsbHidDeviceSendMsgToHost - sends a message
*                           to the host
*
* Parameters:       fd - file descriptor for writing the message
*           ptr - pointer to the message buffer and length
*
* Returns:      void
*
* Notes:        None
*******************************************************************************
*/
static void drvUsbHidDeviceSendMsgToHost(int fd, void *ptr)
{
    DrvUsbHidDeviceMsgT *msgP;

    if (fd < 0) {
        DEBUG_PRINTF(
            "[%s] file descriptor, %d, is invalid",
            __FUNCTION__,
            fd);
        return;
    }

    msgP = (DrvUsbHidDeviceMsgT *)ptr;

    if (msgP != NULL) {
        int bytesWritten = writeData(fd, (char *)msgP->buf, (int)msgP->bufLength);

        if (bytesWritten == (int)msgP->bufLength) {
            if (msgP->bufLength >= 2) {
                DEBUG_PRINTF(
                    "[%s] wrote %u bytes: "
                    "%02hhX %02hhX ...",
                    __FUNCTION__,
                    msgP->bufLength,
                    msgP->buf[0],
                    msgP->buf[1]);
            }
        } else {
            DEBUG_PRINTF(
                "[%s] write failed, ret: %d\n",
                __FUNCTION__,
                bytesWritten);
        }
    } else {
        DEBUG_PRINTF(
            "[%s] message unavailable",
            __FUNCTION__);
    }
}

/*
*******************************************************************************
* Name:         drvUsbHidDeviceTxTask - sends msgs to host
*
* Parameters:       data - ()
*
* Returns:      void
*
* Notes:        None
*******************************************************************************
*/
static void drvUsbHidDeviceTxTask(void *data)
{
    int fd;
    int err = 0;
    QueueItem item;
    drvUsbHidHandler handle = (drvUsbHidHandler)data;
    MsgStruct msg;
    DrvUsbHidDeviceMsgT msgP;

    fd = open(DRV_USB_HID_FILENAME, O_WRONLY);

    if (fd < 0) {
        DEBUG_PRINTF(
            "[%s] failed to open %s, ret: %d, error: %d",
            __FUNCTION__,
            DRV_USB_HID_FILENAME,
            fd,
            errno);
        return;
    }

    while (1) {
        //MsgT rxMsg;
        //==================
        // check msg queue
        //==================

        err = UtilSemTake(&handle->MsgQueueSem, -1);

        if (err != Util_SUCCESS) {
            UtilSleep(5);
            continue;
        }

        UtilPeekQueue(handle->msgQueueId, &item);

        if (item == NULL) {
            UtilSemGive(&handle->MsgQueueSem);
            UtilSleep(5);
            continue;
        }

        memcpy(&msg, item, sizeof(MsgStruct));
        UtilDeQueue(handle->msgQueueId, &item);

        if (item != NULL)
            MemFree(__FILE__, __LINE__, item);

        switch (msg.msgType) {
            case DrvUsbHidDeviceMsgId_sendMsg: {
                drvUsbHidDeviceSendMsgToHost(
                    fd,
                    &msg.msgParam);
                break;
            }

            default:
                break;
        }

        UtilSemGive(&handle->MsgQueueSem);
    }
}

/*
*******************************************************************************
* Name:         drvUsbHidDeviceRxTask - processes msgs
*                           from the host
*
* Parameters:       ptr - unused
*
* Returns:      void
*
* Notes:        None
*******************************************************************************
*/
#define BUF_LEN 512
static void drvUsbHidDeviceRxTask(void *data)
{
    drvUsbHidHandler handle = (drvUsbHidHandler)data;

    const char *filename = NULL;
    int fd = 0;
    char buf[BUF_LEN];
    int cmd_len;
    char report[8];
    int to_send = 8;
    int hold = 0;
    fd_set rfds;
    int retval, i;

    fd = open(DRV_USB_HID_FILENAME, O_RDWR, 0x666);

    if (fd < 0) {
        DEBUG_ERR(
            "[%s] failed to open %s, ret: %d, error: %d",
            __FUNCTION__,
            DRV_USB_HID_FILENAME,
            fd,
            errno);
        return;
    }

    while (1) {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        retval = select(fd + 1, &rfds, NULL, NULL, NULL);

        if (retval == -1 && errno == EINTR)
            continue;

        if (retval < 0) {
            perror("select()");
            return;
        }

        if (FD_ISSET(fd, &rfds)) {
            cmd_len = read(fd, buf, BUF_LEN - 1);
            DEBUG_PRINTF("recv report:");

            for (i = 0; i < cmd_len; i++)
                DEBUG_PRINTF(" %02x", buf[i]);

            DEBUG_PRINTF("\n");
        }

        if (cmd_len == REPORT3_LEN && buf[0] == REPORT3_HEADER) {
            if (handle->callBackFun != NULL) {
                DEBUG_PRINTF("callBackFun\n");
                handle->callBackFun(buf[1] & REPORT3_DATA_HOOK_MASK,
                                    buf[1] & REPORT3_DATA_RING_MASK,
                                    buf[1] & REPORT3_DATA_HOLD_MASK,
                                    buf[1] & REPORT3_DATA_MUTE_MASK);
                DEBUG_PRINTF("callBackFun end\n");
            }
        } else {
            DEBUG_ERR(
                "[%s] unexpected message from host,"
                " length: %d, first byte: 0x%02hhX",
                __FUNCTION__,
                cmd_len,
                buf[0]);
        }
    }

    close(fd);
}

/*
*******************************************************************************
* Name:         drvUsbHidDeviceInit - initialize resources
*               used for communicating with the host over USB
*
* Parameters:       data: drvUsbHidStruct handle
*
* Returns:      RET_SUCCESS on success, RET_FAIL otherwise
*
* Notes:              None
*******************************************************************************
*/
errCode drvUsbHidDeviceInit(drvUsbHidHandle *data)
{
    errCode ret = RET_SUCCESS;
    drvUsbHidHandler handle = NULL;
    handle = (drvUsbHidHandler)MemAlloc(__FILE__, __LINE__, sizeof(drvUsbHidStruct));

    if (NULL == handle) {
        DEBUG_ERR("[%s]: malloc memery failed\n", __FUNCTION__);
        return RET_ERR_MEMORY;
    }

    // Create the msg queue
    handle->msgQueueId = UtilCreateQueue(64, "hidMsgQ");

    UtilSemCreate(&handle->MsgQueueSem);
    UtilSemGive(&handle->MsgQueueSem);

    *data = handle;
    return ret;
}

errCode drvUsbHidBootUp(drvUsbHidHandle data)
{
    drvUsbHidHandler handle = (drvUsbHidHandler)data;

    // start the rx task that will read from the device file
    if (UtilThreadStart(&handle->dataRxThread, drvUsbHidDeviceRxTask, handle) != Util_SUCCESS) {
        //delete msgq
        UtilDestroyQueue(handle->msgQueueId);
        UtilSemDestroy(&handle->MsgQueueSem);
        DEBUG_ERR("pthread_create tHidDevRx failed\n");
        return RET_FAIL;
    }

    // start tx task that will write to the device file
    if (UtilThreadStart(&handle->dataTxThread, drvUsbHidDeviceTxTask, handle) != Util_SUCCESS) {
        //delete msgq
        UtilDestroyQueue(handle->msgQueueId);
        UtilSemDestroy(&handle->MsgQueueSem);
        DEBUG_ERR("pthread_create tHidDevTx failed\n");
        return RET_FAIL;
    }
}


/*
*******************************************************************************
* Name:         drvUsbHidDeviceSendMuteButtonState - send a
*               message to the host with the mutton button
*               state (pressed or released)
*
* Parameters:       keyState - state of the mute button (i.e. pressed,
*                   released, etc)
*
* Returns:      RET_SUCCESS on success, RET_FAIL otherwise
*
* Notes:              None
*******************************************************************************
*/
void drvUsbHidDeviceSendMuteButtonState(int keyState)
{
    return;
}

/*
*******************************************************************************
* Name:         drvUsbHidDeviceSendControlCommand - send USB HID command
*
* Parameters:       data:drvUsbHidHandler command: the HID command
*
* Returns:      RET_SUCCESS on success, RET_FAIL otherwise
*
* Notes:        None
*******************************************************************************
*/
errCode drvUsbHidDeviceSendControlCommand(drvUsbHidHandle data, unsigned char reportHeader, unsigned char reportCmd)
{
    int ret;
    MsgStruct *msg = NULL;
    drvUsbHidHandler handle = (drvUsbHidHandler)data;
    int err = 0;

    msg = (MsgStruct*)MemAlloc(__FILE__, __LINE__, sizeof(MsgStruct));
    memset(msg, 0, sizeof(MsgStruct));

    msg->msgType = DrvUsbHidDeviceMsgId_sendMsg;
    msg->msgParam.buf[0] = reportHeader;
    msg->msgParam.buf[1] = reportCmd;
    msg->msgParam.bufLength = REPORT2_LEN;
    err = UtilSemTake(&handle->MsgQueueSem, -1);

    if (UtilEnQueue(handle->msgQueueId, (QueueItem)msg) == 1) {
        MemFree(__FILE__, __LINE__, msg);
        UtilSemGive(&handle->MsgQueueSem);
        DEBUG_ERR("[%s]:UtilEnQueue FAILED\n", __FUNCTION__);
        return RET_FAIL;
    }

    UtilSemGive(&handle->MsgQueueSem);
    return RET_SUCCESS;
}

/*
*******************************************************************************
* Name:         drvUsbHidDeviceCallbackInstall - set the call state callback.
*
* Parameters:       callStateFunc - callback function pointer
*
* Returns:      RET_SUCCESS on success, RET_FAIL otherwise
*
* Notes:        None
*******************************************************************************
*/
errCode drvUsbHidDeviceCallbackInstall(drvUsbHidHandle data, drvUsbHidDeviceCallStateFuncT callStateFunc)
{
    errCode result = RET_SUCCESS;
    drvUsbHidHandler handle = (drvUsbHidHandler)data;

    if (callStateFunc == NULL) {
        DEBUG_ERR("RET_ERR_PARAMETER\n");
        return RET_ERR_PARAMETER;
    }

    handle->callBackFun = callStateFunc;
    return result;
}

/*
*******************************************************************************
* Name:         drvUsbHidDeviceExit - initialize resources
*               used for communicating with the host over USB
*
* Parameters:       None
*
* Returns:      RET_SUCCESS on success
*
* Notes:        None
*******************************************************************************
*/
errCode drvUsbHidDeviceExit(drvUsbHidHandler *data)
{
    drvUsbHidHandler handle = (drvUsbHidHandler)data;

    UtilThreadStop(&handle->dataRxThread);
    UtilThreadStop(&handle->dataTxThread);
    UtilDestroyQueue(handle->msgQueueId);
    UtilSemDestroy(&handle->MsgQueueSem);
    free(handle);
    return RET_SUCCESS;
}


