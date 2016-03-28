//================================================================
//  File Name:          drvUsbHidDevice.h
//  Description:
//
//  synchronizes mute/hold/volumeup/volumedown/flashing ... status with a PC connected via USB
//
//  Copyright (c) 2015 Polycom
//================================================================

#ifndef _DRVUSBHIDDEVICE_H_
#define _DRVUSBHIDDEVICE_H_

#include "stdbool.h"
#include "utils.h"
#include "Queue.h"

//#define DEBUG_HID

#ifdef DEBUG_HID
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF
#endif

#define DEBUG_ERR    printf
#define REPORT2_DATA_HOOK1_MASK         0x1
#define REPORT2_DATA_HOOK2_MASK         0x2
#define REPORT2_DATA_MUTE_MASK          0x4
#define REPORT2_DATA_FLASH_MASK         0x8

typedef unsigned int        UInt32;
typedef void* drvUsbHidHandle;
typedef void (*drvUsbHidDeviceCallStateFuncT)(bool isOffHook, bool isRinging, bool isOnHold, bool isMute);

typedef enum {
    RET_SUCCESS = 0,
    RET_FAIL,
    RET_ERR_MEMORY,
    RET_ERR_PARAMETER,
} errCode;

/***************************************************************************************************
 * Messages
 *--------------------------------------------------------------------------------------------------
 */
typedef struct {
    unsigned char buf[2];
    unsigned int bufLength;
} DrvUsbHidDeviceMsgT;

typedef enum {
    DrvUsbHidDeviceMsgId_sendMsg = 0,
    DrvUsbHidDeviceMsgId_max
} DrvUsbHidDeviceMsgIdT;

typedef struct {
    UInt32 msgTag;
    UInt32 msgType;         /* Mesage Type*/
    DrvUsbHidDeviceMsgT msgParam;
} MsgStruct;

typedef struct {
    Util_THREAD_HANDLE dataRxThread;
    Util_THREAD_HANDLE dataTxThread;
    QueueHandle msgQueueId;
    Util_SEM_HANDLE MsgQueueSem;
    drvUsbHidDeviceCallStateFuncT callBackFun;
} drvUsbHidStruct;

typedef drvUsbHidStruct *drvUsbHidHandler;

errCode drvUsbHidDeviceInit(drvUsbHidHandle *data);
void drvUsbAudioLeafDeviceSendMuteButtonState(int keyState);
void drvUsbAudioLeafDeviceSendControlCommand(unsigned char command);
errCode drvUsbAudioLeafDeviceCallStateCallbackSet(drvUsbHidDeviceCallStateFuncT callStateFunc);

#endif /* #ifndef _DRVUSBHIDDEVICE_H_ */
