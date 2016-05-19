#ifndef HID_LYNC_DISPLAY_H
#define HID_LYNC_DISPLAY_H

#ifdef __cplusplus
extern "C"
{
#endif

// DFSI: Display Field Support Index
#define USBHID_DFSI_LOCAL_NAME         1
#define USBHID_DFSI_LOCAL_STATUS       2
#define USBHID_DFSI_DATE               3
#define USBHID_DFSI_TIME               4
#define USBHID_DFSI_CALL_STATUS        5
#define USBHID_DFSI_OTHER_NAME         6
#define USBHID_DFSI_OTHER_TITIL        7
#define USBHID_DFSI_SUBJECT            8
#define USBHID_DFSI_DURATION           9
#define USBHID_DFSI_NUMBER             10
#define USBHID_DFSI_OTHER_NUMBER       11
#define USBHID_DFSI_CONV_ID            12

// based on usbhidDescriptor.txt
#define USBHID_TEL_REPORT_ID           0x02
#define USBHID_LED_REPORT_ID           0x03

#define USBHID_TEL_REPORT_LEN          2
#define USBHID_LED_REPORT_LEN          2

#define USBHID_TEL_REPORT_BIT_HOOK     0x03
#define USBHID_TEL_REPORT_BIT_MUTE     0x04
#define USBHID_TEL_REPORT_BIT_FLASH    0x08

#define USBHID_LED_REPORT_BIT_RING     0x01
#define USBHID_LED_REPORT_BIT_MUTE     0x02
#define USBHID_LED_REPORT_BIT_HOOK     0x04
#define USBHID_LED_REPORT_BIT_HOLD     0x08


#define USBHID_VENDOR_EXT_REPORT_ID    0x11
#define USBHID_DISP_ATTR_REPORT_ID     0x12
#define USBHID_DISP_CTRL_REPORT_ID     0x13
#define USBHID_CHAR_ATTR_REPORT_ID     0x14
#define USBHID_CHAR_REPORT_ID          0x15
#define USBHID_ICON_REPORT_ID          0x16

#define USBHID_VENDOR_EXT_REPORT_LEN   5
#define USBHID_DISP_ATTR_REPORT_LEN    5
#define USBHID_DISP_CTRL_REPORT_LEN    2
#define USBHID_CHAR_ATTR_REPORT_LEN    3
#define USBHID_CHAR_REPORT_LEN         18
#define USBHID_ICON_REPORT_LEN         3

#define USBHID_VENDOR_EXT_REPORT_VENDOR_ID    0x045E
#define USBHID_VENDOR_EXT_REPORT_VERSION      0x0002

#define USBHID_DISP_ATTR_REPORT_ROWS   0x02
#define USBHID_DISP_ATTR_REPORT_COLS   0x48
//#define USBHID_DISP_ATTR_REPORT_DFSI   0x0822  // USBHID_DFSI_OTHER_NAME, USBHID_DFSI_CONV_ID
#define USBHID_DISP_ATTR_REPORT_DFSI   ( ( 1 << ( USBHID_DFSI_LOCAL_NAME - 1 ) ) | \
        ( 1 << ( USBHID_DFSI_LOCAL_STATUS - 1 ) ) | \
        ( 1 << ( USBHID_DFSI_OTHER_NAME - 1 ) ) | \
        ( 1 << ( USBHID_DFSI_OTHER_TITIL - 1 ) ) )

#define USBHID_DISP_CTRL_BIT_ENABLE    0x01
#define USBHID_DISP_CTRL_BIT_CLEAR     0x02
#define USBHID_DISP_CTRL_BIT_BACKLIGHT 0x04
#define USBHID_DISP_CTRL_BIT_SELECT    0x78

#define USBHID_DISP_CTRL_SELECT_NO_CHANGE    0
#define USBHID_DISP_CTRL_SELECT_IDLE         1
#define USBHID_DISP_CTRL_SELECT_READY        2
#define USBHID_DISP_CTRL_SELECT_OUTGOING     3
#define USBHID_DISP_CTRL_SELECT_INCOMING     4
#define USBHID_DISP_CTRL_SELECT_INCALL       5
#define USBHID_DISP_CTRL_SELECT_HOLDCALL     6
#define USBHID_DISP_CTRL_SELECT_ENDCALL      7

#define USBHID_DISP_CTRL_GET_ENABLE_STATE(value)   \
    ( ( ( value ) & USBHID_DISP_CTRL_BIT_ENABLE ) )

#define USBHID_DISP_CTRL_GET_CLEAR_STATE(value)   \
    ( ( ( value ) & USBHID_DISP_CTRL_BIT_CLEAR ) >> 1 )

#define USBHID_DISP_CTRL_GET_BACKLIGHT_STATE(value)   \
    ( ( ( value ) & USBHID_DISP_CTRL_BIT_BACKLIGHT ) >> 2 )

#define USBHID_DISP_CTRL_GET_SCREEN_SELECT(value)   \
    ( ( ( value ) & USBHID_DISP_CTRL_BIT_SELECT ) >> 3 )

#define USBHID_CHAR_ATTR_BIT_TXT       0x80
#define USBHID_CHAR_BIT_LAST           0x80

#define USBHID_ICON_BIT_PRESENCE       0x0f
#define USBHID_ICON_BIT_MUTE           0x30
#define USBHID_ICON_BIT_MISSEDCALL     0x08
#define USBHID_ICON_BIT_SPEAKER        0x04
#define USBHID_ICON_BIT_VOICEMAIL      0x02
#define USBHID_ICON_BIT_CALLFWD        0x01


void lync_display_process_set_report(unsigned char *report, unsigned int length);

#ifdef __cplusplus
}
#endif

#endif