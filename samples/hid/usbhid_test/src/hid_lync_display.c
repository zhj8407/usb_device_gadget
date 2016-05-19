#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <iconv.h>

#include "hid_lync_display.h"

#define DEBUG
static void usbhidStatus_hostSetLed(unsigned char status)
{
#ifdef DEBUG
    printf("Set LED status: 0x%02x\n", status);
#endif
}

static void usbhidStatus_hostSetIcon(unsigned char status1, unsigned char status2)
{
#ifdef DEBUG
    printf("Set Icon status: 0x%02x.%02x\n", status1, status2);
#endif

    if ((status2 & USBHID_ICON_BIT_MUTE)) {
        printf("Set Mic: %s\n",
               ((status2 & USBHID_ICON_BIT_MUTE) == USBHID_ICON_BIT_MUTE) ? "MUTE_NOTMUTE" : "MUTE_MUTED");
    }
}

static void usbhidStatus_hostCtrlDisplay(unsigned char status)
{
#ifdef DEBUG
    printf("Set Display status: 0x%02x\n", status);
#endif

    if ((USBHID_DISP_CTRL_GET_ENABLE_STATE(status)) &&
            (USBHID_DISP_CTRL_GET_SCREEN_SELECT(status) == USBHID_DISP_CTRL_SELECT_INCOMING)) {
        printf("Incoming call, Ring start | Hook on | Hold off\n");
    } else if ((USBHID_DISP_CTRL_GET_ENABLE_STATE(status)) &&
               (USBHID_DISP_CTRL_GET_SCREEN_SELECT(status) == USBHID_DISP_CTRL_SELECT_INCALL)) {
        printf("In call, Ring stop | Hook off | Hold off\n");
    } else if ((USBHID_DISP_CTRL_GET_ENABLE_STATE(status)) &&
               (USBHID_DISP_CTRL_GET_SCREEN_SELECT(status) == USBHID_DISP_CTRL_SELECT_ENDCALL)) {
        printf("End call, Ring stop | Hook on | Hold off\n");
    } else if ((USBHID_DISP_CTRL_GET_ENABLE_STATE(status)) &&
               (USBHID_DISP_CTRL_GET_SCREEN_SELECT(status) == USBHID_DISP_CTRL_SELECT_HOLDCALL)) {
        printf("Hold call, Ring stop | Hook off | Hold on\n");
    } else if ((USBHID_DISP_CTRL_GET_ENABLE_STATE(status)) &&
               (USBHID_DISP_CTRL_GET_SCREEN_SELECT(status) == USBHID_DISP_CTRL_SELECT_OUTGOING)) {
        printf("Outgoing call, Ring stop | Hook off | Hold off\n");
    } else if ((USBHID_DISP_CTRL_GET_ENABLE_STATE(status)) &&
               (USBHID_DISP_CTRL_GET_SCREEN_SELECT(status) == USBHID_DISP_CTRL_SELECT_IDLE)) {
        printf("IDLE, Ring stop | Hook on | Hold off\n");
    }
}

static int code_convert(const char *from_charset,
                        const char *to_charset, char *inbuf,
                        size_t inlen, char *outbuf, size_t outlen)
{
    iconv_t cd;
    int rc;

    char **pin = &inbuf;
    char **pout = &outbuf;

    cd = iconv_open(to_charset, from_charset);

    if (cd == NULL) {
        fprintf(stderr, "Can not open the iconv, error: %s(%d)\n",
                strerror(errno), errno);
        return -1;
    }

    memset(outbuf, 0, outlen);

    rc = iconv(cd, pin, &inlen, pout, &outlen);

    if (rc < 0) {
        fprintf(stderr, "Can not do code convert, error: %s(%d)\n",
                strerror(errno), errno);
        return -1;
    }

    iconv_close(cd);

    return 0;
}

// if str == NULL, use predefined string (TBD)
// str is UTF-16BE code
static void usbhidStatus_hostDisplayText(unsigned int dfsi, unsigned char *str, unsigned int len)
{
#ifdef DEBUG
    FILE *fp = NULL;
#endif
    char utf8Buf[2048];

    if (str == NULL) {
        printf("predefined string is not defined yet, string index=%d\n", dfsi);
    } else {
#ifdef DEBUG
        fp = fopen("/root/lync_display_text.txt", "at+");

        if (fp != NULL) {
            fwrite(str, sizeof(unsigned char), len, fp);
            fclose(fp);
        }

#endif
        code_convert("utf-16le", "utf-8", (char *)str, len, utf8Buf, 2048);

        printf("string index=%d: %s\n", dfsi, utf8Buf);
    }
}

#define HID_DEVICE_DISPLAY_FIELD_STRING_LEN   1000

//Global variables
unsigned int            df_index;
unsigned char           df_string[HID_DEVICE_DISPLAY_FIELD_STRING_LEN];
unsigned int            df_len;
unsigned int            df_last;

void lync_display_process_set_report(unsigned char *pReport, unsigned int repLen)
{
    unsigned int repID = pReport[0];

#ifdef DEBUG
    printf("\n+lync_display_process_set_report: repID=0x%x, repLen=%d\n", repID, repLen);
#endif

    switch (repID) {
        case USBHID_LED_REPORT_ID:
            if (repLen < USBHID_LED_REPORT_LEN) {
                printf("?unexpected report length=%d, USBHID_LED_REPORT_LEN=%d \n", repLen, USBHID_LED_REPORT_LEN);
                break;
            }

            usbhidStatus_hostSetLed(pReport[1]);
            break;

        case USBHID_ICON_REPORT_ID:
            if (repLen < USBHID_ICON_REPORT_LEN) {
                printf("?unexpected report length=%d, USBHID_ICON_REPORT_LEN=%d \n", repLen, USBHID_ICON_REPORT_LEN);
                break;
            }

            usbhidStatus_hostSetIcon(pReport[1], pReport[2]);
            break;

        case USBHID_DISP_CTRL_REPORT_ID:
            if (repLen < USBHID_DISP_CTRL_REPORT_LEN) {
                printf("?unexpected report length=%d, USBHID_DISP_CTRL_REPORT_LEN=%d \n", repLen, USBHID_DISP_CTRL_REPORT_LEN);
                break;
            }

            usbhidStatus_hostCtrlDisplay(pReport[1]);
            break;

        case USBHID_CHAR_ATTR_REPORT_ID:
            if (repLen < USBHID_CHAR_ATTR_REPORT_LEN) {
                printf("?unexpected report length=%d, USBHID_CHAR_ATTR_REPORT_LEN=%d \n", repLen, USBHID_CHAR_ATTR_REPORT_LEN);
                break;
            }

            if (pReport[2] & USBHID_CHAR_ATTR_BIT_TXT) {
                // prepared to receive text string via USBHID_CHAR_REPORT_ID
                df_index = pReport[1];
                memset(df_string, 0, HID_DEVICE_DISPLAY_FIELD_STRING_LEN);
                df_len = 0;
                df_last = 0;
            } else {
                usbhidStatus_hostDisplayText(pReport[1], NULL, 0);     // NULL, use predefined string (TBD)
                df_index = 0;
            }

            break;

        case USBHID_CHAR_REPORT_ID:
            if (repLen < USBHID_CHAR_REPORT_LEN) {
                printf("?unexpected report length=%d, USBHID_CHAR_REPORT_LEN=%d \n", repLen, USBHID_CHAR_REPORT_LEN);
                break;
            }

            if (df_last || !df_index) {
                printf("?not expecting char report, index=%d, last=%d, len=%d\n", df_index, df_last, df_len);
                break;
            }

            df_last = pReport[1] & USBHID_CHAR_BIT_LAST;

            memcpy(&df_string[df_len], &pReport[2], USBHID_CHAR_REPORT_LEN - 2);
            df_len += USBHID_CHAR_REPORT_LEN - 2;

            if ((df_len + USBHID_CHAR_REPORT_LEN) >= HID_DEVICE_DISPLAY_FIELD_STRING_LEN) {
                df_len -= USBHID_CHAR_REPORT_LEN;
                printf("?display field string is too long, last few char are overwritten\n");
            }

            if (df_last && df_index) {
                // remove ending 0000
                while ((df_string[df_len - 1] == 0) && (df_string[df_len - 2] == 0) && (df_len >= 2)) {
                    df_len -= 2;
                }

                usbhidStatus_hostDisplayText(df_index, df_string, df_len);
                df_index = 0;
            }

            break;

        default:
            printf("?unsupported repID=0x%x \n", repID);
            break;
    }

#ifdef DEBUG
    printf("\n-lync_display_process_set_report: Done\n\n");
#endif
}
