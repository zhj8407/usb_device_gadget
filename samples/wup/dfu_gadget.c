#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/statvfs.h>

#include <openssl/md5.h>

#include "f_dfu.h"

#define DEBUG_PRINT             0
#define DEBUG_CALCULATE_SPEED   1

#define DFU_BUFFER_SIZE         4096

#define MAX_FILE_NAME_LENGTH    256

#define MD5_SUM_LENGTH          40

#define OUTPUT_DEV_NULL    "/dev/null"

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

/* WUP specific requests. */
#define USB_REQ_WUP_SET_DNLOAD_INFO     0x0001
#define USB_REQ_WUP_GETSTATUS           0x0002
#define USB_REQ_WUP_CLRSTATUS           0x0003
#define USB_REQ_WUP_GETSTATE            0x0005
#define USB_REQ_WUP_ABORT               0x0006
#define USB_REQ_WUP_SYNC                0x0007
#define USB_REQ_WUP_INT_CHECK           0x0008
#define USB_REQ_WUP_START_UPDATE        0x0009

#define WUP_TEMPFILE                    "/home/ubuntu/wup_upgrade_img.tar"

#define FIRMWARE_IMAGE_LOCATION         "/home/ubuntu"

struct wup_status {
    uint8_t     bStatus;
    uint8_t     bState;
    union {
        uint32_t    dwWrittenBytes;
        uint8_t     bReserved[6];
    } u;
} __attribute__((packed));

struct wup_dnload_info {
    int8_t      sSwVersion[32];
    uint32_t    dwImageSize;
    uint32_t    dwSyncBlockSize;
    uint8_t     bForced;
    uint8_t     bReserved[23];
} __attribute__((packed));

#define WUP_STATUS_OK                   0x00
#define WUP_STATUS_errSTATE             0x01
#define WUP_STATUS_errCHECK             0x02
#define WUP_STATUS_errTARGET            0x03
#define WUP_STATUS_errFILE              0x04
#define WUP_STATUS_errWRITE             0x05
#define WUP_STATUS_errVERIFY            0x06
#define WUP_STATUS_errNOTDONE           0x07
#define WUP_STATUS_errINVAL             0x08
#define WUP_STATUS_errTRANS             0x09
#define WUP_STATUS_errUNKNOWN           0x0A

enum wup_state {
    WUP_STATE_dfuDETACHED       = 0,
    WUP_STATE_dfuIDLE           = 1,
    WUP_STATE_dfuDNLOAD_IDLE    = 2,
    WUP_STATE_dfuDNLOAD_BUSY    = 3,
    WUP_STATE_dfuDNLOAD_SYNC    = 4,
    WUP_STATE_dfuDNLOAD_VERY    = 5,
    WUP_STATE_dfuUPDATE_BUSY    = 6,
    WUP_STATE_dfuERROR          = 7,
};

struct wup_device {
    int fd;
    int file_fd;
    char image_file[MAX_FILE_NAME_LENGTH];
    char image_md5[40];
    unsigned long image_size;
    unsigned long sync_block_size;
    unsigned long image_written_bytes;

    MD5_CTX md5_ctx;

    unsigned char status;
    unsigned char state;

    unsigned char f_force_updated;
    char image_sw_version[32];
};

static int wup_open_read_pipe(struct wup_device *dev);
static void wup_close_read_pipe(struct wup_device *dev);

#ifdef DEBUG_CALCULATE_SPEED
static long start_time;

long get_current_time()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#endif

int generate_md5_sum(const char *file_name, char *md5_sum)
{
    int ret;
    char cmd_buf[512];
    FILE *p_fp = NULL;

    snprintf(cmd_buf, sizeof(cmd_buf), "/usr/bin/md5sum %s", file_name);

    p_fp = popen(cmd_buf, "r");

    if (p_fp == NULL) {
        return -1;
    }

    ret = fread(md5_sum, sizeof(char), 32, p_fp);

    if (ret != 32) {
        pclose(p_fp);
        return -2;
    }

    md5_sum[ret] = '\0';

    pclose(p_fp);

    return 0;
}

static int
get_free_space(
    const char *location,
    unsigned long long *free_space)
{
    struct statvfs st;
    int ret;

    ret = statvfs(location, &st);

    if (ret < 0) {
        fprintf(stderr, "statvfs call failed. location: %s, error: %s(%d)\n",
                location, strerror(errno), errno);
        return ret;
    }

    if (st.f_blocks == 0) {
        fprintf(stderr, "No space in location: %s\n", location);
        return -1;
    }

    *free_space = (unsigned long long)st.f_bfree * (unsigned long long)st.f_bsize;

    return 0;
}

static int
check_download_info(
    struct wup_device *dev,
    const struct wup_dnload_info *dnload_info)
{
    int ret;

    // Check current status
    if (dev->state != WUP_STATE_dfuIDLE) {
        fprintf(stderr, RED "CHECK DNLOAD INFO: Wrong State. Current State is %d\n" RESET,
                dev->state);
        dev->status = WUP_STATUS_errSTATE;
        return -1;
    }

#if 0

    // Check the software version
    if (!dnload_info->bForced &&
            compareWithCurrentVersion((char *)(dnload_info->sSwVersion)) <= 0) {
        fprintf(stderr, RED "The SW Version (%s) is not allowed by SoftUpdateService\n",
                dnload_info->sSwVersion);
        dev->status = WUP_STATUS_errCHECK;
        return -2;
    }

#endif

    // Check the free space
    unsigned long long free_space = 0;

    ret = get_free_space(FIRMWARE_IMAGE_LOCATION, &free_space);

    if (ret ||
            (unsigned long long)(dnload_info->dwImageSize) >= free_space) {
        fprintf(stderr, RED "No enough space to store the image. image_size: %u, free_space: %llu\n",
                dnload_info->dwImageSize, free_space);

        dev->status = WUP_STATUS_errTARGET;
        return -3;
    }

    // Try to create the tmp file
    strncpy(dev->image_file, WUP_TEMPFILE, sizeof(dev->image_file));

    // Set the last byte to '\0'
    dev->image_file[sizeof(dev->image_file) - 1] = '\0';

    if (dev->file_fd >= 0) {
        close(dev->file_fd);
    }

    dev->file_fd = open(dev->image_file,
                        O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);

    if (dev->file_fd < 0) {
        fprintf(stderr, RED "Failed to open the output file: %s - %s(%d)\n",
                dev->image_file, strerror(errno), errno);

        dev->status = WUP_STATUS_errFILE;
        return -4;
    }

    return 0;
}

static void
wup_events_process(struct wup_device *dev)
{
    struct dfu_ctrl_event wup_event;
    int ret;
    char md5_sum[40];
    unsigned char md5[MD5_DIGEST_LENGTH];
    int i;
    unsigned short reply_len = 0;

    ret = ioctl(dev->fd, DFU_IOC_DQ_CTRL_EVENT, &wup_event);

    if (ret < 0) {
        fprintf(stderr, RED "DFU_IOC_DQ_CTRL_EVENT failed: %s (%d)\n" RESET,
                strerror(errno), errno);
        return;
    }

#if DEBUG_PRINT
    printf(WHT "Got WUP Control Event. Request: 0x%02x, Value: 0x%04x, "
           "Length: 0x%04x\n" RESET, wup_event.request,
           wup_event.value, wup_event.length);
#endif

    switch (wup_event.request) {
        case 0x01: {

            if (wup_event.value == USB_REQ_WUP_SET_DNLOAD_INFO) {
                struct wup_dnload_info dnload_info;

                memset(&dnload_info, 0x00, sizeof(struct wup_dnload_info));

                if (wup_event.length != sizeof(struct wup_dnload_info)) {
                    fprintf(stderr, RED "Invalid Params. Request: 0x%02x,"
                            " Value: 0x%04x, Length: 0x%04x\n" RESET,
                            wup_event.request,
                            wup_event.value,
                            wup_event.length);
                    dev->status = WUP_STATUS_errINVAL;

                    return;
                }

                // Get the download information
                memcpy(&dnload_info,
                       wup_event.data,
                       wup_event.length);

                ret = check_download_info(dev, &dnload_info);

                if (!ret) {
                    //Pass the download information checking

                    dev->image_size = dnload_info.dwImageSize;
                    dev->sync_block_size = dnload_info.dwSyncBlockSize;
                    dev->f_force_updated = dnload_info.bForced;

                    strncpy(dev->image_sw_version,
                            (char *)dnload_info.sSwVersion,
                            sizeof(dev->image_sw_version));

                    // Clear the written_bytes
                    dev->image_written_bytes = 0;

                    // Reset the MD5 Context
                    memset(&dev->md5_ctx, 0x00, sizeof(MD5_CTX));
                    /* Note that bionic's MD5_* functions return void. */
                    MD5_Init(&dev->md5_ctx);

#if DEBUG_CALCULATE_SPEED
                    start_time = get_current_time();
#endif

#if 1
                    // Start the usb out pipe here
                    ret = wup_open_read_pipe(dev);

                    if (ret < 0) {
                        fprintf(stderr, RED "Failed to enable the read pipe. ret is %d\n" RESET,
                            ret);
                        dev->status = WUP_STATUS_errUNKNOWN;
                        return;
                    }
#endif

                    dev->state = WUP_STATE_dfuDNLOAD_IDLE;
                    dev->status = WUP_STATUS_OK;

                    printf(WHT "Start Transfer. Image Size: %lu, "
                           "SyncBlockSize: %lu, bForced: %d, Version Number: %s\n" RESET,
                           dev->image_size, dev->sync_block_size,
                           dnload_info.bForced, dnload_info.sSwVersion);
                }
            } else if (wup_event.value == USB_REQ_WUP_ABORT) {
                printf(WHT "Reset the state. Current State(%d), Status(%d)\n" RESET,
                       dev->state, dev->status);

                if (dev->state != WUP_STATE_dfuUPDATE_BUSY)
                    dev->state = WUP_STATE_dfuIDLE;

                // We need to reset the pipe here.
                wup_close_read_pipe(dev);
#if 0
                // Then re-open the pipe here
                wup_open_read_pipe(dev);
#endif

                // Remove the obsolete file
                unlink(dev->image_file);

                dev->status = WUP_STATUS_OK;
            } else if (wup_event.value == USB_REQ_WUP_INT_CHECK) {

                if (dev->state != WUP_STATE_dfuDNLOAD_IDLE) {
                    //TODO STALL the request
                    printf(WHT "INT CHECK: Wrong State. Current State is %d\n\n\n" RESET,
                           dev->state);
                    dev->status = WUP_STATUS_errSTATE;
                    return;
                }

                // Set the image md5 sum
                memcpy(dev->image_md5, wup_event.data, wup_event.length);

                // Set the last byte to '\0'
                dev->image_md5[sizeof(dev->image_md5) - 1] = '\0';

                printf(GRN "\tGot image MD5 sum: %s\n" RESET, dev->image_md5);

#if 0
                generate_md5_sum(dev->image_file, md5_sum);
#else
                MD5_Final(md5, &dev->md5_ctx);

                for (i = 0; i < MD5_DIGEST_LENGTH; i++) {
                    sprintf(md5_sum + i * 2, "%02x", md5[i]);
                }

#endif

                printf(GRN "\tGenerated image MD5 sum: %s\n" RESET, md5_sum);

                if (strncmp(dev->image_md5, md5_sum, MD5_SUM_LENGTH)) {
                    fprintf(stderr, RED "\t\tFailed to verify the md5 sum\n");
                    fprintf(stderr, RED "\t\torig md5 sum: %s\n", dev->image_md5);
                    fprintf(stderr, RED "\t\tnew md5 sum: %s\n", md5_sum);
                    dev->status = WUP_STATUS_errVERIFY;
                }

                if (dev->file_fd >= 0) {
                    close(dev->file_fd);
                    dev->file_fd = -1;
                }

                if (dev->status == WUP_STATUS_OK) {
#if 0
                    unlink(dev->image_file);
#endif
                    dev->state = WUP_STATE_dfuIDLE;
                }

                printf("\n\n\n");

            } else if (wup_event.value == USB_REQ_WUP_START_UPDATE) {
#if 0
                char swVerFromImage[64];
                int ret = getAvailableUpdateVersion(WUP_TEMPFILE, swVerFromImage);

                if (ret < 0) {
                    fprintf(stderr, RED "\tFailed to read the version from image. ret : %d\n",
                            ret);
                    dev->status = WUP_STATUS_errNOTDONE;
                } else if (ret > 0 && !dev->f_force_updated) {
                    fprintf(stderr, RED "Same or Older version. We will skip it "
                            "Version: %s. Ret: %d\n",
                            swVerFromImage, ret);
                    dev->status = WUP_STATUS_errNOTDONE;
                } else {
                    printf(WHT "We are going to start the softupdate\n");
                    //Start the update.
                    int update_mode = dev->f_force_updated ?
                                      UPDATE_MODE_NORMAL_FORCED : UPDATE_MODE_NORMAL;

                    int result = startSoftupdate(WUP_TEMPFILE, update_mode, 1);

                    if (result) {
                        fprintf(stderr, RED "Start Soft Update failed. ret: %d\n",
                                result);
                        dev->status = WUP_STATUS_errNOTDONE;
                    } else {
                        dev->state = WUP_STATE_dfuUPDATE_BUSY;
                    }
                }

#else
                dev->state = WUP_STATE_dfuUPDATE_BUSY;
#endif
            }
        }
        break;

        case 0x81: {
            // This is Control In Transfer

            if (wup_event.value == USB_REQ_WUP_SYNC) {
                struct wup_status sync_wup_status;

                sync_wup_status.bStatus = dev->status;
                sync_wup_status.bState = dev->state;
                sync_wup_status.u.dwWrittenBytes = dev->image_written_bytes;

                reply_len = (unsigned short)sizeof(struct wup_status);

                printf(WHT "SYNC: image_written_bytes: %d-%ld\n" RESET,
                       sync_wup_status.u.dwWrittenBytes,
                       dev->image_written_bytes);

                if (dev->state != WUP_STATE_dfuDNLOAD_SYNC) {
                    //TODO STALL the request
                    fprintf(stderr, RED "SYNC: Wrong State. Current State is %d."
                            " Written Bytes: %lu\n" RESET,
                            dev->state, dev->image_written_bytes);
                    sync_wup_status.bStatus = WUP_STATUS_errSTATE;
                } else {
                    if (dev->status == WUP_STATUS_OK) {

                        dev->state = WUP_STATE_dfuDNLOAD_IDLE;

                        if (dev->image_written_bytes < dev->image_size) {
                            // Sync the data to the file system
                            //fsync(dev->file_fd);
                            // Continue to read data from host
                            //usb_dfu_read(dev);
                        } else {
                            // Close the file
                            if (dev->file_fd > 0) {
                                close(dev->file_fd);
                                dev->file_fd = -1;
                            }

                            // We need to reset the pipe here.
                            wup_close_read_pipe(dev);
#if 0
                            // Then re-open the pipe here
                            wup_open_read_pipe(dev);
#endif
                        }
                    }
                }

                memcpy(wup_event.data,
                       &sync_wup_status,
                       sizeof(struct wup_status));

            } else if (wup_event.value == USB_REQ_WUP_GETSTATUS) {
                struct wup_status sync_wup_status;

                sync_wup_status.bStatus = dev->status;
                sync_wup_status.bState = dev->state;
                memcpy(wup_event.data,
                       &sync_wup_status,
                       sizeof(struct wup_status));
                reply_len = (unsigned short)sizeof(struct wup_status);

            } else if (wup_event.value == USB_REQ_WUP_GETSTATE) {
                //Get current state
                memcpy(wup_event.data,
                       &dev->state,
                       sizeof(dev->state));
                reply_len = (unsigned short)sizeof(dev->state);
            }
        }
        break;

        default:
            break;
    }

    if ((wup_event.request & 0x80) && reply_len > 0) {
        wup_event.length = wup_event.length > reply_len ?
                           reply_len : wup_event.length;
        //Send the reply
        ret = ioctl(dev->fd, DFU_IOC_SEND_CTRL_REPLY, &wup_event);

        if (ret < 0) {
            fprintf(stderr, RED "DFU_IOC_SEND_CTRL_REPLY failed: %s (%d)\n" RESET,
                    strerror(errno), errno);
            return;
        }
    }
}

static void
wup_data_process(struct wup_device *dev)
{
    int ret;
    char buf[DFU_BUFFER_SIZE];
    int read_len;

    // Check the current state
    if (dev->state != WUP_STATE_dfuDNLOAD_IDLE &&
            dev->state != WUP_STATE_dfuDNLOAD_BUSY) {
        fprintf(stderr, RED "wup_data_process: Wrong state (%d)\n" RESET,
                dev->state);
        dev->status = WUP_STATUS_errSTATE;
        return;
    }

    if (dev->state == WUP_STATE_dfuDNLOAD_IDLE)
        dev->state = WUP_STATE_dfuDNLOAD_BUSY;

    // Check the file descriptor
    if (dev->file_fd < 0) {
        fprintf(stderr, RED "ReadDone: The output file is not open\n" RESET);
        dev->status = WUP_STATUS_errFILE;
        return;
    }

    while ((read_len = read(dev->fd, buf, DFU_BUFFER_SIZE)) > 0) {
        ret = write(dev->file_fd, buf, read_len);

        if (ret < read_len) {
            fprintf(stderr, RED "Failed to write all data into output file, %d - %d\n" RESET,
                    read_len, ret);
            // Change the state to WUP_STATE_dfuDNLOAD_SYNC
            // Waiting for the host to get status
            dev->state = WUP_STATE_dfuDNLOAD_SYNC;
            dev->status = WUP_STATUS_errWRITE;
            return;
        }

        // Update the MD5 sum
        MD5_Update(&dev->md5_ctx, buf, read_len);

        dev->image_written_bytes += ret;

        if (dev->sync_block_size != 0 &&
                dev->image_written_bytes != 0 &&
                dev->image_written_bytes % dev->sync_block_size == 0) {
            // Reach the sync point
            dev->state = WUP_STATE_dfuDNLOAD_SYNC;
            return;
        }

        if (dev->image_written_bytes >= dev->image_size) {
            printf(GRN "Transfer completed, total length: %lu\n" RESET,
                   dev->image_size);
#if DEBUG_CALCULATE_SPEED
            unsigned long used_ms = get_current_time() - start_time;
            printf(CYN "Transfer completed in %lu.%03lus. Speed: %.3fMB/s\n" RESET,
                   used_ms / 1000, used_ms % 1000,
                   ((float)(dev->image_size) / (1024 * 1024) / (used_ms / 1000)));
#endif
            dev->state = WUP_STATE_dfuDNLOAD_SYNC;
            return;
        }
    }
}

static int
wup_open_read_pipe(struct wup_device *dev)
{
    int type = 0;
    int ret;

#if 0
    printf(MAG "Open the read pipe\n" RESET);
#endif

    ret = ioctl(dev->fd, DFU_IOC_OPEN_STREAM, &type);
    if (ret < 0)
        fprintf(stderr, RED "Failed to open the pipe\n" RESET);

    return ret;
}

static void
wup_close_read_pipe(struct wup_device *dev)
{
    int type = 0;
    int ret;

#if 0
    printf(MAG "Close the read pipe\n" RESET);
#endif

    ret = ioctl(dev->fd, DFU_IOC_CLOSE_STREAM, &type);
    if (ret < 0)
        fprintf(stderr, RED "Failed to close the pipe\n" RESET);
}

int main(int argc, char **argv)
{
    int ret = 0;
    fd_set fds, efds, rfds;
    const char *device_name = "/dev/plcm_dfu";

    struct wup_device *dev = NULL;

    dev = (struct wup_device *)malloc(sizeof(struct wup_device));

    if (dev == NULL) {
        fprintf(stderr, "Failed to allocate the struct wup_device\n");
        return -1;
    }

    memset(dev, 0x00, sizeof(struct wup_device));

    dev->fd = -1;
    dev->file_fd = -1;

    dev->fd = open(device_name, O_RDWR | O_NONBLOCK);

    if (dev->fd < 0) {
        fprintf(stderr, RED "Failed to open the wup file: %s(%d)\n" RESET,
                strerror(errno), errno);
        ret = -1;
        goto out;
    }

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    while (1) {
        efds = fds;
        rfds = fds;
        ret = select(dev->fd + 1, &rfds, NULL, &efds, NULL);

        if (ret == -1) {
            if (errno != EINTR) {
                printf("Error in select\n");
                break;
            }
        } else {
            if (FD_ISSET(dev->fd, &efds))
                wup_events_process(dev);
            else if (FD_ISSET(dev->fd, &rfds))
                wup_data_process(dev);
        }
    }

out:

    if (dev) {
        close(dev->fd);

        free(dev);
    }

    return 0;
}
