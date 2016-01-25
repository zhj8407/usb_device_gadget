/*
 * m10mo-ioctl.h - M10MO ioctl definitions.
 *
 * Copyright 2015 Chun-Hsien Li.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 */
#ifndef _UAPI_LINUX_M10MO_IOCTL_H
#define _UAPI_LINUX_M10MO_IOCTL_H

#include <linux/types.h>
#include <linux/compiler.h>


struct m10mo_ver
{
    uint16_t customer_code;
    uint16_t ver_firmware;
    uint16_t ver_hardware;
    uint16_t ver_parameter;
};

struct m10mo_reg
{
    uint8_t  cnt;
    uint8_t  catagory;
    uint8_t  offset;
    uint32_t value;
};

enum
{
    M10MO_VER_GET_CMD = 0,
    M10MO_REG_SET_CMD,
    M10MO_REG_GET_CMD
};


#define M10MO_IOCTL  0xb5

#define	M10MO_VER_GET  _IOR(M10MO_IOCTL, M10MO_VER_GET_CMD, struct m10mo_ver)
#define	M10MO_REG_SET  _IOW(M10MO_IOCTL, M10MO_REG_SET_CMD, struct m10mo_reg)
#define	M10MO_REG_GET  _IOWR(M10MO_IOCTL, M10MO_REG_GET_CMD, struct m10mo_reg)


#endif /* _UAPI_LINUX_M10MO_IOCTL_H */
