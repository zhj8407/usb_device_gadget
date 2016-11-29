/*
 * VPIF display header file
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef	ZYNQ_DISPLAY_H
#define ZYNQ_DISPLAY_H

#include "zynq_core.h"

/* vpif device structure */
struct vpif_disp_device {
    struct v4l2_device v4l2_dev;
    struct channel_obj *dev[VPIF_DISPLAY_NUM_CHANNELS];
    struct v4l2_subdev **sd;

};

int vpif_display_release(struct pci_dev *pdev);
int vpif_display_init(struct pci_dev *pdev);

#ifdef CONFIG_PM_SLEEP
int vpif_display_suspend(void);
int vpif_display_resume(void);
#endif

#endif				/* DAVINCIHD_DISPLAY_H */
