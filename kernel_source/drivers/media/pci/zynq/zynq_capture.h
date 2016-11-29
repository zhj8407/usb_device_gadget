#ifndef ZYNQ_CAPTURE_H
#define ZYNQ_CAPTURE_H

#include "zynq_core.h"

struct vpif_cap_device {
    struct v4l2_device v4l2_dev;
    struct channel_obj *dev[VPIF_CAPTURE_NUM_CHANNELS];
    struct v4l2_subdev **sd;
    struct v4l2_subdev **sd_of_sd;
};

struct vpif_cap_device * vpif_capture_get_instnace(void);
int vpif_capture_release(struct pci_dev *pdev);
int vpif_capture_init(struct pci_dev *pdev);

#ifdef CONFIG_PM_SLEEP
int vpif_capture_suspend(void);
int vpif_capture_resume(void);
#endif

#endif				/* VPIF_CAPTURE_H */
