#ifndef ZYNQ_CONTROL_H
#define ZYNQ_CONTROL_H

#include <linux/pci.h>
#include <media/v4l2-device.h>

struct vpif_control_device {
    struct v4l2_device v4l2_dev;
};

struct vpif_control_device * vpif_control_get_instnace(void);

int vpif_control_init(struct pci_dev *pdev);

int vpif_control_release(struct pci_dev *pdev);

int vpif_control_init_pipeline(struct pci_dev *pdev);

int vpif_control_release_pipeline(struct pci_dev *pdev);

int vpif_control_config_vin(ePIPEPORTID id, unsigned int enable);

int vpif_control_config_reset(unsigned int enable);

int vpif_control_config_webcam_res(unsigned int out_width, unsigned int out_height);

int vpif_control_config_webcam_enable( unsigned int value);

#endif