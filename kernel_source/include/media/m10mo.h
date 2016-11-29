#ifndef __M10MO_H__
#define __M10MO_H__

#include <dt-bindings/gpio/tegra-gpio.h>

#define M10MO_NAME        "FujistuM10MO"

#define M10MO_I2C_BUSNUM  1
#define M10MO_I2C_ADDR    0x1f

#define M10MO_SPI_BUSNUM  0
#define M10MO_SPI_CS      0
#define M10MO_SPI_MAX_SPEED_HZ   10 * 1000 * 1000

#define M10MO_RST_GPIO   TEGRA_GPIO(W, 3)

#endif // __M10MO_H__
