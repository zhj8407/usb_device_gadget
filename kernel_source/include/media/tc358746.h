#ifndef __TC358746_H__
#define __TC358746_H__

#include <dt-bindings/gpio/tegra-gpio.h>

#define TC358746_NAME    "tc358746"

#define TC358746_I2C_BUSNUM  1
#define TC358746_I2C_ADDR    0x0e

#define TC358746_RST_GPIO    TEGRA_GPIO(W, 2)

#endif // __TC358746_H__
