/* include/linux/serial_tegra.h
 *
 * Copyright (C) 2013 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_PLATFORM_DATA_SERIAL_TEGRA_H__
#define __LINUX_PLATFORM_DATA_SERIAL_TEGRA_H__

#define TEGRA_UART_MAXIMUM			5

typedef void (*tegra_wake_peer_t)(struct uart_port *port);
extern tegra_wake_peer_t tegra_serial_wake_peer[TEGRA_UART_MAXIMUM];

struct tegra_serial_platform_data {
	int dma_req_selector;
	bool modem_interrupt;
	int dev_id;
};

#endif /* __LINUX_PLATFORM_DATA_SERIAL_TEGRA_H__ */

