#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>

#include <linux/io.h>
#include <asm/irq.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/vmalloc.h>
#include  <linux/slab.h>
#include "zynq_gpio.h"
#include "zynq_debug.h"
#include "zynq_core.h"

#include "zynq_fpga_verify.h"

unsigned int uart_interrupt_mode  = 0; //0: single interrupt, 1: four interrupt
module_param(uart_interrupt_mode, int, 0644);

//#ifndef FPGA_VERIF
//#define VIRTUAL_UART_DEVICE 1
//#endif
//////////////////////////////////////////////////////////////////////////
/* ---------------------------------------------------------------------
 * Register definitions
 *
 * For register details see datasheet:
 * http://www.xilinx.com/support/documentation/ip_documentation/opb_uartlite.pdf
 * http://www.xilinx.com/support/documentation/ip_documentation/xps_uartlite/v1_02_a/xps_uartlite.pdf
 *
 * 0. The UART Lite registers are organized as big-endian data.
 * 1.	 The XPS UART Lite generates an interrupt when Receive FIFO becomes non-empty or when transmit FIFO becomes empty.
 * 	 This interrupt can be masked by using an interrupt enable/disable signal.
 * 2.	16-character Transmit FIFO and 16-character Receive FIFO.
 * 3. 	Configurable number of data bits in a character (5-8)
 * 4.	Configurable parity bit (odd or even).
 * 5. 	If interrupts are enabled, an interrupt is generated when one of these two conditions is true:
 * (1)   When the Receive FIFO goes from empty to not empty, such as when the first valid character is received in the Receive FIFO.
 * (2)   When the Transmit FIFO goes from not empty to empty, such as when the last character in the Transmit FIFO is transmitted.
 */
//////////////////////////////////////////////////////////////////////////
#define ULITE_RX		0x00
#define ULITE_TX		0x04
#define ULITE_STATUS		0x08
#define ULITE_CONTROL		0x0c

#define ULITE_REGION		16

#define ULITE_STATUS_RXVALID	0x01
#define ULITE_STATUS_RXFULL	0x02
#define ULITE_STATUS_TXEMPTY	0x04
#define ULITE_STATUS_TXFULL	0x08
#define ULITE_STATUS_IE		0x10
#define ULITE_STATUS_OVERRUN	0x20
#define ULITE_STATUS_FRAME	0x40
#define ULITE_STATUS_PARITY	0x80

#define ULITE_CONTROL_RST_TX	0x01
#define ULITE_CONTROL_RST_RX	0x02
#define ULITE_CONTROL_IE	0x10

struct uartlite_reg_ops {
    u32 (*in)(void __iomem *addr);
    void (*out)(u32 val, void __iomem *addr);
};

static u32 uartlite_inbe32(void __iomem *addr)
{
#if 0
    u32 val = 0;
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    if (zynq_reg_base != NULL) {
        val = fpga_reg_read_be(zynq_reg_base, reg);
    }
    return val;
#else
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    reg &= 0x0ff;
    switch (reg) {
        case 0x00:
            goto exec;
        case 0x04:
            goto exec;
        case 0x08:
            goto exec;
        case 0x0c:
            goto  exec;
        default:
            zynq_printk(0, "[zynq_uart](%d)Ilegal reg: %02x\n", __LINE__, reg);
            goto exit;
    }
exec:
    return ioread32be(addr);
exit:
    return 0;
#endif

}

static void uartlite_outbe32(u32 val, void __iomem *addr)
{
#if 0
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    if (zynq_reg_base != NULL) {
        fpga_reg_write_be(zynq_reg_base, reg, val);
    }
#else
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    reg &= 0x0ff;
    switch (reg) {
        case 0x00:
            goto exec;
        case 0x04:
            goto exec;
        case 0x08:
            goto exec;
        case 0x0c:
            goto  exec;
        default:
            zynq_printk(0, "[zynq_uart](%d)Ilegal reg: %02x\n", __LINE__, reg);
            goto exit;
    }
exec:
    iowrite32be(val, addr);
exit:
    return;
#endif

}

static struct uartlite_reg_ops uartlite_be = {
    .in = uartlite_inbe32,
    .out = uartlite_outbe32,
};

static u32 uartlite_inle32(void __iomem *addr)
{
#if 0
    u32 val = 0;
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    if (zynq_reg_base != NULL) {
        val = fpga_reg_read(zynq_reg_base, reg);
    }
    return val;
#else
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    reg &= 0x0ff;
    switch (reg) {
        case 0x00:
            goto exec;
        case 0x04:
            goto exec;
        case 0x08:
            goto exec;
        case 0x0c:
            goto  exec;
        default:
            zynq_printk(0, "[zynq_uart](%d)Ilegal reg: %02x\n", __LINE__, reg);
            goto exit;
    }
exec:
    return ioread32(addr);
exit:
    return 0;
#endif
}

static void uartlite_outle32(u32 val, void __iomem *addr)
{
#if 0
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    if (zynq_reg_base != NULL) {
        fpga_reg_write(zynq_reg_base, reg, val);
    }
#else
    u32 reg = (u32)addr - (u32)zynq_reg_base;
    reg &= 0x0ff;
    switch (reg) {
        case 0x00:
            goto exec;
        case 0x04:
            goto exec;
        case 0x08:
            goto exec;
        case 0x0c:
            goto  exec;
        default:
            zynq_printk(0, "[zynq_uart](%d)Ilegal reg: %02x\n", __LINE__, reg);
            goto exit;
    }
exec:
    iowrite32(val, addr);
exit:
    return;
#endif
}

static struct uartlite_reg_ops uartlite_le = {
    .in = uartlite_inle32,
    .out = uartlite_outle32,
};

static inline u32 uart_in32(u32 offset, struct uart_port *port)
{
    struct uartlite_reg_ops *reg_ops = port->private_data;

    return reg_ops->in(port->membase + offset);
}

static inline void uart_out32(u32 val, u32 offset, struct uart_port *port)
{
    struct uartlite_reg_ops *reg_ops = port->private_data;

    reg_ops->out(val, port->membase + offset);
}


/* ---------------------------------------------------------------------
 * Core UART driver operations
 */

static int ulite_receive(struct uart_port *port, int stat)
{
    struct tty_port *tport = &port->state->port;
    unsigned char ch = 0;
    char flag = TTY_NORMAL;

    if ((stat & (ULITE_STATUS_RXVALID | ULITE_STATUS_OVERRUN
                 | ULITE_STATUS_FRAME)) == 0)
        return 0;

    /* stats */
    if (stat & ULITE_STATUS_RXVALID) {
        port->icount.rx++;
        ch = uart_in32(ULITE_RX, port);

        if (stat & ULITE_STATUS_PARITY)
            port->icount.parity++;
    }

    if (stat & ULITE_STATUS_OVERRUN)
        port->icount.overrun++;

    if (stat & ULITE_STATUS_FRAME)
        port->icount.frame++;


    /* drop byte with parity error if IGNPAR specificed */
    if (stat & port->ignore_status_mask & ULITE_STATUS_PARITY)
        stat &= ~ULITE_STATUS_RXVALID;

    stat &= port->read_status_mask;

    if (stat & ULITE_STATUS_PARITY)
        flag = TTY_PARITY;


    stat &= ~port->ignore_status_mask;

    if (stat & ULITE_STATUS_RXVALID)
        tty_insert_flip_char(tport, ch, flag);

    if (stat & ULITE_STATUS_FRAME)
        tty_insert_flip_char(tport, 0, TTY_FRAME);

    if (stat & ULITE_STATUS_OVERRUN)
        tty_insert_flip_char(tport, 0, TTY_OVERRUN);

    return 1;
}

static int ulite_transmit(struct uart_port *port, int stat)
{
    struct circ_buf *xmit  = &port->state->xmit;

    if (stat & ULITE_STATUS_TXFULL)
        return 0;

    if (port->x_char) {
        uart_out32(port->x_char, ULITE_TX, port);
        port->x_char = 0;
        port->icount.tx++;
        return 1;
    }

    if (uart_circ_empty(xmit) || uart_tx_stopped(port))
        return 0;

    uart_out32(xmit->buf[xmit->tail], ULITE_TX, port);
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
    port->icount.tx++;

    /* wake up */
    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);

    return 1;
}


//////////////////////////////////////////////////////////////////////////
#ifdef VIRTUAL_UART_DEVICE

#define DELAY_TIME		HZ * 2	/* 2 seconds per character */
#define TINY_DATA_CHARACTER	't'

struct timer_list timer;

static void tiny_tx_char(struct uart_port *port);
static void tiny_timer(unsigned long data);
static void tiny_set_termios(struct uart_port *port, struct ktermios *new, struct ktermios *old);
static int tiny_start_rx(struct uart_port *port);
static void tiny_stop_rx(struct uart_port *port);

#endif
///////////////////////////////////////////////////////////////////////////
/* We'll be using  Xilinx uartlite serial controller's major/minor */
#define ZYNQ_UART_MAJOR	204
#define ZYNQ_UART_MINOR_START	187
#define ZYNQ_UART_NR_PORTS		4
//////////////////////////////////////////////////////////////////////////
extern void __iomem *zynq_reg_base ;
static void __iomem *zynq_uart_reg_base = NULL;

#define ZYNQ_UART_BASE  ((unsigned long)zynq_uart_reg_base)

#define ZYNQ_UART0_PORT_START     (ZYNQ_UART_BASE + 0x200)
#define ZYNQ_UART0_PORT_END         (ZYNQ_UART_BASE + 0x3FF)
#define ZYNQ_UART1_PORT_START     (ZYNQ_UART_BASE + 0x400)
#define ZYNQ_UART1_PORT_END         (ZYNQ_UART_BASE + 0x5FF)

#define ZYNQ_UART2_PORT_START     (ZYNQ_UART_BASE + 0x600)
#define ZYNQ_UART2_PORT_END         (ZYNQ_UART_BASE + 0x7FF)
#define ZYNQ_UART3_PORT_START     (ZYNQ_UART_BASE + 0x800)
#define ZYNQ_UART3_PORT_END         (ZYNQ_UART_BASE + 0x9FF)

#define ZYNQ_UART_IRQ_PIN GPIO_PI7

#define ZYNQ_UART0_IRQ_PIN GPIO_PI7
#define ZYNQ_UART1_IRQ_PIN GPIO_PI7
#define ZYNQ_UART2_IRQ_PIN GPIO_PI7
#define ZYNQ_UART3_IRQ_PIN GPIO_PI7


#define ZYNQ_UART_CLOCK			(9600 * 16) //(460800 * 16)


//[TODO] Use the PORT_UARTLITE as the port number temporarily or define a new port number for ZYNQ uart in include/uapi/linux/serial_core.h ?
#define ZYNQ_PORT PORT_UARTLITE

#define ZYNQ_DUMMY_READ			BIT(16) //0x00010000
///////////////////////////////////////////////////////////////////////////
#define ZYNQ_RX_FIFO_COUNT		0x00c
#define ZYNQ_TX_FIFO_COUNT		0x00e

//////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_SERIAL_ZYNQ_CONSOLE

static void ulite_console_write(struct console *co, const char *s, unsigned int count);
static int ulite_console_setup(struct console *co, char *options);
static struct uart_driver zynq_uart_driver;
//TODO: When the uart driver is built as module, there will be  "ERROR: "uart_console_device" [drivers/media/pci/zynq/zynq.ko] undefined!" .
struct tty_driver *my_uart_console_device(struct console *co, int *index)
{
    struct uart_driver *p = co->data;
    *index = co->index;
    return p->tty_driver;
}
static struct console ulite_console = {
    .name	= "ttyZYNQS",
    .write	= ulite_console_write,
    .device	= my_uart_console_device,// uart_console_device,
    .setup	= ulite_console_setup,
    .flags	=   CON_PRINTBUFFER,
    .index	= -1, /* Specified on the cmdline (e.g. console=ttyZYNQS0 ) */
    .data	= &zynq_uart_driver
};

#define ZYNQ_CONSOLE	&ulite_console

#else

#define ZYNQ_CONSOLE	NULL

#endif

static struct uart_driver zynq_uart_driver = {
    .owner			= THIS_MODULE,
    .driver_name		= "zynq_uart_driver",
    .dev_name		= "ttyZYNQS",
    .major			= ZYNQ_UART_MAJOR,
    .minor			= ZYNQ_UART_MINOR_START,
    .nr					=	ZYNQ_UART_NR_PORTS,
    .cons				= ZYNQ_CONSOLE,
};
//////////////////////////////////////////////////////////////////////////

static unsigned int zynq_uart_tx_empty(struct uart_port *port);
static void zynq_uart_set_mctrl(struct uart_port *port, unsigned int mctrl);
static unsigned int zynq_uart_get_mctrl(struct uart_port *port);
static void zynq_uart_start_tx(struct uart_port *port);
static void zynq_uart_stop_tx(struct uart_port *port);
static void zynq_uart_stop_rx(struct uart_port *port);
static void zynq_uart_break_ctl(struct uart_port *port, int break_state);
static void zynq_uart_enable_ms(struct uart_port *port);
static void zynq_uart_set_termios(struct uart_port *port, struct ktermios *new, struct ktermios *old) ;
static int zynq_uart_startup(struct uart_port *port);
static void zynq_uart_shutdown(struct uart_port *port);
static const char *zynq_uart_type(struct uart_port *port) ;
static void zynq_uart_release_port(struct uart_port *port);
static int zynq_uart_request_port(struct uart_port *port);
static void zynq_uart_config_port(struct uart_port *port, int flags) ;
static int zynq_uart_verify_port(struct uart_port *port, struct serial_struct *ser);
static irqreturn_t  zynq_uart_isr(int irq, void *dev_id);
static irqreturn_t ulite_isr_by_one_port(int irq, void *dev_id);
#ifdef CONFIG_CONSOLE_POLL
static int zynq_get_poll_char(struct uart_port *port);
static void zynq_put_poll_char(struct uart_port *port, unsigned char ch);
#endif
static const struct uart_ops zynq_uart_ops = {
    .tx_empty	= zynq_uart_tx_empty,
    .set_mctrl	= zynq_uart_set_mctrl,
    .get_mctrl	= zynq_uart_get_mctrl,
    .stop_tx	= zynq_uart_stop_tx,
    .start_tx	= zynq_uart_start_tx,
    .stop_rx	= zynq_uart_stop_rx,
    .enable_ms	= zynq_uart_enable_ms,
    .break_ctl	= zynq_uart_break_ctl,
    .startup	= zynq_uart_startup,
    .shutdown	= zynq_uart_shutdown,
    .set_termios	= zynq_uart_set_termios,
    .type		= zynq_uart_type,
    .release_port	= zynq_uart_release_port,
    .request_port	= zynq_uart_request_port,
    .config_port	= zynq_uart_config_port,
    .verify_port	= zynq_uart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
    .poll_get_char	= zynq_get_poll_char,
    .poll_put_char	= zynq_put_poll_char
#endif
};
////////////////////////////////////////////////////////////////////////
struct zynq_card_t;

struct zynq_uart_t {
    struct uart_port	port;
    struct timer_list	timer;
    unsigned int		old_status;
    void __iomem			*base;
    int idx;
    struct zynq_card_t *card;
    unsigned int enable;
    atomic_t refcount;
    unsigned int irq;
};

struct zynq_card_t {
    struct pci_dev *pdev;
    struct zynq_uart_t 	*uarts;
    int	n_uarts;
    spinlock_t			card_lock;
    int initialized_uarts;
    unsigned int irq;
};

struct zynq_card_t  *zynq_card = NULL;

struct zynq_uart_t 	zynq_uarts[] = {
    [0] = {
        .enable = 1,
        .idx = 0,
        .port	= {
            .type		= PORT_UNKNOWN,
            .iotype		= UPIO_MEM,
            .uartclk	= ZYNQ_UART_CLOCK,
            .fifosize	= 16,
            .flags		= UPF_BOOT_AUTOCONF,
            .line		= 0,
            .regshift = 2,
            .iobase = 1
        },
    },
    [1] = {
        .enable = 1,
        .idx = 1,
        .port	= {
            .type		= PORT_UNKNOWN,
            .iotype		= UPIO_MEM,
            .uartclk	= ZYNQ_UART_CLOCK,
            .fifosize	= 16,
            .flags		= UPF_BOOT_AUTOCONF,
            .line		= 1,
            .regshift = 2,
            .iobase = 1
        },
    },
    [2] = {
        .enable = 1,
        .idx = 2,
        .port	= {
            .type		= PORT_UNKNOWN,
            .iotype		= UPIO_MEM,
            .uartclk	= ZYNQ_UART_CLOCK,
            .fifosize	= 16,
            .flags		= UPF_BOOT_AUTOCONF,
            .line		= 2,
            .regshift = 2,
            .iobase = 1
        },
    },
    [3] = {
        .enable = 1,
        .idx = 3,
        .port	= {
            .type		= PORT_UNKNOWN,
            .iotype		= UPIO_MEM,
            .uartclk	= ZYNQ_UART_CLOCK,
            .fifosize	= 16,
            .flags		= UPF_BOOT_AUTOCONF,
            .line		= 3,
            .regshift = 2,
            .iobase = 1
        },
    }
};

static inline struct zynq_uart_t  *port_to_zynq_uart(struct uart_port *port)
{
    return container_of(port, struct zynq_uart_t, port);
}

static void remove_uarts(struct zynq_card_t *card)
{
    int i = 0;

    for (i = 0; i < card->initialized_uarts; i++) {
        struct uart_port *p = &card->uarts[i].port;
        uart_remove_one_port(&zynq_uart_driver, p);
    }
    card->initialized_uarts= 0;
}

static int check_endianess(struct uart_port *port)
{
#ifndef VIRTUAL_UART_DEVICE
    u32 ret  = 0x00000000;
    if (!port->membase) return  -1;
    port->private_data = &uartlite_le;//&uartlite_be;
    ret = uart_in32(ULITE_CONTROL, port);
    uart_out32(ULITE_CONTROL_RST_TX, ULITE_CONTROL, port);
    ret = uart_in32(ULITE_STATUS, port);
    /* Endianess detection */
    if ((ret & ULITE_STATUS_TXEMPTY) != ULITE_STATUS_TXEMPTY) {
        port->private_data = &uartlite_be; //&uartlite_le;
        //zynq_printk(0, "[zynq_uart]The registers are organized as little-endian!!\n");
        //zynq_printk(1, "[zynq_uart]The registers are organized as big-endian!!\n");
    } else {
        //zynq_printk(0, "[zynq_uart]The registers are organized as big-endian!!\n");
        //zynq_printk(1, "[zynq_uart]The registers are organized as little-endian!!\n");
    }
#endif
    return 0;
}

/*
 parameter BASE_GLOBAL                           = 0,
                  	  BASE_UART0                            = 1,
                      BASE_UART1                            = 2,
                      BASE_UART2                            = 3,
                                BASE_UART3                            = 4,
                                BASE_TDM                              = 5,
                                BASE_VDMA                             = 6,
                                BASE_SCALER                           = 7,
                                BASE_OSD                              = 8;

 */

static int init_uart_ports( struct pci_dev *pdev)
{
    int ret = 0, i = 0;

    if (zynq_reg_base == NULL) return  -1;

    zynq_uart_reg_base = zynq_reg_base;

    if (zynq_uart_reg_base == NULL) return  -1;
#if 0
    zynq_printk(1,"[zynq_uart] Zynq UART Regisetr Base Address : 0x%08x (0x%08lx)(phy: 0x%08x)\n",(unsigned int)zynq_uart_reg_base, ZYNQ_UART_BASE, (unsigned int)virt_to_phys((void *)zynq_uart_reg_base));

    zynq_printk(1,"[zynq_uart] Zynq UART 0  Address : 0x%08lx (phy: 0x%08x)\n",ZYNQ_UART0_PORT_START,  (unsigned int)virt_to_phys((void *)ZYNQ_UART0_PORT_START));

    zynq_printk(1,"[zynq_uart] Zynq UART 1  Address : 0x%08lx (phy: 0x%08x)\n",ZYNQ_UART1_PORT_START,  (unsigned int)virt_to_phys((void *)ZYNQ_UART1_PORT_START));

    zynq_printk(1,"[zynq_uart] Zynq UART 2  Address : 0x%08lx (phy: 0x%08x)\n",ZYNQ_UART2_PORT_START,  (unsigned int)virt_to_phys((void *)ZYNQ_UART2_PORT_START));

    zynq_printk(1,"[zynq_uart] Zynq UART 3  Address : 0x%08lx (phy: 0x%08x)\n",ZYNQ_UART3_PORT_START,  (unsigned int)virt_to_phys((void *)ZYNQ_UART3_PORT_START));
#endif

    zynq_card = (struct zynq_card_t *)vmalloc(sizeof(struct zynq_card_t));

    if (!zynq_card) return -ENOMEM;

    spin_lock_init(&zynq_card->card_lock);
    zynq_card->uarts = &zynq_uarts[0];
    zynq_card->n_uarts = 0;
    zynq_card->pdev = pdev;
    zynq_card->initialized_uarts = 0;
    zynq_card->irq =  gpio_to_irq(ZYNQ_UART_IRQ_PIN);

    for (i = 0; i < ZYNQ_UART_NR_PORTS;  i++) {
        if (zynq_uarts[i].enable == 1) zynq_card->n_uarts++;
    }

    zynq_uarts[0].card = zynq_card;
    zynq_uarts[0].base = (void __iomem *)ZYNQ_UART0_PORT_START;
    zynq_uarts[0].port.membase = 	(void __iomem *)ZYNQ_UART0_PORT_START;
    zynq_uarts[0].port.mapbase	= ZYNQ_UART0_PORT_START;
    zynq_uarts[0].port.irq = zynq_card->irq;
    zynq_uarts[0].port.ops = &zynq_uart_ops;
    atomic_set(&zynq_uarts[0].refcount, -1);
    check_endianess(&zynq_uarts[0].port);
    zynq_uarts[0].irq =  gpio_to_irq(ZYNQ_UART0_IRQ_PIN);

    zynq_uarts[1].card = zynq_card;
    zynq_uarts[1].base = (void __iomem *)ZYNQ_UART1_PORT_START;
    zynq_uarts[1].port.membase = 	(void __iomem *)ZYNQ_UART1_PORT_START;
    zynq_uarts[1].port.mapbase	= ZYNQ_UART1_PORT_START;
    zynq_uarts[1].port.irq = zynq_card->irq ;
    zynq_uarts[1].port.ops = &zynq_uart_ops;
    atomic_set(&zynq_uarts[1].refcount, -1);
    check_endianess(&zynq_uarts[1].port);
    zynq_uarts[1].irq =  gpio_to_irq(ZYNQ_UART1_IRQ_PIN);

    zynq_uarts[2].card = zynq_card;
    zynq_uarts[2].base = (void __iomem *)ZYNQ_UART2_PORT_START;
    zynq_uarts[2].port.membase = 	(void __iomem *)ZYNQ_UART2_PORT_START;
    zynq_uarts[2].port.mapbase	= ZYNQ_UART2_PORT_START;
    zynq_uarts[2].port.irq = zynq_card->irq;
    zynq_uarts[2].port.ops = &zynq_uart_ops;
    atomic_set(&zynq_uarts[2].refcount, -1);
    check_endianess(&zynq_uarts[2].port);
    zynq_uarts[2].irq =  gpio_to_irq(ZYNQ_UART2_IRQ_PIN);

    zynq_uarts[3].card = zynq_card;
    zynq_uarts[3].base = (void __iomem *)ZYNQ_UART3_PORT_START;
    zynq_uarts[3].port.membase = 	(void __iomem *)ZYNQ_UART3_PORT_START;
    zynq_uarts[3].port.mapbase	= ZYNQ_UART3_PORT_START;
    zynq_uarts[3].port.irq = zynq_card->irq;
    zynq_uarts[3].port.ops = &zynq_uart_ops;
    atomic_set(&zynq_uarts[3].refcount, -1);
    check_endianess(&zynq_uarts[3].port);
    zynq_uarts[3].irq =  gpio_to_irq(ZYNQ_UART3_IRQ_PIN);

    for (i = 0; i < zynq_card->n_uarts; i++) {
        struct uart_port *p = &zynq_uarts[i].port;
        ret = uart_add_one_port(&zynq_uart_driver, p);
        if (ret) {
            zynq_printk(0, "[zynq_uart]Error registering uart %d: %d !!\n", i, ret);
            remove_uarts(zynq_card);
            break;
        }
        zynq_card->initialized_uarts++;

        if (uart_interrupt_mode == 1) {
            if(request_irq (zynq_uarts[i].irq, ulite_isr_by_one_port,  IRQF_ONESHOT    |   IRQF_TRIGGER_RISING, "zynq_uart", (void *)p)) 	{
                zynq_printk(0,  "[zynq_uart] IRQ request for  pin %u fail!!\n", zynq_uarts[i].irq);
                return -1;
            }
        }

    }
    /*
     * IRQF_ONESHOT - Interrupt is not reenabled after the hardirq handler finished.
     * Used by threaded interrupts which need to keep the irq line disabled until the threaded handler has been run.
     * IRQF_SHARED - allow sharing the irq among several devices
     */

    if (uart_interrupt_mode == 0) {
        if(request_irq (zynq_card->irq, zynq_uart_isr,  IRQF_ONESHOT    |  IRQF_TRIGGER_HIGH /*| IRQF_TRIGGER_RISING*/, "zynq_uart", (void *)zynq_card)) 	{
            zynq_printk(0,  "[zynq_uart] IRQ request for  pin %u fail!!\n", zynq_card->irq);
            return -1;
        }
    }

    return 0;
}

/////////////////////////////////////////////////////////////////////////
/*For 4 interrupt mode*/
static irqreturn_t ulite_isr_by_one_port(int irq, void *dev_id)
{
    struct uart_port *port = (struct uart_port *)dev_id;
    int busy, n = 0;

    do {
        int stat = uart_in32(ULITE_STATUS, port);
        busy  = ulite_receive(port, stat);
        busy |= ulite_transmit(port, stat);
        n++;
    } while (busy);

    /* work done? */
    if (n > 1) {
        tty_flip_buffer_push(&port->state->port);
        return IRQ_HANDLED;
    } else {
        return IRQ_NONE;
    }
}
/////////////////////////////////////////////////////////////////////////
/*For single interrupt mode*/
static int ulite_isr(struct uart_port *port)
{
    int busy =0, n = 0;

    do {
        int stat = uart_in32(ULITE_STATUS, port);
        busy  = ulite_receive(port, stat);
        busy |= ulite_transmit(port, stat);
        n++;
    } while (busy);

    /* work done? */
    if (n > 1) {
        tty_flip_buffer_push(&port->state->port);
        return  1;
    } else {
        return  0;
    }
}

static irqreturn_t  zynq_uart_isr(int irq, void *dev_id)
{
    struct zynq_card_t *card = (struct zynq_card_t *)dev_id;
    struct zynq_uart_t 	*uarts = NULL;
    int handled = 0;
    unsigned int i = 0;

    if (!card) return IRQ_NONE;

    uarts = card->uarts;

    if (!uarts) return IRQ_NONE;

    for (i  = 0 ; i < ZYNQ_UART_NR_PORTS; i++) {
        if (uarts[i].enable) {
            if (atomic_inc_and_test(&uarts[i].refcount)) {
                if (ulite_isr(&(uarts[i].port))) handled |= (1 << i);
            } else {
                zynq_printk(0, "[zynq_uart](%d)The ISR() is breaked.(%d)\n", __LINE__, atomic_read(&uarts[i].refcount) );
            }
            atomic_dec(&uarts[i].refcount);
        }
    }
    return handled ? IRQ_HANDLED : IRQ_NONE;
}

static unsigned int zynq_uart_tx_empty(struct uart_port *port)
{

    struct zynq_uart_t  *up = port_to_zynq_uart(port);
    if (!up) return  0;
#ifndef VIRTUAL_UART_DEVICE
    {
        unsigned int ret;
        ret = uart_in32(ULITE_STATUS, port);
        return ret & ULITE_STATUS_TXEMPTY ? TIOCSER_TEMT : 0;
    }
#else
    return   TIOCSER_TEMT ;
#endif
}
static void zynq_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return;
}
static unsigned int zynq_uart_get_mctrl(struct uart_port *port)
{

    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return 0;

    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}
static void zynq_uart_start_tx(struct uart_port *port)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);
    unsigned int ret;
    if (!up) return;
#ifndef VIRTUAL_UART_DEVICE
    ret=uart_in32(ULITE_STATUS, port);
    ulite_transmit(port, ret);
#else
    //zynq_printk(1,"[zynq_uart]Enter zynq_uart_start_tx()\n");
    tiny_tx_char(port);
    //zynq_printk(1,"[zynq_uart]Leave zynq_uart_start_tx()\n");
#endif

}

static void zynq_uart_stop_tx(struct uart_port *port)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return;

}

static void zynq_uart_stop_rx(struct uart_port *port)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return;

    port->ignore_status_mask = ULITE_STATUS_RXVALID | ULITE_STATUS_PARITY | ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;

}

static void zynq_uart_break_ctl(struct uart_port *port, int break_state)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return;
}

static void zynq_uart_enable_ms(struct uart_port *port)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return;
}


static void zynq_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{

#ifdef VIRTUAL_UART_DEVICE
    tiny_set_termios(port, termios, old);
#else
    unsigned long flags;
    unsigned int baud;

    spin_lock_irqsave(&port->lock, flags);

    port->read_status_mask = ULITE_STATUS_RXVALID | ULITE_STATUS_OVERRUN
                             | ULITE_STATUS_TXFULL;

    if (termios->c_iflag & INPCK)
        port->read_status_mask |=
            ULITE_STATUS_PARITY | ULITE_STATUS_FRAME;

    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNPAR)
        port->ignore_status_mask |= ULITE_STATUS_PARITY
                                    | ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;

    /* ignore all characters if CREAD is not set */
    if ((termios->c_cflag & CREAD) == 0)
        port->ignore_status_mask |=
            ULITE_STATUS_RXVALID | ULITE_STATUS_PARITY
            | ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;

    /* update timeout */
    baud = uart_get_baud_rate(port, termios, old, 0, 460800);
    uart_update_timeout(port, termios->c_cflag, baud);

    spin_unlock_irqrestore(&port->lock, flags);
//	zynq_printk(0,"[zynq_uart]Got the baud rate: %u\n", baud);
#endif
}

static int zynq_uart_startup(struct uart_port *port)
{
#ifdef VIRTUAL_UART_DEVICE
    tiny_start_rx(port);
#else
    uart_out32(ULITE_CONTROL_RST_RX | ULITE_CONTROL_RST_TX, ULITE_CONTROL, port);
    uart_out32(ULITE_CONTROL_IE, ULITE_CONTROL, port);
#endif
    return 0;
}

static void zynq_uart_shutdown(struct uart_port *port)
{

#ifdef VIRTUAL_UART_DEVICE
    tiny_stop_rx(port);
#else
    uart_out32(0, ULITE_CONTROL, port);
    uart_in32(ULITE_CONTROL, port); /* dummy */
#endif
}

static const char *zynq_uart_type(struct uart_port *port)
{
    return (port->type == ZYNQ_PORT) ? "ZYNQ UART" : NULL;

}

static void zynq_uart_release_port(struct uart_port *port)
{
    /* Nothing to release ... */
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return;

    //zynq_printk(1,"[zynq_uart]Release uart port %d (%p) (zynq_uart = %p)\n", up->idx, port, up);
}

static int zynq_uart_request_port(struct uart_port *port)
{
    /* UARTs always present */
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return -1;

    //zynq_printk(1,"[zynq_uart]Request uart port %d (%p) (zynq_uart = %p)\n", up->idx, port, up);
    return 0;
}

static void zynq_uart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE) port->type = ZYNQ_PORT;

}

static int zynq_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{

    if (ser->type != PORT_UNKNOWN && ser->type != ZYNQ_PORT) return -EINVAL;

    return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int zynq_get_poll_char(struct uart_port *port)
{
    if (!(uart_in32(ULITE_STATUS, port) & ULITE_STATUS_RXVALID))
        return NO_POLL_CHAR;

    return uart_in32(ULITE_RX, port);
}

static void zynq_put_poll_char(struct uart_port *port, unsigned char ch)
{
    while (uart_in32(ULITE_STATUS, port) & ULITE_STATUS_TXFULL)
        cpu_relax();

    /* write char to device */
    uart_out32(ch, ULITE_TX, port);
}
#endif

/////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_PM_SLEEP
int zynq_uart_suspend(void)
{
    int ret = 0;

    return ret;
}

int zynq_uart_resume(void)
{
    int ret = 0;

    return  ret;
}
#endif

int zynq_uart_probe( struct pci_dev *pdev)
{
    int ret  = 0;

    if (init_uart_ports(pdev) !=  0) {
        if (zynq_card != NULL) {
            vfree(zynq_card);
            zynq_card = NULL;
        }
        zynq_printk(0, " [zynq_uart]Failed to initialize uart function !!\n");
        return  -1;
    }

    zynq_printk(1, " [zynq_uart]The uart  function is initialized !!\n");

    return ret;
}

int zynq_uart_remove(struct pci_dev *pdev)
{
    if (zynq_card != NULL) {
        if (zynq_card->irq != -1) 	free_irq(zynq_card->irq, zynq_card);
        remove_uarts(zynq_card);
        vfree(zynq_card);
        zynq_card = NULL;
    }
    zynq_printk(1, " [zynq_uart]The uart  function is rleased !!\n");
    return 0;
}

static int is_success_register_uart = 0;

int zynq_uart_init(void)
{
    int ret = 0;

    ret = uart_register_driver(& zynq_uart_driver);

    if (ret) {
        zynq_printk(0,"[zynq_uart]Failed to call  uart_register_driver() !!\n");
        goto exit;
    }
    is_success_register_uart = 1;
    return 0;

exit:
    return ret;
}

void  zynq_uart_exit(void)
{
    if (is_success_register_uart == 1) uart_unregister_driver(& zynq_uart_driver);
}
///////////////////////////////////////////////////////////////////////////////////////////////////
/* ---------------------------------------------------------------------
 * Console driver operations
 */
#ifdef CONFIG_SERIAL_ZYNQ_CONSOLE
static void ulite_console_wait_tx(struct uart_port *port)
{
    int i;
    u8 val;

    /* Spin waiting for TX fifo to have space available */
    for (i = 0; i < 100000; i++) {
        val = uart_in32(ULITE_STATUS, port);
        if ((val & ULITE_STATUS_TXFULL) == 0)
            break;
        cpu_relax();
    }
}

static void ulite_console_putchar(struct uart_port *port, int ch)
{
    ulite_console_wait_tx(port);
    uart_out32(ch, ULITE_TX, port);
}

static void ulite_console_write(struct console *co, const char *s,
                                unsigned int count)
{
    //struct uart_port *port = &ulite_ports[co->index];
    struct uart_port *port = NULL;

    unsigned long flags;
    unsigned int ier;
    int locked = 1;

    if (!zynq_card) return;

    port = &(zynq_card->uarts[co->index].port);

    if (!port) return;

    if (oops_in_progress) {
        locked = spin_trylock_irqsave(&port->lock, flags);
    } else
        spin_lock_irqsave(&port->lock, flags);

    /* save and disable interrupt */
    ier = uart_in32(ULITE_STATUS, port) & ULITE_STATUS_IE;
    uart_out32(0, ULITE_CONTROL, port);

    uart_console_write(port, s, count, ulite_console_putchar);

    ulite_console_wait_tx(port);

    /* restore interrupt state */
    if (ier)
        uart_out32(ULITE_CONTROL_IE, ULITE_CONTROL, port);

    if (locked)
        spin_unlock_irqrestore(&port->lock, flags);
}

static int ulite_console_setup(struct console *co, char *options)
{
    struct uart_port *port = NULL;

    int baud = 9600;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    if (co->index < 0 || co->index >= ZYNQ_UART_NR_PORTS)
        return -EINVAL;

    if (!zynq_card) return -ENODEV;

    //port = &ulite_ports[co->index];
    port = &(zynq_card->uarts[co->index].port);

    if (!port) return -ENODEV;

    /* Has the device been initialized yet? */
    if (!port->mapbase) {
        zynq_printk(0, "console on ttyUL%i not present\n", co->index);
        return -ENODEV;
    }

    /* not initialized yet? */
    if (!port->membase) {
        return -ENODEV;
    }

    if (options)
        uart_parse_options(options, &baud, &parity, &bits, &flow);

    return uart_set_options(port, co, baud, parity, bits, flow);
}


#endif /* CONFIG_SERIAL_ZYNQ_CONSOLE */

///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef VIRTUAL_UART_DEVICE

#define MAX_RX_QUQUE_SIZE 16
static int heads[ZYNQ_UART_NR_PORTS];
static int tails[ZYNQ_UART_NR_PORTS];
static char rx_queues[ZYNQ_UART_NR_PORTS][MAX_RX_QUQUE_SIZE];	// The queue
static struct mutex rx_queue_locks[ZYNQ_UART_NR_PORTS];

int isEmpty(int port_id)
{
    return (heads[port_id] == tails[port_id]);
}

int isFull(int port_id)
{
    return ((tails[port_id] - MAX_RX_QUQUE_SIZE) == heads[port_id]);
}
void InitQueue(int port_id)
{

    int  i = 0;

    if (port_id >=  ZYNQ_UART_NR_PORTS) return;

    heads[port_id] = tails[port_id] = -1;
    mutex_init(&rx_queue_locks[port_id]);
    for (i = 0; i < MAX_RX_QUQUE_SIZE; i++) rx_queues[port_id][i] = 0;
}

void ClearQueue(int port_id)
{

    if (port_id >=  ZYNQ_UART_NR_PORTS) return;

    heads[port_id] = tails[port_id] = -1; // Reset indices to start over
    mutex_destroy(&rx_queue_locks[port_id]);
}

int Enqueue(int port_id, char ch)
{
    // Check to see if the Queue is full

    if (port_id >=  ZYNQ_UART_NR_PORTS) return -1;

    if(isFull(port_id)) return -1;

    mutex_lock(&rx_queue_locks[port_id]);
    // Increment tail index
    tails[port_id]++;
    // Add the item to the Queue
    rx_queues[port_id][tails[port_id] % MAX_RX_QUQUE_SIZE] = ch;
    mutex_unlock(&rx_queue_locks[port_id]);
    return 0;
}

int Dequeue(int port_id, char *ch)
{

    if (port_id >=  ZYNQ_UART_NR_PORTS) return -1;
    // Check for empty Queue
    if(isEmpty(port_id))
        return -1;  // Return null character if queue is empty
    else {
        heads[port_id]++;
        *ch = rx_queues[port_id][heads[port_id] % MAX_RX_QUQUE_SIZE];		// Get character to return
        return 0;				// Return popped character
    }
}

static void tiny_tx_char(struct uart_port *port)
{

    struct zynq_uart_t  *up = port_to_zynq_uart(port);
    struct circ_buf *xmit  = &port->state->xmit;
    //zynq_printk(1,"[zynq_uart]Enter  tiny_tx_char()\n");
    if (port->x_char) {
        port->x_char = 0;
        port->icount.tx++;
        return;
    }

    if (uart_circ_empty(xmit) || uart_tx_stopped(port))
        return;

    zynq_printk(1,"wrote: [%d] = %2x\n",xmit->tail, xmit->buf[xmit->tail]);
    if (up != NULL) Enqueue(up->idx, xmit->buf[xmit->tail]);
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
    port->icount.tx++;


    /* wake up */
    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);

    //zynq_printk(1,"[zynq_uart]Leave  tiny_tx_char()\n");

    return;
}


static void tiny_rx_char(struct uart_port *port)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);
    struct tty_port *tport = &port->state->port;
    char ch = 0;
    int ret = -1;
    char flag = TTY_NORMAL;
    //zynq_printk(1,"[zynq_uart]Enter  tiny_rx_char()\n");

    if (!up) return;

    ret = Dequeue(up->idx, &ch);

    if (ret == -1) {
        zynq_printk(0,"[zynq_uart]rx quque is empty!!\n");
        goto exit;
    }
    zynq_printk(1,"read:  = %2x\n",ch);
    port->icount.rx++;

    tty_insert_flip_char(tport, ch, flag);
    tty_flip_buffer_push(&port->state->port);

exit:
    //zynq_printk(1,"[zynq_uart]Leave  tiny_rx_char()\n");
}


#if 1
static void tiny_timer(unsigned long data)
{
    struct uart_port *port;
    port = (struct uart_port *)data;
    if (!port)
        return;

    /* resubmit the timer again */
    mod_timer(&timer, jiffies + DELAY_TIME);
    /* see if we have any data to transmit */
    tiny_rx_char(port);
}
#endif

static void tiny_set_termios(struct uart_port *port,
                             struct ktermios *new, struct ktermios *old)
{
    int baud, quot, cflag = new->c_cflag;

    //zynq_printk(1,"[zynq_uart]Enter  tiny_set_termios()\n");
    /* get the byte size */
    switch (cflag & CSIZE) {
        case CS5:
            zynq_printk(1," - data bits = 5\n");
            break;
        case CS6:
            zynq_printk(1, " - data bits = 6\n");
            break;
        case CS7:
            zynq_printk(1," - data bits = 7\n");
            break;
        default: // CS8
            zynq_printk(1," - data bits = 8\n");
            break;
    }

    /* determine the parity */
    if (cflag & PARENB)
        if (cflag & PARODD)
            zynq_printk(1," - parity = odd\n");
        else
            zynq_printk(1," - parity = even\n");
    else
        zynq_printk(1," - parity = none\n");

    /* figure out the stop bits requested */
    if (cflag & CSTOPB)
        zynq_printk(1," - stop bits = 2\n");
    else
        zynq_printk(1," - stop bits = 1\n");

    /* figure out the flow control settings */
    if (cflag & CRTSCTS)
        zynq_printk(1," - RTS/CTS is enabled\n");
    else
        zynq_printk(1," - RTS/CTS is disabled\n");

    /* Set baud rate  default is 9600*/
    baud = uart_get_baud_rate(port, new, old, 0, port->uartclk/16);

    quot = uart_get_divisor(port, baud);

    zynq_printk(1,"[zynq_uart]baud_rate:%d, divisor:%d\n", baud, quot);
    //zynq_printk(1,"[zynq_uart]Leave  tiny_set_termios()\n");

}
static int tiny_start_rx(struct uart_port *port)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return -1;

    //zynq_printk(1,"[zynq_uart]Enter  tiny_start_rx()\n");
#if 1
    /* create our timer and submit it */
    init_timer(&timer);
    timer.data = (unsigned long)port;
    timer.expires = jiffies + DELAY_TIME;
    timer.function = tiny_timer;
    InitQueue(up->idx);
    add_timer(&timer);
#endif
    //zynq_printk(1,"[zynq_uart]Leave  tiny_start_rx()\n");
    return 0;
}
static void tiny_stop_rx(struct uart_port *port)
{
    struct zynq_uart_t  *up = port_to_zynq_uart(port);

    if (!up) return;

    //zynq_printk(1,"[zynq_uart]Enter  tiny_stop_rx()\n");
#if 1
    del_timer(&timer);
    ClearQueue(up->idx);
#endif
    //zynq_printk(1,"[zynq_uart]Leave  tiny_stop_rx()\n");
}
#endif
