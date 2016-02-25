#include <linux/io.h>
#include "zynq_fpga_verify.h"
#include "zynq_debug.h"
#include "zynq_core.h"
/*
 * http://www.makelinux.net/ldd3/chp-9-sect-4:
 *
 * On some platforms, you may get away with using the return value from ioremap as a pointer.
 * Such use is not portable, and, increasingly, the kernel developers have been working to eliminate any such use.
 * The proper way of getting at I/O memory is via a set of functions (defined via <asm/io.h>) provided for that purpose.
 * To read from I/O memory, use one of the following:
 * unsigned int ioread8(void *addr);
 * unsigned int ioread16(void *addr);
 * unsigned int ioread32(void *addr);
 * Here, addr should be an address obtained from ioremap (perhaps with an integer offset); the return value is what was read from the given I/O memory.
 */
void fpga_reg_write(void __iomem *base, u32 reg, u32 val)
{

#ifdef FPGA_VERIF
    if (!base) return;
    //zynq_printk(0, "[0x%x]: 0x%x \n", (unsigned int) reg, (unsigned int)val);
    if (reg <  zynq_reg_len) {
        iowrite32((val), (base)+(reg));
    } else {
        zynq_printk(0, "[fpga_reg_verify]Write error!! (0x%x (reg) > 0x%x (len))\n", (unsigned int)reg, (unsigned int)zynq_reg_len);
    }
#endif
}


u32 fpga_reg_read(void __iomem *base, u32  reg)
{
    u32  ret = 0;
#ifdef FPGA_VERIF
    if (!base) return -1;
    if (reg <  zynq_reg_len) {
        ret =  ioread32((base)+(reg));
    } else {
        zynq_printk(0, "[fpga_reg_verify]Read error!! (0x%x (reg) > 0x%x (len))\n", (unsigned int)reg, (unsigned int)zynq_reg_len);
    }
#endif
    return ret;
}

void fpga_reg_rmw(void __iomem *base, u32 reg, u32 clr_bits, u32 set_bits)
{
    u32 tmp = (u32)fpga_reg_read(base, reg);
    tmp &= ~clr_bits;
    tmp |= set_bits;
    fpga_reg_write(base, reg, tmp);
}

void fpga_reg_rmw_clr(void __iomem *base, u32 reg, u32 val)
{
    fpga_reg_rmw(base, reg, val, 0);
}

void fpga_reg_rmw_set(void __iomem *base, u32 reg, u32 val)
{
    fpga_reg_rmw(base, reg, 0, val);
}
////////////////////////////////////////////////////////////////////////

void fpga_reg_write_be(void __iomem *base, u32 reg, u32 val)
{

#ifdef FPGA_VERIF
    if (!base) return;
    //zynq_printk(0, "[0x%x]: 0x%x \n", (unsigned int) reg, (unsigned int)val);
    if (reg <  zynq_reg_len) {
        iowrite32be((val), (base)+(reg));
    } else {
        zynq_printk(0, "[fpga_reg_verify]Write error!! (0x%x (reg) > 0x%x (len))\n", (unsigned int)reg, (unsigned int)zynq_reg_len);
    }
#endif
}


u32 fpga_reg_read_be(void __iomem *base, u32  reg)
{
    u32  ret = 0;
#ifdef FPGA_VERIF
    if (!base) return -1;
    if (reg <  zynq_reg_len) {
        ret =  ioread32be((base)+(reg));
    } else {
        zynq_printk(0, "[fpga_reg_verify]Read error!! (0x%x (reg) > 0x%x (len))\n", (unsigned int)reg, (unsigned int)zynq_reg_len);
    }
#endif
    return ret;
}

void fpga_reg_rmw_be(void __iomem *base, u32 reg, u32 clr_bits, u32 set_bits)
{
    u32 tmp = (u32)fpga_reg_read_be(base, reg);
    tmp &= ~clr_bits;
    tmp |= set_bits;
    fpga_reg_write_be(base, reg, tmp);
}

void fpga_reg_rmw_clr_be(void __iomem *base, u32 reg, u32 val)
{
    fpga_reg_rmw_be(base, reg, val, 0);
}

void fpga_reg_rmw_set_be(void __iomem *base, u32 reg, u32 val)
{
    fpga_reg_rmw_be(base, reg, 0, val);
}



