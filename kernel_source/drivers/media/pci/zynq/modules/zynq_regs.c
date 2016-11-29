#include <linux/io.h>
#include "zynq_regs.h"
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
void zynq_pci_reg_write(void __iomem *base, u32 reg, u32 val)
{
#ifndef IS_VIRTUAL_FPGA_DEV
    //iowrite32((val), (base)+(reg));
#endif
}


u32 zynq_pci_reg_read(void __iomem *base, u32  reg)
{
    u32  ret = 0;
#ifndef IS_VIRTUAL_FPGA_DEV
    //ret =  ioread32((base)+(reg));
#endif
    return ret;
}

void zynq_pci_reg_rmw(void __iomem *base, u32 reg, u32 clr_bits, u32 set_bits)
{
    u32 tmp = (u32)zynq_pci_reg_read(base, reg);
    tmp &= ~clr_bits;
    tmp |= set_bits;
    zynq_pci_reg_write(base, reg, tmp);
}

void zynq_pci_reg_rmw_clr(void __iomem *base, u32 reg, u32 val)
{
    zynq_pci_reg_rmw(base, reg, val, 0);
}

void zynq_pci_reg_rmw_set(void __iomem *base, u32 reg, u32 val)
{
    zynq_pci_reg_rmw(base, reg, 0, val);
}
