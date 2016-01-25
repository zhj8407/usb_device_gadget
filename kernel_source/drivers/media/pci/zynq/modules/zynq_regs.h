#ifndef ZYNQ_REGS_H
#define ZYNQ_REGS_H


#define IS_VIRTUAL_FPGA_DEV 1

void zynq_pci_reg_write(void __iomem *base, u32 reg, u32 val) ;

u32 zynq_pci_reg_read(void __iomem *base, u32  reg);

void zynq_pci_reg_rmw(void __iomem *base, u32 reg, u32 clr_bits, u32 set_bits);

void zynq_pci_reg_rmw_clr(void __iomem *base, u32 reg, u32 val);

void zynq_pci_reg_rmw_set(void __iomem *base, u32 reg, u32 val);

#define ZYNQ_REG_LEN 0x4
////////////////////////////////////////////////////////////////////////////////////////////////
/* Channel Related Registers */

#define ZYNQ_CH_CFG (0x3000)
#define ZYNQ_CH_REG_NUM (0x3)

#define ZYNQ_CH_CFG_CH(i)\
	(ZYNQ_CH_CFG  + (i * ZYNQ_REG_LEN * ZYNQ_CH_REG_NUM))
#define ZYNQ_CH0_CTRL_CH(i) \
	(ZYNQ_CH_CFG_CH(i) + 0)
#define ZYNQ_CH0_Y_START_ADDR_CH(i)\
	(ZYNQ_CH_CFG_CH(i) + (ZYNQ_REG_LEN * 0x1))
#define ZYNQ_CH0_UV_START_ADDR_CH(i)\
	(ZYNQ_CH_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x2))
////////////////////////////////////////////////////////////////////////////////////////////////
/*UART related Registers*/
#define ZYNQ_UART_CFG (0x0000)
#define ZYNQ_SCALER_REG_NUM (0x80)
////////////////////////////////////////////////////////////////////////////////////////////////
/*Audio switch related Registers*/
#define ZYNQ_AUIDO_SWITCH_CFG (0x0200)
#define ZYNQ_AUIDO_SWITCH_REG_NUM (0x80)
////////////////////////////////////////////////////////////////////////////////////////////////
/*Scaler related Registers*/
#define ZYNQ_SCALER_CFG (0x0400)
#define ZYNQ_SCALER_REG_NUM (0x80)

#define ZYNQ_SCALER_CFG_CH(i) \
	(ZYNQ_SCALER_CFG + (i * ZYNQ_REG_LEN * ZYNQ_SCALER_REG_NUM))
#define ZYNQ_SCALER_CTRL_CH(i) \
	(ZYNQ_SCALER_CFG_CH(i) + 0)
#define ZYNQ_SCALER_STATUS_CH(i) \
	(ZYNQ_SCALER_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x1))
#define ZYNQ_SCALER_ERROR_CH(i) \
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x2))
#define ZYNQ_SCALER0_HSF_CH(i) \
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x40))
#define ZYNQ_SCALER_VSF_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x41))
#define ZYNQ_SCALER_SOURCE_VIDEO_SIZE_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x42))
#define ZYNQ_SCALER_SOURCE_H_APERTURE_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x43))
#define ZYNQ_SCALER_SOURCE_V_APERTURE_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x44))
#define ZYNQ_SCALER_SOURCE_OUTPUT_SIZE_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x45))
#define ZYNQ_SCALER_NUM_PHASES_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x46))
#define ZYNQ_SCALER_ACTIVE_COEFS_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x47))
#define ZYNQ_SCALER_HPA_Y_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x48))
#define ZYNQ_SCALER_HPA_C_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x49))
#define ZYNQ_SCALER_VPA_Y_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x4a))
#define ZYNQ_SCALER_VPA_C_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x4b))
#define ZYNQ_SCALER_COEF_SET_WR_ADDR_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x4c))
#define ZYNQ_SCALER_COEF_DATA_IN_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x4d))
#define ZYNQ_SCALER_COEF_SET_RD_ADDR_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x4e))
#define ZYNQ_SCALER_COEF_MEM_RD_ADDR_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x4f))
#define ZYNQ_SCALER_COEF_MEM_OUTPUT_CH(i)\
	(ZYNQ_SCALER_CFG_CH(i) +  (ZYNQ_REG_LEN * 0x50))
////////////////////////////////////////////////////////////////////////////////////////////////
/*Video selector related Registers*/
#define ZYNQ_VIDEO_SELECTOR_CFG (0x0c00)
#define ZYNQ_VIDEO_SELECTOR_REG_NUM (0x80)

////////////////////////////////////////////////////////////////////////////////////////////////
/*VDMA related Registers*/

#define ZYNQ_VDMA0_REG 0x0c00 //0x0c00  ~ 0x0dff
#define ZYNQ_VDMA1_REG 0x0e00 //0x0e00 ~ 0x0fff
#define ZYNQ_VDMA2_REG 0x1000 //0x1000 ~ 0x11ff
#define ZYNQ_VDMA3_REG 0x1400 //0x1400 ~ 0x15ff

////////////////////////////////////////////////////////////////////////////////////////////////
/*OSD related Registers*/
#define ZYNQ_OSD_CFG (0x1600)
#define ZYNQ_OSD_REG_NUM (0x80)
#define ZYNQ_OSD_CFG_CH(i) \
	(ZYNQ_OSD_CFG + (i * ZYNQ_REG_LEN *  ZYNQ_OSD_REG_NUM))
#define ZYNQ_OSD_CTRL_CH(i) \
	(ZYNQ_OSD_CFG_CH(i) + 0)
#define ZYNQ_OSD_OUTPUT_ACTIVE_SIZE(i) \
	(ZYNQ_OSD_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x8))
#define ZYNQ_OSD_LAYER_0_CONTROL(i) \
	(ZYNQ_OSD_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x44))
#define ZYNQ_OSD_LAYER_0_POSITION(i) \
	(ZYNQ_OSD_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x45))
#define ZYNQ_OSD_LAYER_0_SIZE(i) \
	(ZYNQ_OSD_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x46))
#define ZYNQ_OSD_LAYER_1_CONTROL(i) \
	(ZYNQ_OSD_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x48))
#define ZYNQ_OSD_LAYER_1_POSITION(i) \
	(ZYNQ_OSD_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x49))
#define ZYNQ_OSD_LAYER_1_SIZE(i) \
	(ZYNQ_OSD_CFG_CH(i)+ (ZYNQ_REG_LEN * 0x4a))
		
////////////////////////////////////////////////////////////////////////////////////////////////
/* Chroma Resampler Registers*/
#define ZYNQ_RESAMPLER_CFG (0x1a00)
#define ZYNQ_RESAMPLER_REG_NUM (0x80)
#define ZYNQ_RESAMPLER_CFG_CH(i) \
		(ZYNQ_RESAMPLER_CFG + (i * ZYNQ_REG_LEN *  ZYNQ_RESAMPLER_REG_NUM))

////////////////////////////////////////////////////////////////////////////////////////////////
/*Video Timing Controller related Registers*/
#define ZYNQ_VIDEO_TIMING_CONTROLLER_CFG (0x2400)
#define ZYNQ_VIDEO_TIMING_CONTROLLER_NUM (0x80)

////////////////////////////////////////////////////////////////////////////////////////////////
/*Diagnostic related Registers*/
#define ZYNQ_DIAGNOSTIC_CFG (0x2600)
#define ZYNQ_DIAGNOSTIC_NUM (0x80)

////////////////////////////////////////////////////////////////////////////////////////////////
/*GOLBAL related Registers*/
#define ZYNQ_GLOBAL_CFG (0x2800)
#define ZYNQ_GLOBAL_NUM (0x80)

#endif