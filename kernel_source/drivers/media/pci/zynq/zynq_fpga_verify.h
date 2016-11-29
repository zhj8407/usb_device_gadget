#ifndef ZYNQ_FPGA_VERIFY_H
#define ZYNQ_FPGA_VERIFY_H

#define  FPGA_VERIF  1

//Global

#define FPGA_LOGIC_VERSION_REG 0x0000
#define FPGA_COMPILE_TIME_REG  0x0004

#define FPGA_UART_WORKING_MODE_REG 0x0058
#define FPGA_UART_BAUD_RATE_MODE_REG 0x005c

#define FPGA_AUDIO_PERIOD_SIZE_REG 				0x0a00
#define FPGA_AUDIO_READY_REG 		0x0a04 //Audio Ready
#define FPGA_AUDIO_PCM_BUFFER_REG  				0x0010

#define FPGA_PCI_PUSH_READY_REG 0x0020
#define FPGA_AUDIO_PCI_PUSH_READY_MASK 	  	0x00000010  //Audio
#define FPGA_VIDEO0_PCI_PUSH_READY_MASK 	0x00000001 //VIN_0 (Left Eagle eye)
#define FPGA_VIDEO1_PCI_PUSH_READY_MASK 	0x00000002 //VIN_1 (Right Eagle eye)
#define FPGA_VIDEO2_PCI_PUSH_READY_MASK 	0x00000004 //WEBCAM
#define FPGA_VIDEO3_PCI_PUSH_READY_MASK 	0x00000008 //Color bar
#define FPGA_VIDEO6_PCI_PUSH_READY_MASK 	0x00000020 //OSD video
#define FPGA_VIDEO7_PCI_PUSH_READY_MASK 	0x00000040 //USB video

#define FPGA_INTERRUPT_REG 0x1200
#define FPGA_AUDIO_INTERRUPT_MASK 		0x00100000 //Audio
#define FPGA_VIDEO0_INTERRUPT_MASK 	0x00010000 //VIN_0 (Left Eagle eye)
#define FPGA_VIDEO1_INTERRUPT_MASK 	0x00020000  //VIN_1 (Right Eagle eye)
#define FPGA_VIDEO2_INTERRUPT_MASK		0x00040000 //WEBCAM
#define FPGA_VIDEO3_INTERRUPT_MASK 	0x00080000 //Color bar
#define FPGA_VIDEO6_INTERRUPT_MASK     0x00200000 //OSD video
#define FPGA_VIDEO7_INTERRUPT_MASK     0x00400000  //USB video
#define FPGA_DMA_INTERRUPT_MASK_ER    	0x00800000 // FPGA DMA read (for ER board)
#define FPGA_DMA_INTERRUPT_MASK_SR    	0x00200000 //  FPGA DMA read (for SR board)

#define FPGA_AUDIO_INTERRUPT_ST_MASK 	0x00000010  //Audio
#define FPGA_VIDEO0_INTERRUPT_ST_MASK 	0x00000001 //VIN_0 (Left Eagle eye)
#define FPGA_VIDEO1_INTERRUPT_ST_MASK 	0x00000002  //VIN_1 (Right Eagle eye)
#define FPGA_VIDEO2_INTERRUPT_ST_MASK 	0x00000004  //WEBCAM
#define FPGA_VIDEO3_INTERRUPT_ST_MASK 	0x00000008  //Color bar
#define FPGA_VIDEO6_INTERRUPT_ST_MASK 	0x00000020 //OSD video
#define FPGA_VIDEO7_INTERRUPT_ST_MASK 	0x00000040 //USB video
#define FPGA_DMA_INTERRUPT_ST_MASK_ER 		0x00000080 // FPGA DMA read (for ER board)
#define FPGA_DMA_INTERRUPT_ST_MASK_SR 		0x00000020 //  FPGA DMA read (for SR board)


#define FPGA_WEBCAM_VIDEO_RES_REG 0x121c
#define FPGA_WEBCAM_VIDEO_RES_WIDTH_OFFSET 16
#define FPGA_WEBCAM_VIDEO_RES_WIDTH_MASK 0xffff0000
#define FPGA_WEBCAM_VIDEO_RES_HEIGHT_MASK 0x0000ffff

#define FPGA_VIDEO_RES_REG 0x1204
#define FPGA_VIDEO_RES_WIDTH_OFFSET 16
#define FPGA_VIDEO_RES_WIDTH_MASK 0xffff0000
#define FPGA_VIDEO_RES_HEIGHT_MASK 0x0000ffff

#define FPGA_VIDEO_READY_REG 		0x120c
#define FPGA_AUDIO_READY_MASK 	0x00000010  //Audio Ready (it is  alias to "FPGA_AUDIO_READY_REG (0x0a04)")
#define FPGA_VIDEO0_READY_MASK 	0x00000001 //VIN_0 (Left Eagle eye)
#define FPGA_VIDEO1_READY_MASK 	0x00000002 //VIN_1 (Right Eagle eye)
#define FPGA_VIDEO2_READY_MASK 	0x00000004 //WEBCAM
#define FPGA_VIDEO3_READY_MASK 	0x00000008  //Color bar
#define FPGA_VIDEO6_READY_MASK 	0x00000020 //OSD video
#define FPGA_VIDEO7_READY_MASK 	0x00000040 //USB video

#define FPGA_VIDEO0_Y_ADDR_REG 	 0x0024
#define FPGA_VIDEO0_UV_ADDR_REG  0x0028
#define FPGA_VIDEO1_Y_ADDR_REG 	0x002C
#define FPGA_VIDEO1_UV_ADDR_REG 0x0030
#define FPGA_VIDEO2_Y_ADDR_REG 	0x0034
#define FPGA_VIDEO2_UV_ADDR_REG  0x0038
#define FPGA_VIDEO3_Y_ADDR_REG 	0x003C
#define FPGA_VIDEO3_UV_ADDR_REG  0x0040
#define FPGA_VIDEO6_Y_ADDR_REG 	0x0044
#define FPGA_VIDEO6_UV_ADDR_REG  0x0048
#define FPGA_VIDEO7_Y_ADDR_REG 	0x004c
#define FPGA_VIDEO7_UV_ADDR_REG  0x0070

#define FPGA_MIC_PLAYBACK_SELECTION 0x0074 //The bit4~bit0 are corresponding to the selections of  mic4 ~ mic0.

#define FPGA_VIDEO_STREAM_ENABLE_REG 0x0050
#define FPGA_VIDEO_OUT_CLOCK_SELECTION_REG 0x0054

//VMDA
#define  FPGA_VDMA0_REG 0x0c00 //0x0c00  ~ 0x0dff
#define  FPGA_VDMA1_REG 0x0e00 //0x0e00 ~ 0x0fff
#define  FPGA_VDMA2_REG 0x1000 //0x1000 ~ 0x11ff
#define  FPGA_VDMA3_REG 0x1400 //0x1400 ~ 0x15ff

//OSD
#define  FPGA_OSD0_REG 0x1e00 //0x1e00  ~ 0x1eff
#define  FPGA_OSD1_REG 0x2000 //0x2000 ~ 0x21ff

//SCALER
#define  FPGA_SCALER0_REG 0x1600 //0x1600  ~ 0x17ff
#define  FPGA_SCALER1_REG 0x1800 //0x1800 ~ 0x19ff
#define  FPGA_SCALER2_REG 0x1a00 //0x1a00 ~ 0x1bff
#define  FPGA_SCALER3_REG 0x1c00 //0x1c00 ~ 0x1dff
#define  FPGA_SCALER4_REG 0x2800 //0x2800 ~ 0x29ff

//Chroma Resampler
#define  FPGA_CHROMA_RESAMPLER0_REG 0x2200 //0x2200  ~ 0x23ff
#define  FPGA_CHROMA_RESAMPLER1_REG 0x2400 //0x2400 ~ 0x25ff
#define  FPGA_CHROMA_RESAMPLER2_REG 0x2600 //0x2600 ~ 0x27ff
#define  FPGA_CHROMA_RESAMPLER3_REG 0x2800 //0x2800 ~ 0x29ff
#define  FPGA_CHROMA_RESAMPLER4_REG 0x2a00 //0x2a00 ~ 0x2bff

//Video Selector
#define FPGA_VSELECTOR_REG 0x3000 //0x3000 ~ 0x31ff

//Video Timing Controller
#define FPGA_VIDEO_TIMING_CONTROLLER0_REG 0x2c00 //0x2c00~0x2dff
#define FPGA_VIDEO_TIMING_CONTROLLER1_REG 0x2e00 //0x2e00~0x2fff

//Diagnostic
#define FPGA_DIAGNOSTIC_REG 0x3200 //0x3200 ~ 0x33ff

//SYS_Monitor
#define FPGA_SYS_MONITOR_REG 0x3400 //0x3400 ~ 0x35ff

//FPGA DMA
#define FPGA_DMA_ADDR_REG   0x1210
#define FPGA_DMA_RES_REG  0x1214
#define FPGA_DMA_RES_WIDTH_OFFSET 16
#define FPGA_DMA_RES_WIDTH_MASK 0xffff0000
#define FPGA_DMA_RES_HEIGHT_MASK 0x0000ffff
#define FPGA_DMA_READ_ENABLE_REG   0x1218

void fpga_reg_write(void __iomem *base, u32 reg, u32 val);
u32 fpga_reg_read(void __iomem *base, u32  reg);
void fpga_reg_rmw(void __iomem *base, u32 reg, u32 clr_bits, u32 set_bits);
void fpga_reg_rmw_clr(void __iomem *base, u32 reg, u32 val);
void fpga_reg_rmw_set(void __iomem *base, u32 reg, u32 val);


void fpga_reg_write_be(void __iomem *base, u32 reg, u32 val);
u32 fpga_reg_read_be(void __iomem *base, u32  reg);
void fpga_reg_rmw_be(void __iomem *base, u32 reg, u32 clr_bits, u32 set_bits);
void fpga_reg_rmw_clr_be(void __iomem *base, u32 reg, u32 val);
void fpga_reg_rmw_set_be(void __iomem *base, u32 reg, u32 val);

#endif