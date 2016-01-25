#zynq-objs := zynq-cards.o zynq-video.o zynq-core.o zynq-alsa.o

#zynq-objs := vpif.o vpif_capture.o vpif_driver.o vpif_capture_virtual_dev.o

zynq-objs := zynq_core.o zynq_board.o zynq_capture.o zynq_driver.o  zynq_debug.o zynq_display.o zynq_malloc.o  zynq_audio.o zynq_audio_virtual_dev.o zynq_uart.o zynq_fpga_verify.o\
			modules/zynq_regs.o \
			modules/zynq_scaler.o \
			modules/zynq_video_selector.o \
			modules/zynq_video_timing_controller.o\
			modules/zynq_resampler.o\
			modules/zynq_osd.o\
			modules/zynq_vdma.o\
			zynq_control.o\
			zynq_diagnostic.o

pcm1865-objs := zynq_pcm1865.o

fakepci.objs :=  fakepci.o
fakei2cmaster.objs := fakei2cmaster.o

obj-$(CONFIG_VIDEO_ZYNQ) += zynq.o
obj-$(CONFIG_VIDEO_ZYNQ) += pcm1865.o
obj-$(CONFIG_VIDEO_ZYNQ) += fakepci.o
obj-$(CONFIG_VIDEO_ZYNQ) += fakei2cmaster.o

