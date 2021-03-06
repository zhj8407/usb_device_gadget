#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <linux/mutex.h>

#include "zynq_core.h"
#include "zynq_debug.h"
#include "zynq_board.h"
#include "zynq_control.h"
#include "zynq_types.h"

#include "modules/zynq_scaler.h"
#include "modules/zynq_osd.h"
#include "modules/zynq_video_selector.h"
#include "modules/zynq_video_timing_controller.h"
#include "modules/zynq_resampler.h"
#include "modules/zynq_vdma.h"

//#define FPGA_MODULE_SUPPORT_DATE (0x20151027)
#define FPGA_MODULE_SUPPORT_DATE (0x20151125)

/////////////////////////////////////////////////////////////////////////////
unsigned int en_modules = 1;
module_param(en_modules, int, 0644);


static unsigned int g_is_pipeline_initialized = 0;
static struct v4l2_vout_pipeline g_vout_pipeline_config;
static struct mutex g_config_lock;
static unsigned int b_firstconfig_vout_pipeline = 1;

//Only could be 1920x1080 (1080p) or 1280x720 (720p)
static  unsigned int default_in_width = 1920;
static  unsigned int default_in_height = 1080;
static unsigned  int  default_frame_rate = 60;

static atomic_t g_is_vin0_valid;
static atomic_t g_is_vin1_valid;
static atomic_t g_is_vin2_valid;
static atomic_t g_is_cpuin_valid;


static u32 fpga_release_date = 0;
static int config_all_video_pipeline_entities(void __iomem *pci_reg_base);
//static int dump_all_video_pipeline_entities(void __iomem *pci_reg_base) ;
static int stop_all_video_pipeline_entities(void __iomem *pci_reg_base);
static int start_all_video_pipeline_entities(void __iomem *pci_reg_base);
static int release_all_video_pipeline_entities(void __iomem *pci_reg_base);
static int init_all_video_pipeline_entities(void __iomem *pci_reg_base);
static int create_register_dump_sysfs(struct kobject *kobj);
static int destroy_register_dump_sysfs(struct kobject *kobj);
/////////////////////////////////////////////////////////////////////////////

static unsigned int is_initial_vpif_obj = 0;
struct vpif_control_device vpif_obj; 
struct video_device *vpif_video_dev = NULL;
static struct device *vpif_dev = NULL;

////////////////////////////////////////////////////////////////////////////////////////
static int vpif_querycap(struct file *file, void  *priv, struct v4l2_capability *cap);
static int vpif_g_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl);
static int vpif_s_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl);
static int vpif_vout_pipline(struct file *file, void *fh, struct v4l2_vout_pipeline *a);
static int vpif_scaler_crop(struct file *file, void *fh, struct v4l2_scaler_crop *a);

static int vpif_mmap(struct file *filep, struct vm_area_struct *vma);
static int vpif_open(struct file *filep) ;
static int vpif_release(struct file *filep);
static unsigned int vpif_poll(struct file *filep, poll_table * wait);
///////////////////////////////////////////////////////////////////////////////////////

//
//	int (*vidioc_vout_pipeline)       	(struct file *file, void *fh, struct v4l2_vout_pipeline *a);
	
//	int (*vidioc_scaler_crop)       	(struct file *file, void *fh, struct v4l2_scaler_crop *a);
static const struct v4l2_ioctl_ops vpif_ioctl_ops = {
	.vidioc_querycap        	= vpif_querycap,
	.vidioc_s_ctrl                      =  vpif_s_ctrl,
	.vidioc_g_ctrl                      = vpif_g_ctrl,
	.vidioc_vout_pipeline = vpif_vout_pipline,
	.vidioc_scaler_crop = vpif_scaler_crop
};

static const struct v4l2_file_operations vpif_fops = {
	.owner = THIS_MODULE,
	.open = vpif_open,
	.release = vpif_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vpif_mmap,
	.poll = vpif_poll
};

static struct video_device vpif_video_template = {
	.name		= "vpif-control",
	.fops		= &vpif_fops,
	.minor		= -1,
	.ioctl_ops	= &vpif_ioctl_ops,
};

static int vpif_control_config_out_clock(eVideoFormat format) {
	
	u32 val = 0;
	
	if (!zynq_reg_base) return -1;
	
	if ((fpga_release_date >= FPGA_MODULE_SUPPORT_DATE) && (en_modules == 1)) {
		val = fpga_reg_read(zynq_reg_base,FPGA_VIDEO_OUT_CLOCK_SELECTION_REG);
		switch(format) {
			case E1080P60FMT:
			case E1080P50FMT:	
				fpga_reg_write(zynq_reg_base, FPGA_VIDEO_OUT_CLOCK_SELECTION_REG, (val&0xfffffffe) | 0x0); break;
			case E720P60FMT:
			case E720P50FMT:	
				fpga_reg_write(zynq_reg_base, FPGA_VIDEO_OUT_CLOCK_SELECTION_REG, (val&0xfffffffe) | 0x1); break;
			default:
				return  0;
		}
	}
	return 0;
}

static int vpif_control_enable_video_stream_by_id(ePIPEPORTID id, unsigned int enable) {
	
	u32 val = 0;
	
	if (!zynq_reg_base) return -1;
	
	if ((fpga_release_date >= FPGA_MODULE_SUPPORT_DATE) && (en_modules == 1)) {
		
		val = fpga_reg_read(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG);
		switch (id) {
			case EVIN0:
				if (enable)
					fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffffe) | 0x1);
				else
					fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffffe) | 0x0);
				break;
			case EVIN1:
				if (enable)
					fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffffd) | 0x2);
				else
					fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffffd) | 0x0);
				break;
			case EVIN2:
				if (enable)
					fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffffb) | 0x4);
				else
					fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffffb) | 0x0);
				break;
			default:
				return 0;
		}
		val =  fpga_reg_read(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG);
		//zynq_printk(0, "[zynq_control](%d) enable video stream reg: 0x%08x\n", __LINE__, val);
	}
	return 0;
}

static int vpif_control_enable_video_streams(unsigned int enable){
	
	u32 val = 0;
	
	if (!zynq_reg_base) return -1;
	
	if ((fpga_release_date >= FPGA_MODULE_SUPPORT_DATE) && (en_modules == 1)) {
	
		val = fpga_reg_read(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG);
	
		if (enable)
			fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffff8) | 0x7);
		else
			fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffff8) | 0x0);
	
		val =  fpga_reg_read(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG);
		//	zynq_printk(0, "[zynq_control](%d) enable video stream reg: 0x%08x\n", __LINE__, val);
	}
	return  0;
}

int vpif_control_config_vin(ePIPEPORTID id, unsigned int enable) {
	
	switch (id) {
		case EVIN0:
				if (enable) {
					atomic_set(&g_is_vin0_valid, 0);
				} else {
					atomic_set(&g_is_vin0_valid, -1);
				}
				break;
		case EVIN1:
				if (enable) {
					atomic_set(&g_is_vin1_valid, 0);
				} else {
					atomic_set(&g_is_vin1_valid, -1);
				}
				break;
		case EVIN2:
				if (enable) {
					atomic_set(&g_is_vin2_valid, 0);
				} else {
					atomic_set(&g_is_vin2_valid, -1);
				}
				break;
		case ECPU:
				if (enable) {
					atomic_set(&g_is_cpuin_valid, 0);
				} else {
					atomic_set(&g_is_cpuin_valid, -1);
				}
				break;
		default:
			return 0;
	}
	
	return 0;
}

static unsigned int vpif_control_is_valid(ePIPEPORTID id) {
	
	
	switch (id) {
		case EVIN0:
				if (atomic_read(&g_is_vin0_valid) == -1) 
					return 0;
				else 
					return 1;
		case EVIN1:
				if (atomic_read(&g_is_vin1_valid) == -1)
					return 0;
				else
					return 1;
		case EVIN2:
			//TODO: The webcam always valid. Because the driver counld not detect the status of attching for webcam.
			/*
				if (atomic_read(&g_is_vin2_valid) == -1) 
					return 0;
				else
					return 1;*/
			return 1;	
		case ECPU:
			//TODO: The cpu input  always invalid. Because the FPGA function is not implemented now. 
			/*	if (atomic_read(&g_is_cpuin_valid) == -1)
					return 0;
				else
					return 1;*/
			return 0;
		default:
			return 0;
	}

}



int vpif_control_init_pipeline(struct pci_dev *pdev) {

	if (!pdev) return -1;
/////////////////////////////////////////////////////////////////////
	/*FPGA  initialize function*/
	fpga_release_date = fpga_reg_read(zynq_reg_base, FPGA_COMPILE_TIME_REG) ;
	if ((fpga_release_date >= FPGA_MODULE_SUPPORT_DATE) && (en_modules == 1)) {	
		
		zynq_printk(0, "[zynq_control] Enable FPGA modules.\n");	
		
		g_is_pipeline_initialized = 1;
		
		mutex_init(&g_config_lock);
		
	 	init_all_video_pipeline_entities(zynq_reg_base);
	 	config_all_video_pipeline_entities(zynq_reg_base);
	 	start_all_video_pipeline_entities(zynq_reg_base) ;
	 	//dump_all_video_pipeline_entities(zynq_reg_base);
	 	create_register_dump_sysfs(&pdev->dev.kobj);
		// vpif_control_enable_video_streams(1);
		atomic_set(&g_is_cpuin_valid, -1);
		atomic_set(&g_is_vin0_valid, -1);
		atomic_set(&g_is_vin1_valid, -1);
		atomic_set(&g_is_vin2_valid, -1);
		
	} else {
	 	zynq_printk(0, "[zynq_control] Disable FPGA modules. Because 0x%x > 0x%x\n", FPGA_MODULE_SUPPORT_DATE, fpga_release_date);	
	}
////////////////////////////////////////////////////////////////////
	return 0;
}
int vpif_control_release_pipeline(struct pci_dev *pdev){
	
	if (!pdev) return -1;
////////////////////////////////////////////////////////////////////
	/*FPGA  relase function*/
	if (zynq_reg_base != NULL) {
			if ((fpga_release_date >= FPGA_MODULE_SUPPORT_DATE) && (en_modules == 1)) {
				if (g_is_pipeline_initialized) {
					zynq_printk(0, "[zynq_control] Stop and release  FPGA modules.\n");	
					vpif_control_enable_video_streams(0);
					stop_all_video_pipeline_entities(zynq_reg_base);
					release_all_video_pipeline_entities(zynq_reg_base);
					destroy_register_dump_sysfs(&pdev->dev.kobj);
					mutex_destroy(&g_config_lock);
				}
			}
	}
////////////////////////////////////////////////////////////////////
	return 0;
}

int vpif_control_init(struct pci_dev *pdev) {
	
	struct video_device *vfd = NULL;
	int register_err = -1;
	int register_device_err = -1;
	
	vpif_dev = &pdev->dev;
	
	register_err = v4l2_device_register(vpif_dev, &vpif_obj.v4l2_dev);
	if (register_err) goto exit; 
	
	vfd = video_device_alloc();
	if (vfd == NULL)  goto exit ;
	
	*vfd = vpif_video_template;
	vfd->v4l2_dev = &vpif_obj.v4l2_dev;
	vfd->release = video_device_release;
	
	register_device_err = video_register_device(vfd,  VFL_TYPE_GRABBER, 0);
	
	if (register_device_err) goto exit;
	
	vpif_video_dev  = vfd;
	

	is_initial_vpif_obj = 1;
	
	v4l2_info(&vpif_obj.v4l2_dev, "[zynq_control] Zynq control driver initialized\n");
	
	return 0;
	
exit:

	if ((register_device_err == 0) && (vfd != NULL)) video_unregister_device(vfd);
	if (register_err == 0) v4l2_device_unregister(&vpif_obj.v4l2_dev);

	return -1;
}

int vpif_control_release(struct pci_dev *pdev) {
	
	if (is_initial_vpif_obj) {
		if (vpif_video_dev) video_unregister_device(vpif_video_dev);
		v4l2_device_unregister(&vpif_obj.v4l2_dev);
	}
	
	return 0;
}

struct vpif_control_device * vpif_control_get_instnace(void) {
	
	if ( is_initial_vpif_obj) 
		return  &vpif_obj;
	else 
		return NULL;
	
}

static int vpif_open(struct file *filep) {
	return 0;
}


static int vpif_release(struct file *filep) {
	return 0;
}

static int vpif_mmap(struct file *filep, struct vm_area_struct *vma) {
	return 0;
}


static unsigned int vpif_poll(struct file *filep, poll_table * wait) {
	return 0;
}

static int vpif_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////
/*IOCTL functions*/

static int vpif_g_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
	int ret = 0;
#if 0	
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
#if 0	
	/* we only support hue/saturation/contrast/brightness */
	if (ctrl->id < V4L2_CID_BRIGHTNESS || ctrl->id > V4L2_CID_HUE)
			return -EINVAL;
#endif
	if (ch->sd != NULL)
	{	
		mutex_lock(&ch->chan_lock);
		ret = v4l2_subdev_call(ch->sd, core, g_ctrl, ctrl);
		mutex_unlock(&ch->chan_lock);
	}
#endif
	return ret;
}



static int vpif_s_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{	
	int ret = 0;
	
	zynq_printk(0, "[%d](id, value) = (0x%08x, %d)\n", __LINE__, ctrl->id, ctrl->value);
	
#if 0
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
#if 0	
	/* we only support hue/saturation/contrast/brightness */
	if (ctrl->id < V4L2_CID_BRIGHTNESS || ctrl->id > V4L2_CID_HUE)
			return -EINVAL;
#endif
	if (ch->sd != NULL)
	{	
		mutex_lock(&ch->chan_lock);
		ret = v4l2_subdev_call(ch->sd, core, s_ctrl, ctrl);
		mutex_unlock(&ch->chan_lock);
	}
#endif
	return ret;
	
}

static char  *videofmt_to_string(eVideoFormat fmt) {
	switch (fmt) {
		case E1080P60FMT:
			return "1080p60";
		case E1080P50FMT:
			return "1080p50";
		case E720P60FMT:
			return "720p60";
		case E720P50FMT:
			return "720p50";
		default:
			return "UNKNOWNFMT";
	}
}



static int vpif_vout_pipline(struct file *file, void *fh, struct v4l2_vout_pipeline *a) {
	unsigned int i = 0;
	unsigned int vout_num = 2;
	unsigned int is_config = 0;
	vout_pipe_t *p = NULL;
	 
	vpif_vidoe_pipelie_entity_config_t config;
	vselector_source_t  src;
	vselector_vout_frame_size_t size;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	
	unsigned int is_valid_vin0 = vpif_control_is_valid(EVIN0) ;
	unsigned int is_valid_vin1 = vpif_control_is_valid(EVIN1) ;
	unsigned int is_valid_vin2 = vpif_control_is_valid(EVIN2) ;
	unsigned int is_valid_cpu = vpif_control_is_valid(ECPU) ;
	
	entity = board_find_video_pipeline_entity(VSELECTOR);
	
	if ((fpga_release_date < FPGA_MODULE_SUPPORT_DATE) || (en_modules == 0)) return -1;
	
	if (entity == NULL) return  -1;
	
	zynq_printk(0, "[zynq_control]%s -> %s,  (vin0, vin1, vin2, cpu) ->(%u, %u, %u, %u)\n", videofmt_to_string(g_vout_pipeline_config.format), videofmt_to_string(a->format), is_valid_vin0, is_valid_vin1, is_valid_vin2, is_valid_cpu);
	
	if (is_valid_vin0  == 0 && is_valid_vin1 == 0 ) return 0;
	
	/*if (g_vout_pipeline_config.format != a->format)*/
	{
		
		mutex_lock(&g_config_lock);
		g_vout_pipeline_config.format = a->format;
		mutex_unlock(&g_config_lock);
		
		if (a->format == E1080P60FMT || a->format == E1080P50FMT) {
			config.flag = VSELECTOR_OPTION_SET_VOUT_FRAME_SIZE ;
			size.width = 1920;
			size.height =1080;
			config.data = &size;
			entity->config(entity,  &config);
			default_in_width = 1920;
			default_in_height = 1080;
			if (a->format == E1080P60FMT)
				default_frame_rate = 60;
			else
				default_frame_rate = 50;
			
		} else if (a->format == E720P60FMT || a->format == E720P50FMT) {
			config.flag = VSELECTOR_OPTION_SET_VOUT_FRAME_SIZE ;
			size.width = 1280;
			size.height =720;
			config.data = &size;
			entity->config(entity,  &config);
			default_in_width = 1280;
			default_in_height = 720;
			if (a->format == E720P60FMT)
				default_frame_rate = 60;
			else
				default_frame_rate = 50;
		}
		
		vpif_control_enable_video_streams(0);
		stop_all_video_pipeline_entities(zynq_reg_base);
		release_all_video_pipeline_entities(zynq_reg_base);
		
		init_all_video_pipeline_entities(zynq_reg_base);
	 	config_all_video_pipeline_entities(zynq_reg_base);
	 	start_all_video_pipeline_entities(zynq_reg_base) ;
		//zynq_printk(0, "[zynq_control]video format: %s\n", videofmt_to_string(a->format));
		
	}
	
	
	for (i = 0; i < vout_num; i++) {
		p = &(a->pipes[i]);
		
		if (p->is_config == 0) continue;
		
		if ((a->pipes[i].full ==EVIN0 && (is_valid_vin0 == 0)) || (a->pipes[i].full ==EVIN1 && (is_valid_vin1 == 0)) || (a->pipes[i].full ==EVIN2 && (is_valid_vin2 == 0)) || (a->pipes[i].full ==ECPU && (is_valid_cpu ==0)))
			continue;
		
		if ((a->pipes[i].osd ==EVIN0 && (is_valid_vin0 == 0)) || (a->pipes[i].osd ==EVIN1 && (is_valid_vin1 == 0)) || (a->pipes[i].osd ==EVIN2 && (is_valid_vin2 == 0)) || (a->pipes[i].osd ==ECPU && (is_valid_cpu == 0))) 
			continue;
		
		is_config = 1;
		
		//zynq_printk(0, "[zynq_control][%d] (full, osd, config)---->(%s, %s, %d) \n",i,  pipeid_to_string(p->full), pipeid_to_string(p->osd), p->is_config);	
		if (i == 0) {
			config.flag = VSELECTOR_OPTION_SET_VOUT0_FULL_SRC;
			src.vin0	= (p->full == EVIN0 &&  is_valid_vin0)?1:0; 
			src.vin1	=	(p->full == EVIN1 &&  is_valid_vin1)?1:0; 
			src.vin2	=	(p->full == EVIN2 &&  is_valid_vin2)?1:0; 
			src.cpu	=	(p->full == ECPU &&  is_valid_cpu)?1:0; 
			config.data = &src;
			entity->config(entity,  &config);
			
			config.flag = VSELECTOR_OPTION_SET_VOUT0_1_16_SRC;
			src.vin0	= (p->osd == EVIN0 && is_valid_vin0)?1:0; 
			src.vin1	=	(p->osd == EVIN1 && is_valid_vin1)?1:0; 
			src.vin2	=	(p->osd == EVIN2 && is_valid_vin2)?1:0; 
			src.cpu	=	(p->osd == ECPU && is_valid_cpu)?1:0; 
			config.data = &src;
			entity->config(entity,  &config);
			
		} else if (i ==1) {
			config.flag = VSELECTOR_OPTION_SET_VOUT1_FULL_SRC;
			src.vin0	= (p->full == EVIN0 && is_valid_vin0)?1:0; 
			src.vin1	=	(p->full == EVIN1&& is_valid_vin1)?1:0; 
			src.vin2	=	(p->full == EVIN2 && is_valid_vin2)?1:0; 
			src.cpu	=	(p->full == ECPU && is_valid_cpu)?1:0; 
			config.data = &src;
			entity->config(entity,  &config);
			
			config.flag = VSELECTOR_OPTION_SET_VOUT1_1_16_SRC;
			src.vin0	= (p->osd == EVIN0 && is_valid_vin0)?1:0; 
			src.vin1	=	(p->osd == EVIN1 && is_valid_vin1)?1:0; 
			src.vin2	=	(p->osd == EVIN2 && is_valid_vin2)?1:0; 
			src.cpu	=	(p->osd == ECPU && is_valid_cpu)?1:0; 
			config.data = &src;
			entity->config(entity,  &config);
		}
		
		if (is_config) {
			mutex_lock(&g_config_lock);
			if ((a->pipes[i].full ==EVIN0 && is_valid_vin0) || (a->pipes[i].full ==EVIN1 && is_valid_vin1) || (a->pipes[i].full ==EVIN2 && is_valid_vin2) || (a->pipes[i].full ==ECPU && is_valid_cpu)) 
				g_vout_pipeline_config.pipes[i].full = a->pipes[i].full;
			else 
				g_vout_pipeline_config.pipes[i].full = ENONE;
			
			if ((a->pipes[i].osd ==EVIN0 && is_valid_vin0) || (a->pipes[i].osd ==EVIN1 && is_valid_vin1) || (a->pipes[i].osd ==EVIN2 && is_valid_vin2) || (a->pipes[i].osd ==ECPU && is_valid_cpu)) 
				g_vout_pipeline_config.pipes[i].osd = a->pipes[i].osd;
			else 
				g_vout_pipeline_config.pipes[i].osd = ENONE;
			mutex_unlock(&g_config_lock);
		}
	}
	
	if (is_config) {
		int  i = 0;
		
		vpif_control_enable_video_streams(0);
		
		for (i = 0; i < 2; i++) {
			if ((g_vout_pipeline_config.pipes[i].full  == EVIN0) || (g_vout_pipeline_config.pipes[i].osd == EVIN0)) {
				vpif_control_enable_video_stream_by_id(EVIN0, 1);
				break;
			}
		}
	
		for (i = 0; i < 2; i++) {
			if ((g_vout_pipeline_config.pipes[i].full  == EVIN1) || (g_vout_pipeline_config.pipes[i].osd == EVIN1)) {
				vpif_control_enable_video_stream_by_id(EVIN1,  1);
				break;
			}
		}
	
		for (i = 0; i < 2; i++) {
			if ((g_vout_pipeline_config.pipes[i].full  == EVIN2) || (g_vout_pipeline_config.pipes[i].osd == EVIN2)) {
				vpif_control_enable_video_stream_by_id(EVIN2, 1);
				break;
			}
		}
	}
	
	return 0;
}

static int vpif_scaler_crop(struct file *file, void *fh, struct v4l2_scaler_crop *a) {
	
	vpif_vidoe_pipelie_entity_id_t id = VUNKOWNPIPELINEID;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	
	unsigned int x_align = default_in_width/20;
	unsigned int y_align = default_in_height/20;
	
	unsigned int crop_start_x_align = (( (unsigned int) a->crop_start_x + x_align - 1) / x_align) * x_align;
	unsigned int crop_start_y_align = (( (unsigned int) a->crop_start_y + y_align - 1) / y_align) * y_align;
	unsigned int crop_width_align = (( (unsigned int) a->crop_width + x_align - 1) / x_align) * x_align;
	unsigned int crop_height_align =  (( (unsigned int) a->crop_height + y_align - 1) / y_align) * y_align;
	
	if ((fpga_release_date < FPGA_MODULE_SUPPORT_DATE) || (en_modules == 0)) return -1;
	
	if ((crop_start_x_align + crop_width_align  -1) > (default_in_width -1)) {
		zynq_printk(0, "[zynq_control]The scaler  region is illegal !!  (x, w, iw) ---> (%u, %u, %u) \n", crop_start_x_align, crop_width_align , default_in_width);
		return -1;
	}	
	if ((crop_start_y_align + crop_height_align -1) > (default_in_height -1)) { 
		zynq_printk(0, "[zynq_control]The scaler  region is illegal !!  (y, h, ih) ---> (%u, %u, %u) \n", crop_start_y_align, crop_height_align,  default_in_height);
		return -1;
	}
	
	zynq_printk(0, "[zynq_control]scaler crop (i, x, y, w, h) ---> (%u, %u, %u, %u, %u)\n", a->index, crop_start_x_align, crop_start_y_align, crop_width_align , crop_height_align);
	
	switch (a->index) {
		case 0:
			id = SCALER0; break;
		case 1:
			id = SCALER1; break;
		case 2:
			id = SCALER2; break;
		case 3:
			id = SCALER3;break;
		default:
			goto exit;
	}
	
	entity = board_find_video_pipeline_entity(id);
	if (entity) { 
		entity->config_crop(entity, crop_start_x_align, crop_start_y_align,  crop_width_align, crop_height_align);
	}
exit:	
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////

static int init_all_video_pipeline_entities(void __iomem *pci_reg_base) {
	
	unsigned int  i = 0;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	
	for (i = 0; i < board_video_pipeline_entity_num; i++){		 
		entity = &board_video_pipeline_entities[i];
		if (entity->init) {
			//zynq_printk(0, "[zynq_core][init_config](%d)id = %d\n", __LINE__,entity->id);
			entity->init(entity, pci_reg_base);
			//zynq_printk(0, "[zynq_core][init_config](%d)id = %d\n", __LINE__,entity->id);
		}
	}
	return 0;
}

static int release_all_video_pipeline_entities(void __iomem *pci_reg_base) {
	
	unsigned int  i = 0;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	
	for (i = 0; i < board_video_pipeline_entity_num; i++){		 
		entity = &board_video_pipeline_entities[i];
		if (entity->rls) 
			entity->rls(entity, pci_reg_base);
	}
	return 0;
}

static int start_all_video_pipeline_entities(void __iomem *pci_reg_base) {
	
	unsigned int  i = 0;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	
	for (i = 0; i < board_video_pipeline_entity_num; i++){		 
		entity = &board_video_pipeline_entities[i];
		if (entity->start) {
			//zynq_printk(0, "[zynq_core][start_config](%d)id = %d\n", __LINE__,entity->id);
			entity->start(entity);
			//zynq_printk(0, "[zynq_core][start_config](%d)id = %d\n", __LINE__,entity->id);
		}
	}
	return 0;
}


static int stop_all_video_pipeline_entities(void __iomem *pci_reg_base) {
	
	unsigned int  i = 0;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	
	for (i = 0; i < board_video_pipeline_entity_num; i++){		 
		entity = &board_video_pipeline_entities[i];
		if (entity->stop) 
			entity->stop(entity);
	}
	return 0;
}

#if 0
static int dump_all_video_pipeline_entities(void __iomem *pci_reg_base) {
	
	unsigned int  i = 0;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	
	for (i = 0; i < board_video_pipeline_entity_num; i++){		 
		entity = &board_video_pipeline_entities[i];
		if (entity->dump_regs) {
			zynq_printk(1,"%s:\n", to_video_pipelin_entity_name(entity->id));
			entity->dump_regs(entity);
		}
	}
	return 0;
	
}
#endif

static void config_scaler(vpif_vidoe_pipelie_entity_t *entity);
static void config_vselector(vpif_vidoe_pipelie_entity_t *entity) ;
static void config_osd(vpif_vidoe_pipelie_entity_t *entity);
static void config_resampler(vpif_vidoe_pipelie_entity_t *entity) ;
static void config_vtiming(vpif_vidoe_pipelie_entity_t *entity);
static void config_vdma(vpif_vidoe_pipelie_entity_t *entity);

static int config_all_video_pipeline_entities(void __iomem *pci_reg_base) {
	
	unsigned int  i = 0;
	vpif_vidoe_pipelie_entity_t *entity = NULL;
	for (i = 0; i < board_video_pipeline_entity_num; i++){		 
		entity = &board_video_pipeline_entities[i];
		if (entity->config) {
			switch (entity->type ) {
				case SCALER_TYPE: 
					config_scaler(entity);
					break;
				case VSELECTOR_TYPE: 
					config_vselector(entity);
					break;
				case VDMA_TYPE: 
					config_vdma(entity);
					break;
				case OSD_TYPE: 
					config_osd(entity);
					break;
				case CRESAMPLER_TYPE: 
					config_resampler(entity);
					break;
				case VTIMING_TYPE: 
					config_vtiming(entity);
					break;
				case VOUT_TYPE: break;
				case VIN_TYPE: break;
				case PCIEIF_TYPE: break;
				default: break;
			}
		}
	}
	return 0;
}


static void config_vdma(vpif_vidoe_pipelie_entity_t *entity) {
	vpif_vidoe_pipelie_entity_config_t config;	
	vdma_size_t size;
	//zynq_printk(0, "[zynq_core][vdma_config](%d)id = %d\n", __LINE__,entity->id);
	config.flag = VDMA_OPTION_SET_IN_SIZE;
	if (entity->id == VDMA0 || entity->id == VDMA2) {
		size.width = default_in_width;
		size.height = default_in_height;
	} else if (entity->id == VDMA1 || entity->id == VDMA3) {
		size.width = default_in_width/4;
		size.height = default_in_height/4;
	}
	config.data = &size;
	entity->config(entity,  &config);
	//zynq_printk(0, "[zynq_core][vdma_config](%d)id = %d\n", __LINE__,entity->id);
	return ;
}
static void config_vtiming(vpif_vidoe_pipelie_entity_t *entity) {
	
	vpif_vidoe_pipelie_entity_config_t config;	
	vtiming_size_t size;
	vtiming_fromat_t format;
	
	config.flag = VTIMING_OPTION_SET_FORMAT;
	format.value = VTIMING_FORMAT_YUV422;
	config.data = &format;
	entity->config(entity,  &config);
	
	config.flag = VTIMING_OPTION_SET_SIZE;
	if ((default_in_width == 1920) && (default_in_height == 1080)&&(default_frame_rate == 60))
		size.value = VTIMING_SIZE_1080P60;
	else if ((default_in_width == 1280) && (default_in_height == 720)&&(default_frame_rate == 60))
		size.value = VTIMING_SIZE_720P60;
	else if ((default_in_width == 1920) && (default_in_height == 1080)&&(default_frame_rate == 50))
		size.value = VTIMING_SIZE_1080P50;
	else if ((default_in_width == 1280) && (default_in_height == 720)&&(default_frame_rate == 50))
		size.value = VTIMING_SIZE_720P50;
	
	if (size.value == VTIMING_SIZE_1080P60)
		vpif_control_config_out_clock(E1080P60FMT);
	else if (size.value == VTIMING_SIZE_720P60) 
		vpif_control_config_out_clock(E720P60FMT);
	else if (size.value == VTIMING_SIZE_1080P50)
		vpif_control_config_out_clock(E1080P50FMT);
	else if (size.value == VTIMING_SIZE_720P50)
		vpif_control_config_out_clock(E720P50FMT);
	
	config.data=&size;
	entity->config(entity,  &config);
	
//	zynq_printk(0, "[zynq_control](%d) config_vtiming...\n", __LINE__);
	return;
}

static void config_resampler(vpif_vidoe_pipelie_entity_t *entity) {
	vpif_vidoe_pipelie_entity_config_t config;	
	resampler_size_t size;
	
	config.flag = RESAMPLER_OPTION_SET_ACTIVE_SIZE;
	size.width = default_in_width;
	size.height = default_in_height;
	config.data = &size;
	entity->config(entity,  &config);
	
	return ;
}

static void config_osd(vpif_vidoe_pipelie_entity_t *entity) {
	
	vpif_vidoe_pipelie_entity_config_t config;
	
	osd_enable_t enable;
	osd_size_t active_output_size;
	osd_layer_paramter_t param; 
	
	//zynq_printk(0, "[zynq_core][osd_config](%d)id = %d\n", __LINE__,entity->id);
	config.flag = OSD_OPTION_SET_OUTPUT_ACTIVE_SIZE;
	active_output_size.width = default_in_width;
	active_output_size.height = default_in_height;
	config.data = &active_output_size;
	entity->config(entity,  &config);
	
	config.flag = OSD_OPTION_SET_LAYER;
	param.id = OSD_LAYER_0;
	param.enable = 1;
	param.global_alpha_enable = 1;
	param.priority = 0;
	param.alpha_value = 0x100;
	param.position_x = 0;
	param.position_y = 0;
	param.width = default_in_width;
	param.height = default_in_height;
	config.data = &param;
	entity->config(entity,  &config);
	
	config.flag = OSD_OPTION_SET_LAYER;
	param.id = OSD_LAYER_1;
	param.enable = 1;
	param.global_alpha_enable = 1;
	param.priority = 1;
	param.alpha_value = 0x100;
	if ((default_in_width == 1920) && (default_in_height == 1080)) {
		param.position_x =  1440;
		param.position_y = 810;
	} else {
		param.position_x =  960;
		param.position_y = 540;
	}
	param.width = default_in_width >> 2;
	param.height = default_in_height >> 2;
	config.data = &param;
	entity->config(entity,  &config);
	
//	zynq_printk(0, "[zynq_control](%d) (x,y,w,h) ---> (%u, %u, %u, %u)\n", __LINE__, param.position_x, param.position_y, param.width, param.height);
	
	config.flag = OSD_OPTION_UPDATE_CHANGE;
	enable.value =1;
	config.data = &enable;
	entity->config(entity,  &config);
	
	//zynq_printk(0, "[zynq_core][osd_config](%d)id = %d\n", __LINE__,entity->id);
	return;
}

static void config_vselector(vpif_vidoe_pipelie_entity_t *entity) {
	
	vpif_vidoe_pipelie_entity_config_t config;
	int  i = 0;
	vselector_source_t  src;
	vselector_vout_frame_size_t size;
	
	if (b_firstconfig_vout_pipeline == 1) {
		b_firstconfig_vout_pipeline = 0;
		zynq_printk(0, "[zynq_control] Firstly initial the vselector!!\n");
		mutex_lock(&g_config_lock);
		g_vout_pipeline_config.pipes[0].full = ENONE;
		g_vout_pipeline_config.pipes[0].osd = ENONE;
		g_vout_pipeline_config.pipes[1].full = ENONE;
		g_vout_pipeline_config.pipes[1].osd = ENONE;
		mutex_unlock(&g_config_lock);
	}
	
		if ((default_in_width == 1920) && (default_in_height == 1080)) {
			config.flag = VSELECTOR_OPTION_SET_VOUT_FRAME_SIZE ;
			size.width = 1920;
			size.height =1080;
			config.data = &size;
			entity->config(entity,  &config);
			
		} else if ((default_in_width == 1280) && (default_in_height == 720)) {
			config.flag = VSELECTOR_OPTION_SET_VOUT_FRAME_SIZE ;
			size.width = 1280;
			size.height =720;
			config.data = &size;
			entity->config(entity,  &config);
		}
	
	
	config.flag = VSELECTOR_OPTION_SET_VOUT0_FULL_SRC;
	src.vin0	=  (g_vout_pipeline_config.pipes[0].full == EVIN0)?1:0 ; 
	src.vin1	=	 (g_vout_pipeline_config.pipes[0].full == EVIN1)?1:0 ; 
	src.vin2	=	 (g_vout_pipeline_config.pipes[0].full == EVIN2)?1:0 ; 
	src.cpu	=	 (g_vout_pipeline_config.pipes[0].full == ECPU)?1:0 ; 
	config.data = &src;
	entity->config(entity,  &config);
	
	config.flag = VSELECTOR_OPTION_SET_VOUT0_1_16_SRC;
	src.vin0	=  (g_vout_pipeline_config.pipes[0].osd == EVIN0)?1:0 ; 
	src.vin1	=	 (g_vout_pipeline_config.pipes[0].osd == EVIN1)?1:0 ; 
	src.vin2	=	 (g_vout_pipeline_config.pipes[0].osd == EVIN2)?1:0 ; 
	src.cpu	=	 (g_vout_pipeline_config.pipes[0].osd == ECPU)?1:0 ; 
	config.data = &src;
	entity->config(entity,  &config);
	
	config.flag = VSELECTOR_OPTION_SET_VOUT1_FULL_SRC;
	src.vin0	=  (g_vout_pipeline_config.pipes[1].full == EVIN0)?1:0 ; 
	src.vin1	=	 (g_vout_pipeline_config.pipes[1].full == EVIN1)?1:0 ; 
	src.vin2	=	 (g_vout_pipeline_config.pipes[1].full == EVIN2)?1:0 ; 
	src.cpu	=	 (g_vout_pipeline_config.pipes[1].full == ECPU)?1:0 ; 
	config.data = &src;
	entity->config(entity,  &config);
	

	config.flag = VSELECTOR_OPTION_SET_VOUT1_1_16_SRC;
	src.vin0	=  (g_vout_pipeline_config.pipes[1].osd == EVIN0)?1:0 ; 
	src.vin1	=	 (g_vout_pipeline_config.pipes[1].osd == EVIN1)?1:0 ; 
	src.vin2	=	 (g_vout_pipeline_config.pipes[1].osd == EVIN2)?1:0 ; 
	src.cpu	=	 (g_vout_pipeline_config.pipes[1].osd == ECPU)?1:0 ; 
	config.data = &src;
	entity->config(entity,  &config);


	
	vpif_control_enable_video_streams(0);
	
	for (i = 0; i < 2; i++) {
		if ((g_vout_pipeline_config.pipes[i].full  == EVIN0) || (g_vout_pipeline_config.pipes[i].osd == EVIN0)) {
			vpif_control_enable_video_stream_by_id(EVIN0, 1);
			break;
		}
	}
	
	for (i = 0; i < 2; i++) {
		if ((g_vout_pipeline_config.pipes[i].full  == EVIN1) || (g_vout_pipeline_config.pipes[i].osd == EVIN1)) {
			vpif_control_enable_video_stream_by_id(EVIN1, 1);
			break;
		}
	}
	
	for (i = 0; i < 2; i++) {
		if ((g_vout_pipeline_config.pipes[i].full  == EVIN2) || (g_vout_pipeline_config.pipes[i].osd == EVIN2)) {
			vpif_control_enable_video_stream_by_id(EVIN2, 1);
			break;
		}
	}
	
	return;
}

static void config_scaler(vpif_vidoe_pipelie_entity_t *entity) {
	
	vpif_vidoe_pipelie_entity_config_t config;
	scaler_size_t size;
	scaler_crop_area_t crop;
	scaler_num_phases_t phases;
	scaler_coef_data_t  coef_data;
	scaler_shrink_factor_t shrink_factor;
	unsigned int crop_start_x = 0;
	unsigned int crop_start_y = 0;
	unsigned int crop_width = default_in_width;
	unsigned int crop_height = default_in_height;
	unsigned int in_width = default_in_width;
	unsigned int in_height = default_in_height;
	unsigned int out_width = default_in_width >> 2;
	unsigned int out_height = default_in_height >> 2;
	
	//zynq_printk(0, "[zynq_core][scaler_config](%d)id = %d\n", __LINE__,entity->id);
	
	config.flag =  SCALER_OPTION_SET_IN_SIZE;
	size.width = in_width;
	size.height = in_height;
	config.data = &size;
	entity->config(entity,  &config);
	
	config.flag = SCALER_OPTION_SET_OUT_SIZE;
	size.width = out_width;
	size.height = out_height;
	config.data = &size;
	entity->config(entity,  &config);
	
//	if (entity->id == SCALER2){
		config.flag = SCALER_OPTION_SET_CROP;
		crop.start_x= crop_start_x;
		crop.start_y= crop_start_y;
		crop.width= crop_width;
		crop.height= crop_height;
		config.data = &crop;
		entity->config(entity,  &config);
		
		config.flag = SCALER_OPTION_SET_SHRINK_FACTOR;
		shrink_factor.hsf = ((crop_width << 20 )/out_width);
		shrink_factor.vsf = ((crop_height << 20)/out_height);
		config.data = &shrink_factor;
		entity->config(entity,  &config);	
//}
	
	config.flag = SCALER_OPTION_SET_NUM_PHASES;
	phases.num_h_phases = 4;
	phases.num_v_phases = 4;
	config.data = &phases;
	entity->config(entity,  &config);
	
	config.flag = SCALER_OPTION_SET_COEF_DATA;
	coef_data.coef_data = scaler_coef_data_0;
	config.data=&coef_data;
	entity->config(entity,  &config);
	//zynq_printk(0, "[zynq_core][scaler_config](%d)id = %d\n", __LINE__,entity->id);
}
////////////////////////////////////////////////////////////////////////////////////////
#define MAX_MODULE_NUM 5
static char g_scaler_regs_dump_str[MAX_MODULE_NUM][PAGE_SIZE] = {{0}};
static char g_vselector_regs_dump_str[MAX_MODULE_NUM][PAGE_SIZE]={{0}};
static char g_osd_regs_dump_str[MAX_MODULE_NUM][PAGE_SIZE] ={{0}};
static char g_resampler_regs_dump_str[MAX_MODULE_NUM][PAGE_SIZE]={{0}};
static char g_vtiming_regs_dump_str[MAX_MODULE_NUM][PAGE_SIZE]={{0}};
static char g_vdma_regs_dump_str[MAX_MODULE_NUM][PAGE_SIZE]={{0}};

static int set_register(vpif_vidoe_pipelie_entity_id_t  id, u16 offset, u32 value) {
	
	switch (id) {
		case SCALER0:
			scaler_set_reg(0, offset, value);
			break;
		case SCALER1:
			scaler_set_reg(1, offset, value);
			break;
		case SCALER2:
			scaler_set_reg(2, offset, value);
			break;
		case SCALER3:
			scaler_set_reg(3, offset, value);
			break;
		case VSELECTOR:
			vselector_set_reg(offset, value);
			break;
		case OSD0:
			osd_set_reg(0, offset, value);
			break;
		case OSD1:
			osd_set_reg(1, offset, value);
			break;
		case CRESAMPLER0:
			resampler_set_reg(0, offset, value);
			break;
		case CRESAMPLER1:
			resampler_set_reg(1, offset, value);
			break;
		case CRESAMPLER2:
			resampler_set_reg(2, offset, value);
			break;
		case CRESAMPLER3:
			resampler_set_reg(3, offset, value);
			break;
		case CRESAMPLER4:
			resampler_set_reg(4, offset, value);
			break;
		case VTIMING0:
			vtiming_set_reg(0 ,offset, value);
			break;
		case VTIMING1:
			vtiming_set_reg(1 ,offset, value);
			break;	
		case VDMA0:
			vdma_set_reg(0, offset, value);
			break;
		case VDMA1:
			vdma_set_reg(1, offset, value);
			break;
		case VDMA2:
			vdma_set_reg(2, offset, value);
			break;
		case VDMA3:
			vdma_set_reg(3, offset, value);
			break;
		default:
			return 0;
	}
	
	return 0;
}

static int dump_registers(vpif_vidoe_pipelie_entity_id_t  id , char *regs_dump_str, int len)
{
	unsigned int i = 0;
	unsigned int size = 0;
	vpif_video_cfg_regs_t  cfg_regs;
	vpif_video_cfg_reg_t *cfg_reg_ptr = NULL;
	
	unsigned int offset = 0;
	char *str = regs_dump_str;
	
	if ( str == NULL ) return -1;
	
	switch (id) {
		case SCALER0:
			scaler_get_regs(0, &cfg_regs);
			break;
		case SCALER1:
			scaler_get_regs(1, &cfg_regs);
			break;
		case SCALER2:
			scaler_get_regs(2, &cfg_regs);
			break;
		case SCALER3:
			scaler_get_regs(3, &cfg_regs);
			break;
		case VSELECTOR:
			vselector_get_regs(&cfg_regs);
			break;
		case OSD0:
			osd_get_regs(0, &cfg_regs);
			break;
		case OSD1:
			osd_get_regs(1, &cfg_regs);
			break;
		case CRESAMPLER0:
			resampler_get_regs(0, &cfg_regs);
			break;
		case CRESAMPLER1:
			resampler_get_regs(1, &cfg_regs);
			break;
		case CRESAMPLER2:
			resampler_get_regs(2, &cfg_regs);
			break;
		case CRESAMPLER3:
			resampler_get_regs(3, &cfg_regs);
			break;
		case CRESAMPLER4:
			resampler_get_regs(4, &cfg_regs);
			break;
		case VTIMING0:
			vtiming_get_regs(0, &cfg_regs);
			break;
		case VTIMING1:
			vtiming_get_regs(1, &cfg_regs);
			break;	
		case VDMA0:
			vdma_get_regs(0, &cfg_regs); 
			break;
		case VDMA1:
			vdma_get_regs(1, &cfg_regs); 
			break;
		case VDMA2:
			vdma_get_regs(2, &cfg_regs); 
			break;
		case VDMA3:
			vdma_get_regs(3, &cfg_regs); 
			break;
		default:
			return 0;
	}
	
	size = cfg_regs.num;
	cfg_reg_ptr = cfg_regs.regs;
	
	if (cfg_reg_ptr == NULL) return  -1;
	
	for (i = 0; i < size; i++) {
		offset = snprintf(str, len, "[0x%04x]:0x%08x\n", cfg_reg_ptr[i].offset, cfg_reg_ptr[i].value);
		str += offset;
	}
	return 0;
}

static ssize_t b_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	char *str = NULL;
	vpif_vidoe_pipelie_entity_id_t  id = VUNKOWNPIPELINEID;

	if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER0)) == 0) {	
		id = SCALER0;
		str = g_scaler_regs_dump_str[0];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER1)) == 0) {	
		id = SCALER1;
		str = g_scaler_regs_dump_str[1];
	} else if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER2)) == 0) {	
		id = SCALER2;
		str = g_scaler_regs_dump_str[2];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER3)) == 0) {	
		id = SCALER3;
		str = g_scaler_regs_dump_str[3];
	}  else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VSELECTOR)) == 0) {	
		id = VSELECTOR ;
		str = g_vselector_regs_dump_str[0];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(OSD0)) == 0) {	
		id = OSD0;
		str = g_osd_regs_dump_str[0];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(OSD1)) == 0) {	
		id = OSD1;
		str = g_osd_regs_dump_str[1];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER0)) == 0) {	
		id = CRESAMPLER0;
		str = g_resampler_regs_dump_str[0];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER1)) == 0) {	
		id = CRESAMPLER1;
		str = g_resampler_regs_dump_str[1];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER2)) == 0) {	
		id = CRESAMPLER2;
		str = g_resampler_regs_dump_str[2];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER3)) == 0) {	
		id = CRESAMPLER3;
		str = g_resampler_regs_dump_str[3];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER4)) == 0) {	
		id = CRESAMPLER4;
		str = g_resampler_regs_dump_str[4];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VTIMING0)) == 0) {	
		id = VTIMING0;
		str =g_vtiming_regs_dump_str[0];
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VTIMING1)) == 0) {	
		id = VTIMING1;
		str =g_vtiming_regs_dump_str[1];
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA0)) == 0) {	
		id = VDMA0 ;
		str =g_vdma_regs_dump_str[0];
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA1)) == 0) {	
		id = VDMA1;
		str =g_vdma_regs_dump_str[1];
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA2)) == 0) {	
		id = VDMA2;
		str =g_vdma_regs_dump_str[2];
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA3)) == 0) {	
		id = VDMA3;
		str =g_vdma_regs_dump_str[3];
	}
	
	if (str == NULL) {
		return  0;
	} else {
		dump_registers(id, str, PAGE_SIZE);
		return sprintf(buf, "%s", str);
	}
}

/*
 * For example, if wanting to set  the regisetr '0x0000'  of 'CRESAMPLER0' as '0x00000008':
 * echo "0x0000:0x00000008" > CRESAMPLER0
*/
static int parser_str(const char *buf, u16 *offset,  u32 *value) {
	
	int len = 0, nel = 0;
    const char *query = buf;
    char *q = NULL;
	char *v = NULL;
	char remove_new_line_str[16];
	unsigned int count = 0 ;
	q = (char *)query;
	
	memset(remove_new_line_str, 0x0, sizeof(remove_new_line_str));
	
	len = strlen(query);
    nel = 1;
    while (strsep(&q, ":")) nel++;
	
	if (nel != 3) return  -1;
	
	for (q = (char *)query; q < (query + len);) {
		
		char new_line = *(q+strlen(q) -1);
		
		if (count  == 2) 	return  -1;
		
		if (new_line == '\n') {
			strncpy(remove_new_line_str, q, strlen(q)-1);
			v = remove_new_line_str;
		} else {
			v = q;
		}
		
		if (count == 0) {
			
			if (strlen(v) != 6) return  -1;
			*offset = simple_strtoul(v, NULL, 16);
		} else {
			if (strlen(v) != 10)  return  -1;
			*value = simple_strtoul(v, NULL, 16);
		}
		count++;
		
       for (q += strlen(q); q < (query + len) && !*q; q++);
	}
	return 0;
} 
static ssize_t b_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	 
	 u16 offset = 0;
	 u32 value = 0;
	 vpif_vidoe_pipelie_entity_id_t  id = VUNKOWNPIPELINEID;
	// const char *id_str = attr->attr.name;
	 unsigned int got = 0;
	 
	 if ((!attr) || (!buf) || (count == 1) || (strlen(buf) == 1) ) goto exit;
	 
	 //zynq_printk(1, "[zynq_core] sysfs 2 (name, buf, count, strlen(buf)) = (%s, %s, %u, %u)\n",  id_str, buf, count,  strlen(buf));
	 
	 if (parser_str(buf, &offset, &value) != 0)  goto exit; 
	 
	if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER0)) == 0) {	
		id = SCALER0;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER1)) == 0) {	
		id = SCALER1;
		got = 1;
	} else if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER2)) == 0) {	
		id = SCALER2;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(SCALER3)) == 0) {	
		id = SCALER3;
		got = 1;
	}  else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VSELECTOR)) == 0) {	
		id = VSELECTOR ;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(OSD0)) == 0) {	
		id = OSD0;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(OSD1)) == 0) {	
		id = OSD1;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER0)) == 0) {	
		id = CRESAMPLER0;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER1)) == 0) {	
		id = CRESAMPLER1;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER2)) == 0) {	
		id = CRESAMPLER2;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER3)) == 0) {	
		id = CRESAMPLER3;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(CRESAMPLER4)) == 0) {	
		id = CRESAMPLER4;
		got = 1;
	} else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VTIMING0)) == 0) {	
		id = VTIMING0;
		got = 1;
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VTIMING1)) == 0) {	
		id = VTIMING1;
		got = 1;
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA0 )) == 0) {	
		id = VDMA0 ;
		got = 1;
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA1 )) == 0) {	
		id = VDMA1 ;
		got = 1;
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA2 )) == 0) {	
		id = VDMA2 ;
		got = 1;
	}else  if (strcmp(attr->attr.name, to_video_pipelin_entity_name(VDMA3 )) == 0) {	
		id = VDMA3 ;
		got = 1;
	}
	
	 if (got == 1) set_register(id, offset, value);
	 
exit:	 
	 return count;
}
static struct kobj_attribute scaler0_attribute = __ATTR(SCALER0,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute scaler1_attribute = __ATTR(SCALER1,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute scaler2_attribute = __ATTR(SCALER2,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute scaler3_attribute = __ATTR(SCALER3,  S_IRUGO |  S_IWUGO , b_show, b_store);
static struct kobj_attribute vselector_attribute = __ATTR(VSELECTOR,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute osd0_attribute = __ATTR(OSD0,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute osd1_attribute = __ATTR(OSD1,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute cresampler0_attribute = __ATTR(CRESAMPLER0,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute cresampler1_attribute = __ATTR(CRESAMPLER1,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute cresampler2_attribute = __ATTR(CRESAMPLER2,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute cresampler3_attribute = __ATTR(CRESAMPLER3,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute cresampler4_attribute = __ATTR(CRESAMPLER4,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute vtiming0_attribute = __ATTR(VTIMING0,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute vtiming1_attribute = __ATTR(VTIMING1,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute vdma0_attribute = __ATTR(VDMA0,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute vdma1_attribute = __ATTR(VDMA1,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute vdma2_attribute = __ATTR(VDMA2,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute vdma3_attribute = __ATTR(VDMA3,  S_IRUGO |  S_IWUGO, b_show, b_store);

static struct attribute *attrs[] = {
	(struct attribute *)&scaler0_attribute,
	(struct attribute *)&scaler1_attribute,
	(struct attribute *)&scaler2_attribute,
	(struct attribute *)&scaler3_attribute,
	(struct attribute *)&vselector_attribute,
	(struct attribute *)&osd0_attribute,
	(struct attribute *)&osd1_attribute,
	(struct attribute *)&cresampler0_attribute,
	(struct attribute *)&cresampler1_attribute,
	(struct attribute *)&cresampler2_attribute,
	(struct attribute *)&cresampler3_attribute,
	(struct attribute *)&cresampler4_attribute,
	(struct attribute *)&vtiming0_attribute ,
	(struct attribute *)&vtiming1_attribute ,
	(struct attribute *)&vdma0_attribute ,
	(struct attribute *)&vdma1_attribute ,
	(struct attribute *)&vdma2_attribute ,
	(struct attribute *)&vdma3_attribute ,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


static int create_register_dump_sysfs(struct kobject *kobj)
{
	int retval  = -1;
	unsigned int scaler_num  = 4 ;
	unsigned int vselector_num =1;
	unsigned int osd_num =2;
	unsigned int resampler_num = 5;
	unsigned int vtiming_num = 2;
	unsigned int vdma_num = 4;
	unsigned int  i = 0;
	
	for (i = 0; i < scaler_num; i++) 
		memset(g_scaler_regs_dump_str[i], 0x0, PAGE_SIZE);
	for (i = 0; i < vselector_num; i++) 
		memset(g_vselector_regs_dump_str[i], 0x0, PAGE_SIZE);
	for (i = 0; i < osd_num ; i++) 
		memset(g_osd_regs_dump_str[i], 0x0, PAGE_SIZE);
	for (i = 0; i <  resampler_num ; i++) 
		memset(g_resampler_regs_dump_str[i], 0x0, PAGE_SIZE);
	for (i = 0; i < vtiming_num; i++) 
		memset(g_vtiming_regs_dump_str[i], 0x0, PAGE_SIZE);
	for (i = 0; i < vdma_num; i++) 
		memset(g_vdma_regs_dump_str[i], 0x0, PAGE_SIZE);
	
	if (kobj != NULL)
		retval = sysfs_create_group(kobj, &attr_group);
	
	return retval;
}

static int destroy_register_dump_sysfs(struct kobject *kobj)
{
	if (!kobj) return -1;
	
	sysfs_remove_group(kobj, &attr_group);
	
	return 0;
}