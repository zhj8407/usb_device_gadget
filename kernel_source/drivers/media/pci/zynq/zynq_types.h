#ifndef _ZYNQ_TYPES_H
#define _ZYNQ_TYPES_H

#include <linux/i2c.h>
#include <linux/videodev2.h>
#define VPIF_CAPTURE_MAX_CHANNELS	6
#define VPIF_DISPLAY_MAX_CHANNELS	1

enum vpif_if_type {
    VPIF_IF_BT656,
    VPIF_IF_BT1120,
    VPIF_IF_RAW_BAYER
};

struct vpif_interface {
    enum vpif_if_type if_type;
    unsigned hd_pol:1;
    unsigned vd_pol:1;
    unsigned fid_pol:1;
};

////////////////////////////////////////////////
/*FPGA related type*/

typedef enum  {
    VIN0_IN = 0, //means VIN_0
    VIN0_OUT = 1,
    VIN1_IN = 2, //means VIN_1
    VIN1_OUT =3,
    VIN2_IN = 4, //means VIN_2
    VIN2_OUT =5,
    VOUT0_IN= 6,
    VOUT0_OUT =7, //means VOUT_0
    VOUT1_IN =8,
    VOUT1_OUT =9, //means VOUT_1
    OSD0_IN =10,
    OSD0_OUT =11,
    OSD1_IN= 12,
    OSD1_OUT =13,
    SCALER0_IN = 14,
    SCALER0_OUT =15,
    SCALER1_IN =16,
    SCALER1_OUT =17,
    SCALER2_IN =18,
    SCALER2_OUT =19,
    SCALER3_IN =20,
    SCALER3_OUT = 21,
    VSELECTOR_IN = 22,
    VSELECTOR_OUT = 23,
    VDMA0_IN  = 24,
    VDMA0_OUT  = 25,
    VDMA1_IN  = 26,
    VDMA1_OUT  = 27,
    VDMA2_IN  = 28,
    VDMA2_OUT  = 29,
    VDMA3_IN  = 30,
    VDMA3_OUT  = 31,
    CRESAMPLER0_IN = 32,
    CRESAMPLER0_OUT = 33,
    CRESAMPLER1_IN = 34,
    CRESAMPLER1_OUT = 35,
    CRESAMPLER2_IN = 36,
    CRESAMPLER2_OUT = 37,
    CRESAMPLER3_IN = 38,
    CRESAMPLER3_OUT = 39,
    CRESAMPLER4_IN = 40,
    CRESAMPLER4_OUT = 41,
    PCIEIF_IN = 42,
    PCIEIF_OUT = 43,
    VPINNONE = 44, //means none pin
	SCALER4_IN =45,
    SCALER4_OUT = 46,
    VPINNUM = 47
} vpif_video_data_pin_t;

#define VIN_0 (VIN0_IN)
#define VIN_1 (VIN1_IN)
#define VIN_2 (VIN2_IN)
#define VOUT_0 (VOUT0_OUT)
#define VOUT_1 (VOUT1_OUT)

typedef enum {
    SCALER0 = 0,
    SCALER1 = 1,
    SCALER2 = 2,
    SCALER3 = 3,
    VSELECTOR = 4,
    VDMA0 = 5,
    VDMA1 = 6,
    VDMA2 = 7,
    VDMA3 = 8,
    OSD0 = 9,
    OSD1 = 10,
    CRESAMPLER0 = 11,
    CRESAMPLER1 = 12,
    CRESAMPLER2 = 13,
    CRESAMPLER3 = 14,
    CRESAMPLER4 = 15,
    VTIMING0 = 16,
    VTIMING1 = 17,
    VIN0 = 18,
    VIN1= 19,
    VIN2 = 20,
    VOUT0 = 21,
    VOUT1 = 22,
    PCIEIF =23,
    VUNKOWNPIPELINEID = 24,
	SCALER4 = 25,
    VPIPELINEIDNUM = 26
} vpif_vidoe_pipelie_entity_id_t ;

typedef enum {
    SCALER_TYPE = 0,
    VSELECTOR_TYPE =1,
    VDMA_TYPE = 2,
    OSD_TYPE = 3,
    CRESAMPLER_TYPE = 4,
    VTIMING_TYPE = 5,
    VOUT_TYPE =6,
    VIN_TYPE =7,
    PCIEIF_TYPE =8,
    VUNKOWNPIPELINETYPE =9,
    VPIPELINETYPENUM = 10
} vpif_vidoe_pipelie_entity_type_t;

typedef struct {
    unsigned int flag;
    void *data;
} vpif_vidoe_pipelie_entity_config_t;

struct vpif_vidoe_pipelie_entity;

typedef void (*ENTITY_CONFIG_FUNC) (struct vpif_vidoe_pipelie_entity* handle,  vpif_vidoe_pipelie_entity_config_t* config);
typedef void (*ENTITY_START_FUNC) (struct vpif_vidoe_pipelie_entity* handle);
typedef void (*ENTITY_STOP_FUNC) (struct vpif_vidoe_pipelie_entity* handle);
typedef void (*ENTITY_INIT_FUNC) (struct vpif_vidoe_pipelie_entity* handle, 	void __iomem *pci_base_addr);
typedef void (*ENTITY_RLS_FUNC) (struct vpif_vidoe_pipelie_entity* handle, 	void __iomem *pci_base_addr);
typedef void (*ENTITY_DUMPREGS_FUNC) (struct vpif_vidoe_pipelie_entity* handle);
typedef void (*ENTITY_CONFIG_INPUTSIZE_FUNC) (struct vpif_vidoe_pipelie_entity* handle, unsigned int in_width, unsigned int in_height);
typedef void (*ENTITY_CONFIG_CROP_FUNC) (struct vpif_vidoe_pipelie_entity* handle, unsigned int crop_start_x, unsigned int crop_start_y, unsigned int crop_width,  unsigned int crop_height);

typedef  struct vpif_vidoe_pipelie_entity {
    vpif_vidoe_pipelie_entity_id_t id;
    vpif_vidoe_pipelie_entity_type_t type;
    ENTITY_CONFIG_FUNC config;
    ENTITY_START_FUNC start;
    ENTITY_STOP_FUNC stop;
    ENTITY_INIT_FUNC init;
    ENTITY_RLS_FUNC rls;
    ENTITY_DUMPREGS_FUNC dump_regs;
    ENTITY_CONFIG_INPUTSIZE_FUNC config_input_size;
    ENTITY_CONFIG_CROP_FUNC config_crop;
    struct list_head list;
} vpif_vidoe_pipelie_entity_t;

typedef struct  {
    vpif_video_data_pin_t start;
    vpif_video_data_pin_t end;
    struct list_head path_head;
} vpif_video_pipeline_config_t;

typedef struct {
    u16 offset;
    u32 value;
} vpif_video_cfg_reg_t;

typedef struct {
    vpif_video_cfg_reg_t *regs;
    unsigned int num;
} vpif_video_cfg_regs_t;

///////////////////////////////////////////////
struct vpif_subdev_info {
    unsigned int enable;
    const char *name;
    struct i2c_board_info board_info;
    vpif_video_data_pin_t data_pin;
};

struct vpif_output {
    struct v4l2_output output;
    const char *subdev_name;
    u32 input_route;
    u32 output_route;
};

struct vpif_display_chan_config {
    const struct vpif_output *outputs;
    int output_count;
    bool clip_en;
};

struct vpif_display_config {
    int (*set_clock)(int, int);
    struct vpif_subdev_info *subdevinfo;
    int subdev_count;
    struct vpif_display_chan_config chan_config[VPIF_DISPLAY_MAX_CHANNELS];
    const char *card_name;
};

struct vpif_input {
    struct v4l2_input input;
    const char *subdev_name;
    u32 input_route;
    u32 output_route;
};

struct vpif_capture_chan_config {
    struct vpif_interface vpif_if;
    const struct vpif_input *inputs;
    int input_count;
};

struct vpif_capture_config {
    int (*setup_input_channel_mode)(int);
    int (*setup_input_path)(int, const char *);
    struct vpif_capture_chan_config chan_config[VPIF_CAPTURE_MAX_CHANNELS];
    struct vpif_subdev_info *subdev_info;
    int subdev_count;
    const char *card_name;
};

struct vpif_video_pipeline_config {
    int channel_id;
    int scaler_id;
    int vdma_id;
    int osd_id;
    int resampler_id;
};

#endif /* _VPIF_TYPES_H */
