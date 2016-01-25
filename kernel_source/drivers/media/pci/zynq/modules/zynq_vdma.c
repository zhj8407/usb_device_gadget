#include <linux/mutex.h>
#include "zynq_vdma.h"
//#include "zynq_regs.h"
#include "../zynq_fpga_verify.h"
#include "../zynq_debug.h"
#include "../zynq_types.h"

typedef struct {
	u16 offset;
	u32 value;
} vdma_cfg_reg;

typedef struct{
	void __iomem *base;
	u16 mm2s_ctrl;
	u16 mm2s_status;
	u16 mm2s_reg_index;
	u16 mm2s_s2mm_park_pointer_reg;
	u16 version_reg;
	u16 s2mm_ctrl;
	u16 s2mm_status;
	u16 s2mm_irq_mask;
	u16 s2mm_reg_index;
	u16 mm2s_vsize;
	u16 mm2s_hsize;
	u16 mm2s_frame_delay_and_stride;
	u16 mm2s_start_address[3]; // 0x5c , 0x5c+0x4, 0x5c+0x8
	
	u16 s2mm_vsize;
	u16 s2mm_hsize;
	u16 s2mm_frame_delay_and_stride;
	u16 s2mm_start_address[3]; // 0xac, 0xac+0x4, 0xac+0x8
	
	u32  height;
	u32  width;
} vdma_handle_t;

#define VDMA_REG_NUM  (21)
extern vdma_cfg_reg vdma_cached_registers_0[];
extern vdma_cfg_reg vdma_cached_registers_1[];
extern vdma_cfg_reg vdma_cached_registers_2[];
extern vdma_cfg_reg vdma_cached_registers_3[];

//VDMA0 : 0x0c00 ~ 0x0dff
//VDMA1 : 0x0e00 ~ 0x0fff
//VDMA2 : 0x1000 ~ 0x11ff
//VDMA3 : 0x1400 ~ 0x15ff

static vdma_handle_t  handles[VDMA_MAX_NUM];
static struct mutex locks[VDMA_MAX_NUM];
static  vdma_cfg_reg  *cached_registers[VDMA_MAX_NUM];


static int set_in_size(vdma_size_t *userdata, unsigned int index);
static int sw_reset(unsigned int  index);
static int set_update_change(vdma_enable_t *userdata, unsigned int index);
static int set_enable(vdma_enable_t *userdata, unsigned int index);


static u8 is_initialized[VDMA_MAX_NUM] = {0};
static u8 is_started[VDMA_MAX_NUM] = {0};
static u8 is_reset[VDMA_MAX_NUM] = {0};

static u32 get_cached_registers(unsigned int index, u16 reg) {
	unsigned int count  = VDMA_REG_NUM;
	unsigned int  i = 0;
	for (i = 0; i < count; i++)
		if (cached_registers[index][i].offset == reg) 
			return cached_registers[index][i].value;
	
	return 0;
}

static int set_cached_registers(unsigned int index, u16 reg, u32 value) {
	unsigned int count  = VDMA_REG_NUM;
	unsigned int  i = 0;
	for (i = 0; i < count; i++)
		if (cached_registers[index][i].offset == reg) {
			cached_registers[index][i].value = value;
			return 0;
		}
	
	return -1;
}

void vdma_get_status(vdma_status_t *st, unsigned index){
	st->is_initialized = is_initialized[index];
	st->is_started = is_started[index];
}

int vdma_initial_by_index(void __iomem *pci_base_addr, unsigned index){
	unsigned i = index;
	
	if (i >= VDMA_MAX_NUM) return 0;
	
	if (is_initialized[i] == 1) return 0;
	
	if (i == 0)
		handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA0_REG);
	else if (i == 1)
		handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA1_REG);
	else if (i == 2)
		handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA2_REG);
	else if (i == 3)
		handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA3_REG);
		
	handles[i].mm2s_ctrl = 0x0000;
	handles[i].mm2s_status = 0x0004;
	handles[i]. mm2s_reg_index = 0x0014;
	handles[i]. mm2s_s2mm_park_pointer_reg = 0x0028;
	handles[i].version_reg = 0x002c;
	handles[i]. s2mm_ctrl = 0x0030;
	handles[i].s2mm_status = 0x0034;
	handles[i].s2mm_irq_mask = 0x003c;
	handles[i].s2mm_reg_index = 0x0044;
	handles[i]. mm2s_vsize = 0x0050;
	handles[i].mm2s_hsize = 0x0054;
	handles[i].mm2s_frame_delay_and_stride = 0x0058;
	handles[i].mm2s_start_address[0] = 0x005c; // 0x5c , 0x5c+0x4, 0x5c+0x8
	handles[i].mm2s_start_address[1] = 0x005c + 0x0004;
	handles[i].mm2s_start_address[2] = 0x005c + 0x0008;
	
	handles[i].s2mm_vsize = 0x00a0;
	handles[i].s2mm_hsize = 0x00a4;
	handles[i].s2mm_frame_delay_and_stride = 0x00a8;
	handles[i].s2mm_start_address[0] = 0x00ac; // 0xac, 0xac+0x4, 0xac+0x8
	handles[i].s2mm_start_address[1] = 0x00ac + 0x0004; // 0xac, 0xac+0x4, 0xac+0x8
	handles[i].s2mm_start_address[2] = 0x00ac + 0x0008; // 0xac, 0xac+0x4, 0xac+0x8
	
	handles[i].height = (u32) -1;
	handles[i].width =  (u32)-1;
		
	mutex_init(&locks[i]);
		
	if (i == 0)
			cached_registers[i] = &vdma_cached_registers_0[0];
	else  if (i == 1)
			cached_registers[i] = &vdma_cached_registers_1[0];
	else if (i == 2)
			cached_registers[i] = &vdma_cached_registers_2[0];
	else  if (i == 3)
			cached_registers[i] = &vdma_cached_registers_3[0];
		
	sw_reset(i);
	set_cached_registers(i, handles[i].mm2s_ctrl , fpga_reg_read(handles[i].base , handles[i].mm2s_ctrl )); 
	set_cached_registers(i, handles[i].mm2s_status, fpga_reg_read(handles[i].base , handles[i].mm2s_status )); 
	set_cached_registers(i, handles[i].mm2s_reg_index , fpga_reg_read(handles[i].base , handles[i].mm2s_reg_index)); 
	set_cached_registers(i, handles[i].mm2s_s2mm_park_pointer_reg , fpga_reg_read(handles[i].base , handles[i].mm2s_s2mm_park_pointer_reg)); 
	set_cached_registers(i, handles[i].version_reg, fpga_reg_read(handles[i].base , handles[i].version_reg)); 
	set_cached_registers(i, handles[i].s2mm_ctrl , fpga_reg_read(handles[i].base , handles[i].s2mm_ctrl )); 
	set_cached_registers(i, handles[i].s2mm_irq_mask, fpga_reg_read(handles[i].base , handles[i].s2mm_irq_mask )); 
	set_cached_registers(i, handles[i]. s2mm_reg_index , fpga_reg_read(handles[i].base , handles[i]. s2mm_reg_index)); 
	set_cached_registers(i, handles[i].mm2s_vsize , fpga_reg_read(handles[i].base , handles[i].mm2s_vsize )); 
	set_cached_registers(i, handles[i].mm2s_hsize , fpga_reg_read(handles[i].base , handles[i].mm2s_hsize )); 
	set_cached_registers(i, handles[i].mm2s_frame_delay_and_stride, fpga_reg_read(handles[i].base , handles[i].mm2s_frame_delay_and_stride )); 
	set_cached_registers(i, handles[i].mm2s_start_address[0] , fpga_reg_read(handles[i].base , handles[i].mm2s_start_address[0] )); 
	set_cached_registers(i, handles[i].mm2s_start_address[1] , fpga_reg_read(handles[i].base , handles[i].mm2s_start_address[1] )); 
	set_cached_registers(i, handles[i].mm2s_start_address[2] , fpga_reg_read(handles[i].base , handles[i].mm2s_start_address[2] ));
	set_cached_registers(i, handles[i].s2mm_vsize , fpga_reg_read(handles[i].base , handles[i].s2mm_vsize )); 
	set_cached_registers(i, handles[i].s2mm_hsize , fpga_reg_read(handles[i].base , handles[i].s2mm_hsize )); 
	set_cached_registers(i, handles[i].s2mm_frame_delay_and_stride, fpga_reg_read(handles[i].base , handles[i].s2mm_frame_delay_and_stride )); 
	set_cached_registers(i, handles[i].s2mm_start_address[0] , fpga_reg_read(handles[i].base , handles[i].s2mm_start_address[0] )); 
	set_cached_registers(i, handles[i].s2mm_start_address[1] , fpga_reg_read(handles[i].base , handles[i].s2mm_start_address[1] )); 
	set_cached_registers(i, handles[i].s2mm_start_address[2] , fpga_reg_read(handles[i].base , handles[i].s2mm_start_address[2] ));
	is_initialized[i] = 1;
	is_reset[i] = 1;
	return 0;
}

int vdma_initial(void __iomem *pci_base_addr){
	
	int i = 0;
	
	for (i = 0; i < VDMA_MAX_NUM; i++){
		
		if (is_initialized[i] == 1) continue;
		zynq_printk(1, "[zynq_vdma](%d)%d \n", __LINE__,i);
		if (i == 0)
			handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA0_REG);
		else if (i == 1)
			handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA1_REG);
		else if (i == 2)
			handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA2_REG);
		else if (i == 3)
			handles[i].base =  (void __iomem *)((u32)pci_base_addr + FPGA_VDMA3_REG);
		
		handles[i].mm2s_ctrl = 0x0000;
		handles[i].mm2s_status = 0x0004;
		handles[i]. mm2s_reg_index = 0x0014;
		handles[i]. mm2s_s2mm_park_pointer_reg = 0x0028;
		handles[i].version_reg = 0x002c;
		handles[i]. s2mm_ctrl = 0x0030;
		handles[i].s2mm_status = 0x0034;
		handles[i].s2mm_irq_mask = 0x003c;
		handles[i].s2mm_reg_index = 0x0044;
		handles[i]. mm2s_vsize = 0x0050;
		handles[i].mm2s_hsize = 0x0054;
		handles[i].mm2s_frame_delay_and_stride = 0x0058;
		handles[i].mm2s_start_address[0] = 0x005c; // 0x5c , 0x5c+0x4, 0x5c+0x8
		handles[i].mm2s_start_address[1] = 0x005c + 0x0004;
		handles[i].mm2s_start_address[2] = 0x005c + 0x0008;
	
		handles[i].s2mm_vsize = 0x00a0;
		handles[i].s2mm_hsize = 0x00a4;
		handles[i].s2mm_frame_delay_and_stride = 0x00a8;
		handles[i].s2mm_start_address[0] = 0x00ac; // 0xac, 0xac+0x4, 0xac+0x8
		handles[i].s2mm_start_address[1] = 0x00ac + 0x0004; // 0xac, 0xac+0x4, 0xac+0x8
		handles[i].s2mm_start_address[2] = 0x00ac + 0x0008; // 0xac, 0xac+0x4, 0xac+0x8
		
		handles[i].height = (u32) -1;
		handles[i].width =  (u32)-1;
		
		mutex_init(&locks[i]);
		
		if (i == 0)
			cached_registers[i] = &vdma_cached_registers_0[0];
		else  if (i == 1)
			cached_registers[i] = &vdma_cached_registers_1[0];
		else if (i == 2)
			cached_registers[i] = &vdma_cached_registers_2[0];
		else  if (i == 3)
			cached_registers[i] = &vdma_cached_registers_3[0];
		
		sw_reset(i);
		set_cached_registers(i, handles[i].mm2s_ctrl , fpga_reg_read(handles[i].base , handles[i].mm2s_ctrl )); 
		set_cached_registers(i, handles[i].mm2s_status, fpga_reg_read(handles[i].base , handles[i].mm2s_status )); 
		set_cached_registers(i, handles[i].mm2s_reg_index , fpga_reg_read(handles[i].base , handles[i].mm2s_reg_index)); 
		set_cached_registers(i, handles[i].mm2s_s2mm_park_pointer_reg , fpga_reg_read(handles[i].base , handles[i].mm2s_s2mm_park_pointer_reg)); 
		set_cached_registers(i, handles[i].version_reg, fpga_reg_read(handles[i].base , handles[i].version_reg)); 
		set_cached_registers(i, handles[i].s2mm_ctrl , fpga_reg_read(handles[i].base , handles[i].s2mm_ctrl )); 
		set_cached_registers(i, handles[i].s2mm_irq_mask, fpga_reg_read(handles[i].base , handles[i].s2mm_irq_mask )); 
		set_cached_registers(i, handles[i]. s2mm_reg_index , fpga_reg_read(handles[i].base , handles[i]. s2mm_reg_index)); 
		set_cached_registers(i, handles[i].mm2s_vsize , fpga_reg_read(handles[i].base , handles[i].mm2s_vsize )); 
		set_cached_registers(i, handles[i].mm2s_hsize , fpga_reg_read(handles[i].base , handles[i].mm2s_hsize )); 
		set_cached_registers(i, handles[i].mm2s_frame_delay_and_stride, fpga_reg_read(handles[i].base , handles[i].mm2s_frame_delay_and_stride )); 
		set_cached_registers(i, handles[i].mm2s_start_address[0] , fpga_reg_read(handles[i].base , handles[i].mm2s_start_address[0] )); 
		set_cached_registers(i, handles[i].mm2s_start_address[1] , fpga_reg_read(handles[i].base , handles[i].mm2s_start_address[1] )); 
		set_cached_registers(i, handles[i].mm2s_start_address[2] , fpga_reg_read(handles[i].base , handles[i].mm2s_start_address[2] ));
		set_cached_registers(i, handles[i].s2mm_vsize , fpga_reg_read(handles[i].base , handles[i].s2mm_vsize )); 
		set_cached_registers(i, handles[i].s2mm_hsize , fpga_reg_read(handles[i].base , handles[i].s2mm_hsize )); 
		set_cached_registers(i, handles[i].s2mm_frame_delay_and_stride, fpga_reg_read(handles[i].base , handles[i].s2mm_frame_delay_and_stride )); 
		set_cached_registers(i, handles[i].s2mm_start_address[0] , fpga_reg_read(handles[i].base , handles[i].s2mm_start_address[0] )); 
		set_cached_registers(i, handles[i].s2mm_start_address[1] , fpga_reg_read(handles[i].base , handles[i].s2mm_start_address[1] )); 
		set_cached_registers(i, handles[i].s2mm_start_address[2] , fpga_reg_read(handles[i].base , handles[i].s2mm_start_address[2] ));
		is_initialized[i] = 1;
		is_reset[i] = 1;
		zynq_printk(1, "[zynq_vdma](%d)%d \n", __LINE__,i);
	}

	return 0;
}

int vdma_setoption( EVDMAOptionFlags flag, void *userdata, unsigned  index){
	
	if (index >= VDMA_MAX_NUM) return  -1;
	
	if (is_initialized[index] == 0) return 0;
	
	switch (flag) {
		case VDMA_OPTION_RESET:
			sw_reset(index);
			 break;
		case VDMA_OPTION_UPDATE_CHANGE:
			set_update_change((vdma_enable_t *)userdata, index);
			break;
		case  VDMA_OPTION_ENABLE:
			set_enable((vdma_enable_t *)userdata, index);//TODO: 
			break;
		case VDMA_OPTION_SET_IN_SIZE:
			set_in_size((vdma_size_t *)userdata, index);
			break;
		default:
			break;
	}
	
	return 0;
}

int vdma_config_input_size(unsigned index, unsigned int in_width, unsigned int in_height) {
	
	vdma_size_t  size;
	vdma_enable_t enable;
	
	if (is_initialized[index] == 0) return 0;
	
	if ((in_width != handles[index].width) || (in_height != handles[index].height) ) {
		
		enable.value = 0;
		set_enable(&enable, index);
		size.width = in_width;
		size.height = in_height;
		set_in_size(&size, index);
		enable.value = 1;
		set_enable(&enable, index);
	}
	return 0;
}
int vdma_reset(unsigned index){
	
	if (index >= VDMA_MAX_NUM) return  -1;
	
	if (is_initialized[index] == 0) return 0;
	
	return sw_reset(index);
}

int vdma_start(unsigned index){
	vdma_enable_t enable;
	
	if (index >= VDMA_MAX_NUM) return  -1;
	
	if (is_initialized[index] == 0) return 0;
	
	if (is_started[index] == 1) return 0;
	
	enable.value = 1;
	set_enable(&enable, index);
	
	mutex_lock(&locks[index]);
	is_started[index] = 1;
	mutex_unlock(&locks[index]);
	
	return 0;
}

int vdma_stop(unsigned index){
	vdma_enable_t enable;
	
	if (index >= VDMA_MAX_NUM) return  -1;
	
	if (is_initialized[index] == 0) return 0;
	
	if (is_started[index] == 0) return 0;
	
	enable.value = 0;
	set_enable(&enable, index);
	
	mutex_lock(&locks[index]);
	is_started[index] = 0;
	mutex_unlock(&locks[index]);

	return 0;
}

int vdma_release(void __iomem *pci_base_addr){
	int i = 0;
	for (i = 0; i < VDMA_MAX_NUM; i++){
			
			if (is_initialized[i] == 0) continue;
			
			mutex_lock(&locks[i]);
			is_initialized[i] = 0;
			mutex_unlock(&locks[i]);
			
			mutex_destroy(&locks[i]);
	}
	return 0;
}

void vdma_dump_registers(unsigned index){
	unsigned int i = 0; 
	unsigned int count  = VDMA_REG_NUM;
	
	for (i  = 0; i < count; i ++ ) {
		zynq_printk(1, "[0x%02x] = 0x%08x \n", cached_registers[index][i].offset, cached_registers[index][i].value);		
	}
}

int vdma_enable_reg_update(unsigned index)
{
	vdma_enable_t enable;
	
	if (index >= VDMA_MAX_NUM) return  -1;
	
	if (is_initialized[index] == 0) return 0;
	
	enable.value = 0x1;
	set_update_change(&enable, index);
	
	return 0;
}//TODO: 

int  vdma_disable_reg_update(unsigned index) {
	vdma_enable_t enable;
	
	if (index >= VDMA_MAX_NUM) return  -1;
	
	if (is_initialized[index] == 0) return 0;
	
	enable.value = 0x0;
	set_update_change(&enable, index);
	
	return  0;
}

int vdma_get_regs(unsigned index, void *regs_ptr) {
	
	vpif_video_cfg_regs_t  *regs = (vpif_video_cfg_regs_t *)regs_ptr;
	
	if (!regs) return -1;
	
	regs->regs = (vpif_video_cfg_reg_t *)(&cached_registers[index][0]);
	regs->num = VDMA_REG_NUM;
	
	return 0;	
	
}

int vdma_set_reg(unsigned index, u16 offset, u32 value){
	
	if (index >= VDMA_MAX_NUM) return  -1;
	
	mutex_lock(&locks[index]);
	fpga_reg_write(handles[index].base, offset, value);
	set_cached_registers(index, offset, value);
	mutex_unlock(&locks[index]);
	
	return 0;
}

///////////////////////////////////////////////////////////

static void  do_specific_initialization(unsigned int index) {
	
	//VDMA0 : 0x0c00 ~ 0x0dff
	//VDMA1 : 0x0e00 ~ 0x0fff
	//VDMA2 : 0x1000 ~ 0x11ff
	//VDMA3 : 0x1400 ~ 0x15ff
	
	mutex_lock(&locks[index]);
	
	if (index == 0) {
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[0],  0x08000000);
		set_cached_registers(index, handles[index].mm2s_start_address[0], 0x08000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[1],  0x0a000000);
		set_cached_registers(index, handles[index].mm2s_start_address[1], 0x0a000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[2], 0x09000000);
		set_cached_registers(index, handles[index].mm2s_start_address[2], 0x09000000);
		
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[0],  0x08000000);
		set_cached_registers(index, handles[index].s2mm_start_address[0], 0x08000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[1],  0x0a000000);
		set_cached_registers(index, handles[index].s2mm_start_address[1], 0x0a000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[2], 0x09000000);
		set_cached_registers(index, handles[index].s2mm_start_address[2], 0x09000000);
	} else  if (index == 1) {
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[0],  0x0b000000);
		set_cached_registers(index, handles[index].mm2s_start_address[0], 0x0b000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[1],   0x0d000000);
		set_cached_registers(index, handles[index].mm2s_start_address[1],  0x0d000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[2], 0x0c000000);
		set_cached_registers(index, handles[index].mm2s_start_address[2], 0x0c000000);
		
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[0],  0x0b000000);
		set_cached_registers(index, handles[index].s2mm_start_address[0], 0x0b000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[1],   0x0d000000);
		set_cached_registers(index, handles[index].s2mm_start_address[1],  0x0d000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[2], 0x0c000000);
		set_cached_registers(index, handles[index].s2mm_start_address[2], 0x0c000000);
	} else  if (index == 2)  {
		/*
		 	dbg wl 0x3280105c 0x0E000000
			dbg wl 0x32801060 0x18000000
			dbg wl 0x32801064 0x0F000000
		 */
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[0],  0x0E000000);
		set_cached_registers(index, handles[index].mm2s_start_address[0], 0x0E000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[1],   0x18000000);
		set_cached_registers(index, handles[index].mm2s_start_address[1],  0x18000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[2], 0x0F000000);
		set_cached_registers(index, handles[index].mm2s_start_address[2], 0x0F000000);
		
		/*
		 	dbg wl 0x328010ac 0x0E000000
			dbg wl 0x328010b0 0x18000000
			dbg wl 0x328010b4 0x0F000000
		 */
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[0],  0x0E000000);
		set_cached_registers(index, handles[index].s2mm_start_address[0],0x0E000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[1],   0x18000000);
		set_cached_registers(index, handles[index].s2mm_start_address[1],  0x18000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[2], 0x0F000000);
		set_cached_registers(index, handles[index].s2mm_start_address[2], 0x0F000000);
	} else  if (index == 3) {
		/*
		 	dbg wl 0x3280145c 0x19000000
			dbg wl 0x32801460 0x1b000000
			dbg wl 0x32801464 0x1a000000
		 */
		
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[0],  0x19000000);
		set_cached_registers(index, handles[index].mm2s_start_address[0], 0x19000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[1],    0x1b000000);
		set_cached_registers(index, handles[index].mm2s_start_address[1],  0x1b000000);
	
		fpga_reg_write(handles[index].base, handles[index].mm2s_start_address[2], 0x1a000000);
		set_cached_registers(index, handles[index].mm2s_start_address[2], 0x1a000000);
		/*
		 	dbg wl 0x328014ac 0x19000000
			dbg wl 0x328014b0 0x1b000000
			dbg wl 0x328014b4 0x1a000000
		 */
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[0],  0x19000000);
		set_cached_registers(index, handles[index].s2mm_start_address[0],0x19000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[1],   0x1b000000);
		set_cached_registers(index, handles[index].s2mm_start_address[1],  0x1b000000);
	
		fpga_reg_write(handles[index].base, handles[index].s2mm_start_address[2], 0x1a000000);
		set_cached_registers(index, handles[index].s2mm_start_address[2], 0x1a000000);
	}

	fpga_reg_write(handles[index].base, handles[index].mm2s_frame_delay_and_stride, 0x01002000);
	set_cached_registers(index, handles[index].mm2s_frame_delay_and_stride, 0x01002000);
		
		
	fpga_reg_write(handles[index].base, handles[index].s2mm_frame_delay_and_stride, 0x01002000);
	set_cached_registers(index, handles[index].s2mm_frame_delay_and_stride, 0x01002000);
	
	mutex_unlock(&locks[index]);
	
}
static int sw_reset(unsigned int  index){

	//	mm2s_ctrl : 0x0000 , s2mm_ctrl : 0x0030
	u32 value = 0x00000004;
	
	mutex_lock(&locks[index]);
	fpga_reg_write(handles[index].base, handles[index].s2mm_ctrl, value);
	set_cached_registers(index, handles[index].s2mm_ctrl,value);
	mutex_unlock(&locks[index]);
	
	value = 0x00000008;
	mutex_lock(&locks[index]);
	fpga_reg_write(handles[index].base, handles[index].s2mm_ctrl, value);
	set_cached_registers(index, handles[index].s2mm_ctrl,value);
	mutex_unlock(&locks[index]);
	
	
	value = 0x00000004;
	
	mutex_lock(&locks[index]);
	fpga_reg_write(handles[index].base, handles[index].mm2s_ctrl, value);
	set_cached_registers(index, handles[index].mm2s_ctrl,value);
	mutex_unlock(&locks[index]);
	
	value = 0x00000008;
	mutex_lock(&locks[index]);
	fpga_reg_write(handles[index].base, handles[index].mm2s_ctrl, value);
	set_cached_registers(index, handles[index].mm2s_ctrl,value);
	mutex_unlock(&locks[index]);
	
	do_specific_initialization(index);
	
	return  0;
	
}

static int set_update_change(vdma_enable_t *enable, unsigned int index){

	//TODO:Controling for updating regitser should be implemented here?
	return 0;
}

static int set_enable(vdma_enable_t *enable, unsigned int index){
	u32 value= 0x0;
		
	//	mm2s_ctrl : 0x0000 , s2mm_ctrl : 0x0030
	
	if (enable->value)
		value =  0x00000003;
	else 
		value = get_cached_registers(index, handles[index].s2mm_ctrl) & ~(0x00000003);
	
	mutex_lock(&locks[index]);
	fpga_reg_write(handles[index].base, handles[index].s2mm_ctrl, value);
	set_cached_registers(index, handles[index].s2mm_ctrl,value);
	mutex_unlock(&locks[index]);
	
	if ((handles[index].height != (u32)-1) && (enable->value)) {
		u16 s2mm_vsize = handles[index].s2mm_vsize;
		fpga_reg_write(handles[index].base, s2mm_vsize, handles[index].height);
		set_cached_registers(index, s2mm_vsize, handles[index].height);
	} 
#if 0	
	if (enable->value)
		value = 0x000000083;
	else 
		value = get_cached_registers(index, handles[index].mm2s_ctrl) & ~(0x00000083);
#endif
	if (enable->value)
		value = 0x00000000b;
	else 
		value = get_cached_registers(index, handles[index].mm2s_ctrl) & ~(0x0000000b);
	
	mutex_lock(&locks[index]);
	fpga_reg_write(handles[index].base, handles[index].mm2s_ctrl, value);
	set_cached_registers(index, handles[index].mm2s_ctrl,value);
	mutex_unlock(&locks[index]);
	
	if ((handles[index].height != (u32)-1) && (enable->value)) {
		u16 mm2s_vsize = handles[index].mm2s_vsize;
		fpga_reg_write(handles[index].base, mm2s_vsize, handles[index].height);
		set_cached_registers(index, mm2s_vsize, handles[index].height);
	}
	
	return 0;
}

static int set_in_size(vdma_size_t *size, unsigned int index) {

//	u16 mm2s_vsize = handles[index].mm2s_vsize;
	u16 mm2s_hsize = handles[index].mm2s_hsize;
// u16 s2mm_vsize = handles[index].s2mm_vsize;
	u16 s2mm_hsize = handles[index].s2mm_hsize;
	
	u32  width = size->width;
	u32 height = size->height;

	mutex_lock(&locks[index]);
	
	if (height != handles[index].height) {
		handles[index].height = height;
#if 0		
		fpga_reg_write(handles[index].base, mm2s_vsize, height);
		set_cached_registers(index, mm2s_vsize, height);	
		fpga_reg_write(handles[index].base, s2mm_vsize, height);
		set_cached_registers(index, s2mm_vsize, height);
#endif	
	}
	
	if (width != handles[index].width) {
		handles[index].width = width;
		fpga_reg_write(handles[index].base, mm2s_hsize, (width * 2));
		set_cached_registers(index, mm2s_hsize, (width * 2));
		fpga_reg_write(handles[index].base, s2mm_hsize, (width * 2));
		set_cached_registers(index, s2mm_hsize, (width * 2));
	}
	mutex_unlock(&locks[index]);
	return 0;
}

/////////////////////////////////////////////////////////
vdma_cfg_reg vdma_cached_registers_0[] = {
{0x0000, 0x0},
{0x0004, 0x0},
{0x0014, 0x0},
{0x0028, 0x0},
{0x002c, 0x0},
{0x0030, 0x0},
{0x0034, 0x0},
{0x003c, 0x0},
{0x0044, 0x0},
{0x0050, 0x0},
{0x0054, 0x0},
{0x0058, 0x0},
{0x005c, 0x0},
{0x0060, 0x0},
{0x0064, 0x0},
{0x00a0, 0x0},
{0x00a4, 0x0},
{0x00a8, 0x0},
{0x00ac, 0x0},
{0x00b0, 0x0},
{0x00b4, 0x0}
};

vdma_cfg_reg vdma_cached_registers_1[] = {
{0x0000, 0x0},
{0x0004, 0x0},
{0x0014, 0x0},
{0x0028, 0x0},
{0x002c, 0x0},
{0x0030, 0x0},
{0x0034, 0x0},
{0x003c, 0x0},
{0x0044, 0x0},
{0x0050, 0x0},
{0x0054, 0x0},
{0x0058, 0x0},
{0x005c, 0x0},
{0x0060, 0x0},
{0x0064, 0x0},
{0x00a0, 0x0},
{0x00a4, 0x0},
{0x00a8, 0x0},
{0x00ac, 0x0},
{0x00b0, 0x0},
{0x00b4, 0x0}
};

vdma_cfg_reg vdma_cached_registers_2[] = {
{0x0000, 0x0},
{0x0004, 0x0},
{0x0014, 0x0},
{0x0028, 0x0},
{0x002c, 0x0},
{0x0030, 0x0},
{0x0034, 0x0},
{0x003c, 0x0},
{0x0044, 0x0},
{0x0050, 0x0},
{0x0054, 0x0},
{0x0058, 0x0},
{0x005c, 0x0},
{0x0060, 0x0},
{0x0064, 0x0},
{0x00a0, 0x0},
{0x00a4, 0x0},
{0x00a8, 0x0},
{0x00ac, 0x0},
{0x00b0, 0x0},
{0x00b4, 0x0}
};

vdma_cfg_reg vdma_cached_registers_3[] = {
{0x0000, 0x0},
{0x0004, 0x0},
{0x0014, 0x0},
{0x0028, 0x0},
{0x002c, 0x0},
{0x0030, 0x0},
{0x0034, 0x0},
{0x003c, 0x0},
{0x0044, 0x0},
{0x0050, 0x0},
{0x0054, 0x0},
{0x0058, 0x0},
{0x005c, 0x0},
{0x0060, 0x0},
{0x0064, 0x0},
{0x00a0, 0x0},
{0x00a4, 0x0},
{0x00a8, 0x0},
{0x00ac, 0x0},
{0x00b0, 0x0},
{0x00b4, 0x0}
};


////////////////////////////////////////////////////////