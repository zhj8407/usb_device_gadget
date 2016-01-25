
#include <linux/stringify.h>
#include "zynq_debug.h"
#include "zynq_core.h"
#include "zynq_fpga_verify.h"


static int create_diagnostic_sysfs(struct kobject *kobj);
static int destroy_diagnostic_sysfs(struct kobject *kobj);

int zynq_diagnostic_probe( struct pci_dev *pdev) {
	
	create_diagnostic_sysfs(&pdev->dev.kobj);
	
	return  0;	
}

int zynq_diagnostic_remove(struct pci_dev *pdev) {
	
	destroy_diagnostic_sysfs(&pdev->dev.kobj);
	return  0;
}

int zynq_diagnostic_init(void) {
	return 0;
}

void  zynq_diagnostic_exit(void) {
	return;
}

//////////////////////////////////////////////////////////////////

static char g_fpga_logic_version_str[PAGE_SIZE];
static char g_fpga_compile_time_str[PAGE_SIZE];
static char g_fpga_temperature_str[PAGE_SIZE];
static char g_fpga_vccint_str[PAGE_SIZE];
static char g_fpga_vccaux_str[PAGE_SIZE];
static char g_fpga_vbram_str[PAGE_SIZE];
static char g_fpga_varmcore_str[PAGE_SIZE];
static char g_fpga_varmcoureaux_str[PAGE_SIZE];
static char g_fpga_varmcoremem_str[PAGE_SIZE];
static char g_fpga_dram_verify_enable_str[PAGE_SIZE];
static char g_fpga_dram_verify_status_str[PAGE_SIZE];

typedef enum {
	eTemperature,
	eVCCINT,
	eVCCAUX,
	eVBRAM,
	eVARMCore,
	eVARMCoreAux,
	eVARMCoreMem
} ESYSMonitorREGID;

void sys_monitor(ESYSMonitorREGID id, u32 *rval) {
		
	u16 offset = 0;
	u32 val = 0;
	
	if (!zynq_reg_base) return;
	
	switch (id) {
		case eTemperature: 
			offset = 0x0000; break;
		case eVCCINT:
			offset = 0x0004; break;
		case eVCCAUX:
			offset = 0x0008; break;
		case eVBRAM:
			offset = 0x0018; break;
		case eVARMCore:
			offset = 0x0034; break;
		case eVARMCoreAux:
			offset = 0x0038; break;
		case eVARMCoreMem:
			offset = 0x003c; break;
		default:
			return;
	}
	val = fpga_reg_read(zynq_reg_base, FPGA_SYS_MONITOR_REG + offset);
	
	*rval = (val & 0x0000fff0) >> 4;
	
}

static ssize_t b_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	char *str = NULL;
	u32 val = 0xffffffff;
	u8 bSetVal = 0;
	
	if (strcmp(attr->attr.name, __stringify(fpga_logic_version)) == 0) {
		str = &g_fpga_logic_version_str[0];
		memset(str, 0x0, PAGE_SIZE);
		if (zynq_reg_base) {
			val = fpga_reg_read(zynq_reg_base, FPGA_LOGIC_VERSION_REG);
			bSetVal = 1;
		}
	} else if (strcmp(attr->attr.name, __stringify(fpga_compile_time)) == 0) {
		str = &g_fpga_compile_time_str[0];
		memset(str, 0x0, PAGE_SIZE);
		if (zynq_reg_base) {
			val = fpga_reg_read(zynq_reg_base, FPGA_COMPILE_TIME_REG);
			bSetVal = 1;
		}
	} else if (strcmp(attr->attr.name, __stringify(fpga_temperature)) == 0) { 
		str = &g_fpga_temperature_str[0];
		memset(str, 0x0, PAGE_SIZE);
		sys_monitor(eTemperature, &val) ;
		bSetVal = 1;
	} else if (strcmp(attr->attr.name, __stringify(fpga_vccint)) == 0) { 
		str = &g_fpga_vccint_str[0];
		memset(str, 0x0, PAGE_SIZE);
		sys_monitor(eVCCINT, &val) ;
		bSetVal = 1;
	}else if (strcmp(attr->attr.name, __stringify(fpga_vccaux)) == 0) { 
		str = &g_fpga_vccaux_str[0];
		memset(str, 0x0, PAGE_SIZE);
		sys_monitor(eVCCAUX, &val) ;
		bSetVal = 1;
	}else if (strcmp(attr->attr.name, __stringify(fpga_vbram)) == 0) { 
		str = &g_fpga_vbram_str[0];
		memset(str, 0x0, PAGE_SIZE);
		sys_monitor(eVBRAM, &val) ;
		bSetVal = 1;
	}else if (strcmp(attr->attr.name, __stringify(fpga_varmcore)) == 0) { 
		str = &g_fpga_varmcore_str[0];
		memset(str, 0x0, PAGE_SIZE);
		sys_monitor(eVARMCore, &val) ;
		bSetVal = 1;
	}else if (strcmp(attr->attr.name, __stringify(fpga_varmcoreaux)) == 0) { 
		str = &g_fpga_varmcoureaux_str[0];
		memset(str, 0x0, PAGE_SIZE);
		sys_monitor(eVARMCoreAux, &val) ;
		bSetVal = 1;
	} else if (strcmp(attr->attr.name, __stringify(fpga_varmcoremem)) == 0) { 
		str = &g_fpga_varmcoremem_str[0];
		memset(str, 0x0, PAGE_SIZE);
		sys_monitor(eVARMCoreMem, &val) ;
		bSetVal = 1;
	} else if (strcmp(attr->attr.name, __stringify(fpga_dram_verify_enable)) == 0) { 
		str = &g_fpga_dram_verify_enable_str[0];
		memset(str, 0x0, PAGE_SIZE);
		bSetVal = 1;
	} else if (strcmp(attr->attr.name, __stringify(fpga_dram_verify_status)) == 0) { 
		str = &g_fpga_dram_verify_status_str[0];
		memset(str, 0x0, PAGE_SIZE);
		bSetVal = 1;
	}
	
	if (str != NULL) {
		if (bSetVal) {
			snprintf(str, PAGE_SIZE, "0x%08x\n", val );
			return sprintf(buf, "%s", str);
		} else  {
			return  0;
		}
	} else {
		return 0;
	}
}

//echo 1 > /sys/bus/pci/devices/0000\:01\:00.0/fpga_dram_verify_enable
static ssize_t b_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	
	 u32 val = 0;
	 
	 if ((!attr) || (!buf) || (count == 1) || (strlen(buf) == 1) ) goto exit;
	
	 val = simple_strtoul(buf, NULL, 16);
	 
	if (strcmp(attr->attr.name, __stringify(fpga_dram_verify_enable)) == 0) { 
		zynq_printk(0, "[zynq_diagnostic]0x%08x\n", val);
	}
	 
exit:	 
	 return count;
}



static struct kobj_attribute fpga_logic_version_attribute = __ATTR(fpga_logic_version,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_compile_time_attribute = __ATTR(fpga_compile_time,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_temperature_attribute = __ATTR(fpga_temperature,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_vccinit_attribute = __ATTR(fpga_vccint,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_vccaux_attribute = __ATTR(fpga_vccaux,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_vbram_attribute = __ATTR(fpga_vbram,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_varmcore_attribute = __ATTR(fpga_varmcore,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_varmcoreaux_attribute = __ATTR(fpga_varmcoreaux,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_varmcoremem_attribute = __ATTR(fpga_varmcoremem,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_dram_verify_enable_attribute = __ATTR(fpga_dram_verify_enable,  S_IRUGO |  S_IWUGO, b_show, b_store);
static struct kobj_attribute fpga_dram_verify_status_attribute = __ATTR(fpga_dram_verify_status,  S_IRUGO |  S_IWUGO, b_show, b_store);



static struct attribute *attrs[] = {
	(struct attribute *)&fpga_logic_version_attribute,
	(struct attribute *)&fpga_compile_time_attribute ,
	(struct attribute *)&fpga_temperature_attribute,
	(struct attribute *)&fpga_vccinit_attribute,
	(struct attribute *)&fpga_vccaux_attribute,
	(struct attribute *)&fpga_vbram_attribute,
	(struct attribute *)&fpga_varmcore_attribute,
	(struct attribute *)&fpga_varmcoreaux_attribute,
	(struct attribute *)&fpga_varmcoremem_attribute,
	(struct attribute *)&fpga_dram_verify_enable_attribute,
	(struct attribute *)&fpga_dram_verify_status_attribute,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


static int create_diagnostic_sysfs(struct kobject *kobj)
{
	int retval  = -1;
	
	memset( g_fpga_logic_version_str, 0x0, sizeof( g_fpga_logic_version_str));
	memset(g_fpga_compile_time_str, 0x0, sizeof(g_fpga_compile_time_str));
	
	memset(g_fpga_vccint_str, 0x0, sizeof(g_fpga_vccint_str));
	memset(g_fpga_vccaux_str, 0x0, sizeof(g_fpga_vccaux_str));
	memset(g_fpga_vbram_str, 0x0, sizeof(g_fpga_vbram_str));
	memset(g_fpga_varmcore_str, 0x0, sizeof(g_fpga_varmcore_str));
	memset(g_fpga_varmcoureaux_str, 0x0, sizeof(g_fpga_varmcoureaux_str));
	memset(g_fpga_varmcoremem_str, 0x0, sizeof(g_fpga_varmcoremem_str));
	memset(g_fpga_temperature_str, 0x0, sizeof(g_fpga_temperature_str));
	memset(g_fpga_dram_verify_enable_str, 0x0, sizeof(g_fpga_dram_verify_enable_str));
	memset(g_fpga_dram_verify_status_str, 0x0, sizeof(g_fpga_dram_verify_status_str));
	
	if (kobj != NULL)
		retval = sysfs_create_group(kobj, &attr_group);
	
	return retval;
}

static int destroy_diagnostic_sysfs(struct kobject *kobj)
{
	if (!kobj) return -1;
	
	sysfs_remove_group(kobj, &attr_group);
	
	return 0;
}