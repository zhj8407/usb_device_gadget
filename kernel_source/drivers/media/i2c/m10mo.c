/*
 * m10mochk.c
 * OmniVision 2722 Basic Check Driver
 *
 * Copyright (C) 2013 IAC
 * Author: Wei Xiao Hui <Wei.Xiao-Hui@iac.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <media/m10mo.h>
#include <linux/m10mo-ioctl.h>

#define M10MO_FW_NAME		"/data/m10mo.bin"

static struct file *gM10MO_FileID;
static mm_segment_t fs_old;
unsigned int uRegValue = 0;

#define AUXCLK2	0
#if AUXCLK2
static struct clk *auxclk2_ck = NULL;
#endif

#define M10MO_FLASH_MEMORY_ADDRESS     0x00000000		 
#define M10MO_DESTINATION_SIO_ADDRESS  0x20000000
#define M10MO_SDRAM_SIZE               0x200000
#define M10MO_PLL_VALUE                0x00170141
#define M10MO_SPI_RISING               0x4C
#define M10MO_SPI_FALLING              0x44
#define M10MO_SPI_MODE                 0x02
#define M10MO_FW_BUF_SIZE              0x10000

#define ISP_RESET_L  M10MO_RST_GPIO

struct m10mo_data {
	struct i2c_client *client;
	/* lock for sysfs operations */
	struct mutex lock;
};
static struct spi_device *m10mo_spi = NULL;
static struct m10mo_data *m10mo_des = NULL;

static dev_t m10mo_dev_t = 0;
static struct cdev *m10mo_cdev = NULL;
static struct class *m10mo_cls = NULL;
static struct m10mo_ver m10mo_ver;

#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
//#define test_i2c_do

#if 0
static u32 io_read(unsigned long pad_addr)
{
	int val;
	u32 *addr;

	addr = (u32 *) ioremap(pad_addr, 4);
	if (!addr) {
		printk(KERN_ERR" ioremap failed with addr %lx\n",
			pad_addr);
		return 0;
	}

	val = __raw_readl(addr);

	iounmap(addr);

	return val;
}
#endif

static int m10mo_read_reg(struct i2c_client *i2c, unsigned char * send_buf, int send_num, unsigned char * rcv_buf,  int rcv_num)

{
    int err;
    int tries = 0;
#ifdef   test_i2c_do
    int i = 0;
#endif    
    struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);	
    struct i2c_msg msgs[] = {
        {
            .addr = i2c->addr,
            .flags = 0,
            .len = send_num,
            .buf = send_buf,
        },
        {
            .addr = i2c->addr,
            .flags = I2C_M_RD,
            .len = rcv_num,
            .buf = rcv_buf,
        },
    };	

#ifdef   test_i2c_do
    printk("i2c read slave address %x\n", msgs[0].addr);

    for(i=0; i<send_num; i++)
        printk(" %x ", send_buf[i]);

    printk("\n");
#endif
    do {
        err = i2c_transfer(adapter, msgs, 2);
        if (err != 2)
            msleep_interruptible(I2C_RETRY_DELAY);
    } while ((err != 2) && (++tries < I2C_RETRIES));

    if (err != 2) {
        printk("i2c read transfer error, err = %d\n", err);
        err = -EIO;
    } else {		
        err = 0;
    }

#ifdef   test_i2c_do
    int i = 0;
    printk("i2c read result: ");
    for(i=0; i<rcv_num; i++)
        printk(" %x ", rcv_buf[i]);
    printk("\n");
#endif
    return err;
}


static int m10mo_write_reg(struct i2c_client *i2c, unsigned char * send_buf, int send_num)
{
	int err;
	int tries = 0;

	struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);	
	struct i2c_msg msgs[] = {
		{
		 .addr = i2c->addr,
		 .flags = 0,
		 .len = send_num,
		 .buf = send_buf,
		 },
	};

#ifdef   test_i2c_do
    printk("i2c write slave address %x ", msgs[0].addr);

    for(i=0; i<send_num; i++)
        printk(" %x ", send_buf[i]);

    printk("\n");
#endif
	do {
		err = i2c_transfer(adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		printk("i2c write transfer error, err = %d\n", err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


#ifndef M10MO_USE_MALLOC
unsigned char gM10MO_FwBuf[M10MO_FW_BUF_SIZE];
#endif


void* M10MO_Custom_Malloc(unsigned long size)
{
	return kmalloc(size, GFP_KERNEL);
}

void M10MO_Custom_Free(void * addr)
{
	if(addr != NULL){
		kfree(addr);
	}
}

int M10MO_Custom_StartReadFirmware(char *fw_name)
{
	// TODO init file pointer here

	gM10MO_FileID = filp_open(fw_name, O_RDWR | O_CREAT, 0644);
      if (IS_ERR(gM10MO_FileID)) {
        printk("create file error\n");
        return -1;
    }

	 fs_old = get_fs();
     set_fs(KERNEL_DS);
	
	return 0;
}

static void M10MO_Custom_Delay(unsigned int mtime)
{
//	mdelay(mtime);
    msleep(mtime);
}

int M10MO_Custom_ReadFirmware(unsigned char * addr, unsigned long size, unsigned long pos)
{
	int err;
    loff_t pos_t = (loff_t) pos;
	err = vfs_read(gM10MO_FileID, addr, size,  &pos_t);
//	printk("vfs_read size %d, pos_t %lld pos %lld err %x\n", size, pos_t, pos, err);
	if( err < 0 ){
		return	-1;
	}
	
	return err;
}

int M10MO_Custom_EndReadFirmware(void)
{
	if(gM10MO_FileID != NULL)
        filp_close(gM10MO_FileID, NULL);
    gM10MO_FileID = NULL;

	
    set_fs(fs_old);
    return 0;
}

static int M10MO_WriteOneByteCRAM(struct i2c_client *i2c, unsigned char cat,unsigned char byte,unsigned char data)
{
    int ret;
    unsigned char cmd_senddata[8];
    cmd_senddata[0] = 5;
    cmd_senddata[1] = 2;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    cmd_senddata[4] = data;
    ret = m10mo_write_reg(i2c,  cmd_senddata, 5);
    return ret;
}

static unsigned char M10MO_ReadOneByteCRAM(struct i2c_client *i2c, unsigned char cat,unsigned char byte)
{
    unsigned char cmd_senddata[8];
    unsigned char cmd_recdata[2] = {0};
    cmd_senddata[0] = 5;
    cmd_senddata[1] = 1;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    cmd_senddata[4] = 1;
    //Read 2 byte: 1 byte for length and 1 byte data
    m10mo_read_reg(i2c,  cmd_senddata, 5, cmd_recdata, 2); 
    return cmd_recdata[1];
}

static int M10MO_WriteOneHalfwordCRAM(struct i2c_client *i2c,  unsigned char cat, unsigned char byte, unsigned short data )
{
    int ret;
    unsigned char cmd_senddata[8];
    cmd_senddata[0] = 6;
    cmd_senddata[1] = 2;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    cmd_senddata[4] = (unsigned char)( data >>8 );
    cmd_senddata[5] = (unsigned char)( data &0x00FF );
    ret = m10mo_write_reg(i2c,  cmd_senddata, 6);
    return ret;
}

static unsigned short M10MO_ReadOneHalfwordCRAM(struct i2c_client *i2c,  unsigned char cat, unsigned char byte )
{
    unsigned char	cmd_senddata[8];
    unsigned char	cmd_recdata[3] = {0};
    unsigned short	ret;
    cmd_senddata[0] = 5;
    cmd_senddata[1] = 1;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    cmd_senddata[4] = 2;
    //Read 3 byte: 1 byte for length and 2 byte data
    m10mo_read_reg(i2c,  cmd_senddata, 5, cmd_recdata, 3); 
    ret  = cmd_recdata[1]<<8;
    ret += cmd_recdata[2];
    return	ret;
}

static int M10MO_WriteOneWordCRAM(struct i2c_client *i2c,  unsigned char cat, unsigned char byte, unsigned long data )
{
    int ret;
    unsigned char cmd_senddata[8];
    cmd_senddata[0] = 8;
    cmd_senddata[1] = 2;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    cmd_senddata[4] = (unsigned char)(( data &0xFF000000 )>>24 );
    cmd_senddata[5] = (unsigned char)(( data &0x00FF0000 )>>16 );
    cmd_senddata[6] = (unsigned char)(( data &0x0000FF00 )>>8 );
    cmd_senddata[7] = (unsigned char)(  data &0x000000FF );
    ret = m10mo_write_reg(i2c,  cmd_senddata, 8);
    return ret;
}

static unsigned long M10MO_ReadOneWordCRAM(struct i2c_client *i2c,  unsigned char cat, unsigned char byte )
{
    unsigned char	cmd_senddata[8];
    unsigned char	cmd_recdata[5] = {0};
    unsigned long	ret;
    cmd_senddata[0] = 5;
    cmd_senddata[1] = 1;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    cmd_senddata[4] = 4;
    //Read 5 byte: 1 byte for length and 4 byte data
    m10mo_read_reg(i2c,  cmd_senddata, 5, cmd_recdata, 5); 
    ret  = cmd_recdata[0]<<24;
    ret += cmd_recdata[1]<<16;
    ret += cmd_recdata[2]<<8;
    ret += cmd_recdata[3];
    return	ret;
}

#if 0
static int M10MO_ReadBytesCRAM(struct i2c_client *i2c,  unsigned char cat, unsigned char byte , unsigned char byteCnt, unsigned char* cmd_recdata)
{
    int ret;
    unsigned char	cmd_senddata[8];
    cmd_senddata[0] = 5;
    cmd_senddata[1] = 1;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    cmd_senddata[4] = byteCnt;
    //Read byteCnt+1 byte: 1 byte for length and byteCnt byte data
    ret = m10mo_read_reg(i2c,  cmd_senddata, 5, cmd_recdata, byteCnt+1); 
    return ret;
}

static int M10MO_WriteBytesCRAM(struct i2c_client *i2c, unsigned char cat, unsigned char byte, unsigned char byteCnt, unsigned char* cmd_snddata)
{
    int ret, i;
    unsigned char	cmd_senddata[28];
    cmd_senddata[0] = byteCnt + 4;
    cmd_senddata[1] = 2;
    cmd_senddata[2] = cat;
    cmd_senddata[3] = byte;
    for(i = 0; i < byteCnt; i++)
        cmd_senddata[i+4] = *cmd_snddata++;
    //Read byteCnt+1 byte: 1 byte for length and byteCnt byte data
    ret = m10mo_write_reg(i2c,  cmd_senddata, byteCnt+4); 
    return ret;
}
#endif

static int M10MO_WriteMemory(struct i2c_client *i2c, unsigned char * send_buf, unsigned long buf_size, unsigned long addr, unsigned long write_size)
{
    if(buf_size >= 8 + write_size)
    {
        int ret;
        send_buf[0] = 0x0;
        send_buf[1] = 4;	//CMD_EX_WRITE
        send_buf[2] = (unsigned char)((addr & 0xFF000000) >> 24);
        send_buf[3] = (unsigned char)((addr & 0x00FF0000) >> 16);
        send_buf[4] = (unsigned char)((addr & 0x0000FF00) >>  8);
        send_buf[5] = (unsigned char)( addr & 0x000000FF);
        send_buf[6] = (unsigned char)((write_size & 0xFF00) >> 8);
        send_buf[7] = (unsigned char)(write_size & 0x00FF);		
        ret = m10mo_write_reg(i2c,  send_buf, 8 + write_size);
        return ret;

    }else
       return 1;
}

static int M10MO_GetCheckSum(struct i2c_client *i2c, unsigned short * sum )
{
    int loop = 0;
    int status,chk_sum=0x5;
    //begin to check the firmare sum
    M10MO_WriteOneByteCRAM(i2c,0x0F,0x09,0x04);
    printk(KERN_ALERT"Pooling to do checksum....\n");
    for(loop = 0;loop < 200; loop++)
    {
        //polling to check the status
        status = M10MO_ReadOneByteCRAM(i2c,0x0F,0x09);
        printk("Data return 0x%x\n",status);
        if(status == 0x00)
            break;
        M10MO_Custom_Delay(100);
    }
    *sum = chk_sum;
    if(loop > 200)
        return -1;
    //read the checksum status
    chk_sum = M10MO_ReadOneHalfwordCRAM(i2c, 0x0F, 0x0A);
    *sum = chk_sum;
	return	0;
}

static int M10MO_FwWrite(struct i2c_client *i2c, char *fw_name)
{
	int	ret;
	unsigned short	write_sum = 0;
    unsigned int flash_locate = 0;
    unsigned char sendbuf[9];

    printk(KERN_ALERT"%s\n", __func__);

    // Write the sectors
    //ret = M10MO_FwWriteMain(i2c, fw_name);
    //begin to write the firmware
    printk(KERN_ALERT"Set the FLASH memory address\n");
    M10MO_WriteOneWordCRAM(i2c,  0x0F, 0x00, M10MO_FLASH_MEMORY_ADDRESS);
    printk(KERN_ALERT"Begin to erase the memory\n");
    M10MO_WriteOneByteCRAM(i2c, 0x0F, 0x06, 0x02);

    printk(KERN_ALERT"Waiting for the erase result.....\n");
    
    while(1)
    {
        ret = M10MO_ReadOneByteCRAM(i2c, 0x0f, 0x06);
        if( ret == 0x0)
        {
            printk(KERN_ALERT"Erase M10M0 whole chip DONE\n");
            break;
        }
        M10MO_Custom_Delay(500);
    }
    printk(KERN_ALERT"Set the M10MO PLL value 0x%x for 24MHz MCLK\n", M10MO_PLL_VALUE);
    M10MO_WriteOneWordCRAM(i2c, 0x0F, 0x1C, M10MO_PLL_VALUE);
    printk(KERN_ALERT"Set the destination of the SPI transmission\n");
    M10MO_WriteOneWordCRAM(i2c, 0x0F, 0x14, M10MO_DESTINATION_SIO_ADDRESS);
    printk(KERN_ALERT"set the SDRAM size\n");
    M10MO_WriteOneWordCRAM(i2c, 0x0F, 0x18, M10MO_SDRAM_SIZE);
    printk(KERN_ALERT"set the SDRAM\n");
    M10MO_WriteOneHalfwordCRAM(i2c, 0x0F, 0x18, 0x0608);
    printk(KERN_ALERT"set the SPI receive mode\n");
    M10MO_WriteOneByteCRAM(i2c, 0x0f, 0x4B, M10MO_SPI_RISING);
    M10MO_WriteOneByteCRAM(i2c, 0x0f, 0x4A, M10MO_SPI_MODE);
    M10MO_Custom_Delay(15);

    printk(KERN_ALERT"Begining to send the firmware...\n");
    if(m10mo_spi == NULL)
    {
        printk(KERN_ALERT"Don't detect the M10MO spi handle\n");
        return ret;
    }
    M10MO_Custom_StartReadFirmware(fw_name);
    while(flash_locate < M10MO_SDRAM_SIZE)
    {
        ret = M10MO_Custom_ReadFirmware(gM10MO_FwBuf, M10MO_FW_BUF_SIZE, flash_locate);
        if(ret == -1){
    	    printk(KERN_ALERT"M10MO: Error reading firmware\n");
    		M10MO_Custom_EndReadFirmware();
            break;
		}
        //set the destination address
        spi_write(m10mo_spi, gM10MO_FwBuf, M10MO_FW_BUF_SIZE);
        printk(KERN_ALERT"Transfer firmware data to SDRAM, from %x to %x\n",flash_locate, flash_locate+M10MO_FW_BUF_SIZE);
        flash_locate += M10MO_FW_BUF_SIZE;
	}

    M10MO_WriteOneWordCRAM(i2c, 0x0F, 0x00, M10MO_FLASH_MEMORY_ADDRESS);
    M10MO_WriteOneWordCRAM(i2c, 0x0F, 0x18, M10MO_SDRAM_SIZE); 
    printk(KERN_ALERT"Begining to program the flashROM\n");
    M10MO_WriteOneByteCRAM(i2c, 0x0F, 0x07, 0x01);
    
    printk(KERN_ALERT"Waiting for the flash programming done....\n");

    while(1)
    {
        ret = M10MO_ReadOneByteCRAM(i2c, 0x0F, 0x07);
        if( ret == 0x00)
        {
            printk(KERN_ALERT"Flash ROM program Done\n");
            break;
        }
        M10MO_Custom_Delay(500);
    }
    printk(KERN_ALERT"Write memory address [0x13000005] to 0x7f\n");
    sendbuf[8] = 0x7f;
    //set the ISP to boot
    M10MO_WriteMemory(i2c,sendbuf, 9, 0x13000005,0x1);
    M10MO_Custom_Delay(500);

	// Checksum
    M10MO_GetCheckSum(i2c,&write_sum);
    printk(KERN_ALERT"M10MO: Checksum is 0x%04x\n",write_sum);
    printk(KERN_ALERT"M10MO: Firmware write %s.\n",write_sum==0?"succeed":"failed");
	// write_sum = 0: OK, otherwise: NG
    M10MO_Custom_EndReadFirmware();
  	return ret;
}

static int M10MO_FwClear(struct i2c_client *i2c)
{
	int	ret;

    printk(KERN_ALERT"%s\n", __func__);

    // Write the sectors
    //ret = M10MO_FwWriteMain(i2c, fw_name);
    //begin to write the firmware
    printk(KERN_ALERT"Set the FLASH memory address\n");
    M10MO_WriteOneWordCRAM(i2c,  0x0F, 0x00, M10MO_FLASH_MEMORY_ADDRESS);
    printk(KERN_ALERT"Begin to erase the memory\n");
    M10MO_WriteOneByteCRAM(i2c, 0x0F, 0x06, 0x02);

    printk(KERN_ALERT"Waiting for the erase result.....\n");
    
    while(1)
    {
        ret = M10MO_ReadOneByteCRAM(i2c, 0x0f, 0x06);
        if( ret == 0x0)
        {
            printk(KERN_ALERT"Erase M10M0 whole chip DONE\n");
            break;
        }
        M10MO_Custom_Delay(500);
    }
  	return ret;
}

#if 0
static int M10MO_ReadMemory(struct i2c_client *i2c, unsigned char * rcv_buf, unsigned long addr, unsigned long rcv_num)
{
    unsigned char send_buf[16];
    int ret;

    send_buf[0] = 0x00;
    send_buf[1] = 0x03;	//CMD_EX_READ
    send_buf[2] = (unsigned char)((addr & 0xFF000000) >> 24);
    send_buf[3] = (unsigned char)((addr & 0x00FF0000) >> 16);
    send_buf[4] = (unsigned char)((addr & 0x0000FF00) >>  8);
    send_buf[5] = (unsigned char)( addr & 0x000000FF);
    send_buf[6] = (unsigned char)((rcv_num & 0xFF00) >> 8);
    send_buf[7] = (unsigned char)(rcv_num & 0x00FF);
    ret = m10mo_read_reg(i2c, send_buf, 8, rcv_buf, rcv_num+3); //rcv_num+3 since there are 3 bytes for I2C information (0x00, number of byte to read (bit15-08), number of byte to read (bit07-00))
    return ret;

}
#endif

static ssize_t m10mo_get_register(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret;
    u8 cnt = 1;
    u32 cat_addr, byte_N;
    struct m10mo_data *data = (struct m10mo_data *)dev_get_drvdata(dev);
    struct i2c_client *client = data->client;
    printk(KERN_ALERT"Get register - buff %s \n", buf);
    
    ret = sscanf(buf, "%hhd %x %x", &cnt, &cat_addr, &byte_N);
    if (3 != ret) {
        printk("%s: Invalid value. Format cnt %x cat_addr %x  byte_N %x ret:%d\n", __func__, cnt, cat_addr, byte_N,ret);
        goto exit;
    }
    printk(KERN_ALERT"m10mo: Get register - Format cnt %x cat_addr %x  byte_N %x \n", cnt, cat_addr, byte_N);
    switch(cnt)
    {
        case 1 :               
             ret = M10MO_ReadOneByteCRAM(client, cat_addr, byte_N);
            break;
        case 2 :
             ret = M10MO_ReadOneHalfwordCRAM(client, cat_addr, byte_N);
            break;
        case 4 :
           ret = M10MO_ReadOneWordCRAM(client, cat_addr, byte_N );
            break;	
#if 0				
        case 0 :
            M10MO_ReadMemory(struct i2c_client *i2c, unsigned char * rcv_buf, unsigned long addr, unsigned long rcv_num);
            break;
#endif
        default:
            printk("%s: Invalid count %d \n", __func__, cnt);
            break;
    }

    printk(KERN_ALERT"get value - 0x%x: \n", ret);
    uRegValue = (unsigned int)ret;
exit:
    return count;
}

static ssize_t m10mo_get_register_show(struct device *dev, struct device_attribute *attr,
    char *buf)
{
      return sprintf(buf, "%d\n", uRegValue);
}

static ssize_t m10mo_set_register(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret;
    int cnt, data_write,cat_addr;
    int byte_N;
    struct m10mo_data *data = (struct m10mo_data *)dev_get_drvdata(dev);
    struct i2c_client *client = data->client;

    ret = sscanf(buf, "%d %x %x %x", &cnt, &cat_addr, &byte_N, &data_write);
    if (4 != ret ) {
        printk("%s: Invalid value. Format cmd %d  byte %d data_write %d ret %d\n", __func__, cat_addr, byte_N, data_write,ret);
        goto exit;
    }

    printk(KERN_ALERT"m10mo: Set register - Format %d %d %x %x \n", cnt, cat_addr, byte_N, data_write);
    switch(cnt)
    {
        case 1 :            
            M10MO_WriteOneByteCRAM(client, cat_addr, byte_N, data_write);
            break;
        case 2 :

            M10MO_WriteOneHalfwordCRAM(client, cat_addr, byte_N, data_write );
            break;
        case 4 :

            M10MO_WriteOneWordCRAM(client, cat_addr, byte_N, data_write );
            break;					
        case 0 :
            {
                unsigned char tmp_addr[9];
                tmp_addr[8] = data_write;
                M10MO_WriteMemory(client, tmp_addr, 9, cat_addr, 1);
            }
            break;
        default:
            printk("%s: Invalid byte %d \n", __func__, cnt);
            break;
    }
exit:
    return count;
}

static ssize_t m10mo_load_firmware(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{

    struct m10mo_data *data = (struct m10mo_data *)dev_get_drvdata(dev);
    struct i2c_client *client = data->client;

    gpio_direction_output(ISP_RESET_L, 0);	/* active high */
    M10MO_Custom_Delay(50);
    gpio_set_value_cansleep(ISP_RESET_L, 1); 
    M10MO_Custom_Delay(100);

    M10MO_FwWrite(client,M10MO_FW_NAME);
    
    return count;

}

static ssize_t m10mo_clear_firmware(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{

    struct m10mo_data *data = (struct m10mo_data *)dev_get_drvdata(dev);
    struct i2c_client *client = data->client;

    gpio_direction_output(ISP_RESET_L, 0);	/* active high */
    M10MO_Custom_Delay(50);
    gpio_set_value_cansleep(ISP_RESET_L, 1); 
    M10MO_Custom_Delay(100);

    M10MO_FwClear(client);
    
    return count;

}

#define READ_FIRMWARE_SIZE  0x10000

static ssize_t m10mo_get_firmware(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)

{
    return count;
}

static ssize_t m10mo_get_version(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{

    struct m10mo_data *data = (struct m10mo_data *)dev_get_drvdata(dev);
    struct i2c_client *client = data->client;
    int i, ret;
    unsigned char sendbuf[9];
	
    sendbuf[8] = 0x7f;

    /*set the GPIO as output high */
    gpio_direction_output(ISP_RESET_L, 0);	/* active high */
    M10MO_Custom_Delay(50);
    gpio_set_value_cansleep(ISP_RESET_L, 1); 

    //set the ISP to boot
    M10MO_WriteMemory(client,sendbuf, 9, 0x13000005,0x1);
    M10MO_Custom_Delay(500);

    M10MO_WriteOneByteCRAM(client, 0x0F, 0x12, 0x01);
    M10MO_Custom_Delay(500);

    for(i = 0; i < 5; i++)
    {
        ret = M10MO_ReadOneByteCRAM(client, 0x0, 0x1C);
        if(ret == 0x01)
            break;
        M10MO_Custom_Delay(500);
    }

    if (ret == 0x01) {
        m10mo_ver.customer_code = M10MO_ReadOneHalfwordCRAM( client,  0x00, 0x00 );
        m10mo_ver.ver_firmware = M10MO_ReadOneHalfwordCRAM( client,  0x00, 0x02 );
        m10mo_ver.ver_hardware = M10MO_ReadOneHalfwordCRAM( client,  0x00, 0x04 );
        m10mo_ver.ver_parameter = M10MO_ReadOneHalfwordCRAM( client,  0x00, 0x06 );
    }

    printk(KERN_ALERT"m10mo: version %.4x-%.4x-%.4x-%.4x\n", 
            m10mo_ver.customer_code,
            m10mo_ver.ver_firmware,
            m10mo_ver.ver_hardware,
            m10mo_ver.ver_parameter);

    return count;
}

static ssize_t m10mo_monitor(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct m10mo_data *data = (struct m10mo_data *)dev_get_drvdata(dev);
    struct i2c_client *client = data->client;
    //int ret;

    //printk(KERN_ALERT"Set the mipi lanes to 4\n");
    //M10MO_WriteOneByteCRAM(client, 0x01, 0x3E, 0x4); 

    printk(KERN_ALERT"Set to parameter mode\n");
    M10MO_WriteOneByteCRAM(client, 0x00,0x0B,0x01); 
    M10MO_Custom_Delay(100);

    printk(KERN_ALERT"Set the resolution to 1920P30\n");
    M10MO_WriteOneByteCRAM(client, 0x01,0x01,0x28); 

    printk(KERN_ALERT"Set to monitor mode\n");
    M10MO_WriteOneByteCRAM(client, 0x00,0x0B,0x02); 
    M10MO_Custom_Delay(100);
    //ret = M10MO_ReadOneByteCRAM(client, 0x00, 0x0B);
    //printk(KERN_ALERT"Read 00 0B  %x", ret );

    printk(KERN_ALERT"Set to continouse AF\n");
    M10MO_WriteOneByteCRAM(client, 0x0A, 0x00, 0x06);
    M10MO_Custom_Delay(100);
    M10MO_WriteOneByteCRAM(client, 0x0A, 0x02, 0x01);
    M10MO_Custom_Delay(100);
    //ret = M10MO_ReadOneByteCRAM(client, 0x0A, 0x02);
    //printk(KERN_ALERT"Read 0A 02  %x", ret );

    printk(KERN_ALERT"Force Focus on start\n");  //by Yang
    M10MO_WriteOneByteCRAM(client, 0x0A, 0x00, 0x06);
    M10MO_Custom_Delay(100);
    M10MO_WriteOneByteCRAM(client, 0x0A, 0x02, 0x01);

    return count;
}

static ssize_t m10mo_rst(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret;
    int value;

    ret = sscanf(buf, "%d", &value);
    if (1 != ret) {
        printk(KERN_ALERT"m10m0 %s: Invalid format\n", __func__);
        goto exit;
    }

    printk(KERN_ALERT"m10mo %s: set ISP_RST GPIO pin %d to %d\n", __func__, ISP_RESET_L, value);
    gpio_direction_output(ISP_RESET_L, value);

exit:
    return count;
}

static DEVICE_ATTR(m10moread, S_IRUGO |S_IWUSR, m10mo_get_register_show, m10mo_get_register);
static DEVICE_ATTR(m10mowrite, S_IRUGO |S_IWUSR, NULL, m10mo_set_register);
static DEVICE_ATTR(loadfirmware, S_IRUGO |S_IWUSR, NULL, m10mo_load_firmware);
static DEVICE_ATTR(clearfirmware, S_IRUGO |S_IWUSR, NULL, m10mo_clear_firmware);
static DEVICE_ATTR(getfirmware, S_IRUGO |S_IWUSR, NULL, m10mo_get_firmware);
static DEVICE_ATTR(m10moversion, S_IRUGO |S_IWUSR, NULL, m10mo_get_version);
static DEVICE_ATTR(m10monitor, S_IRUGO |S_IWUSR, NULL, m10mo_monitor);
static DEVICE_ATTR(m10morst, S_IRUGO |S_IWUSR, NULL, m10mo_rst);

static struct attribute *m10mo_attributes[] = {
    &dev_attr_m10moread.attr,
    &dev_attr_m10mowrite.attr,
    &dev_attr_loadfirmware.attr,
    &dev_attr_clearfirmware.attr,
    &dev_attr_getfirmware.attr,
    &dev_attr_m10moversion.attr,
    &dev_attr_m10monitor.attr,
    &dev_attr_m10morst.attr,
    NULL
};

static const struct attribute_group m10mo_attr_group = {
    .attrs = m10mo_attributes,
};

static int m10mo_i2c_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    printk(KERN_ALERT"%s: Probing %x\n", __func__,client->addr);

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        printk("%s: need I2C_FUNC_I2C\n", __func__);
        ret = -EIO;
        return ret;
    }
    m10mo_des->client = client;
    i2c_set_clientdata(client, m10mo_des);
	
#if 0
    ret = gpio_request(ISP_RESET_L, "M10MO_ISP_REST");
    if(ret < 0)
        goto FreeRes_1;
    gpio_direction_output(ISP_RESET_L, 1);	/* active high */
    //do reset the M10MO chip that we don't reset it in the ducati
    M10MO_Custom_Delay(50);
    gpio_direction_output(ISP_RESET_L, 0);	/* active high */
    M10MO_Custom_Delay(50);
    gpio_set_value_cansleep(ISP_RESET_L, 1); 

#endif

#if AUXCLK2
	auxclk2_ck = clk_get(NULL, "auxclk2_ck");
	if (IS_ERR(auxclk2_ck)) {
		pr_err("Cannot request auxclk2\n");
	} else {
		clk_set_rate(auxclk2_ck, 19200000);
		clk_enable(auxclk2_ck);
	}
#endif
    mutex_init(&m10mo_des->lock);

    ret = sysfs_create_group(&client->dev.kobj, &m10mo_attr_group);
    if (ret) {
        dev_err(&client->dev, "failed to register sysfs hooks\n");
        goto err_op_failed;
    }

    /* release resources */
    //gpio_free(ISP_RESET_L);
    return 0;
//FreeRes_1:
//    printk(KERN_ALERT"%s GPIO request failed %d\n",__func__,ret);
err_op_failed:
    i2c_set_clientdata(client, NULL);
    return ret;
}

static int m10mo_i2c_suspend(struct i2c_client *c, pm_message_t state)
{
#if AUXCLK2
	clk_put(auxclk2_ck);
	clk_enable(auxclk2_ck);
#endif

	return 0;
}

static int m10mo_i2c_resume(struct i2c_client *c)
{
#if AUXCLK2
	auxclk2_ck = clk_get(NULL, "auxclk2_ck");
	if (IS_ERR(auxclk2_ck)) {
		pr_err("Cannot request auxclk2\n");
	} else {
		clk_set_rate(auxclk2_ck, 19200000);
		clk_enable(auxclk2_ck);
	}
#endif

	return 0;
}

static int m10mo_i2c_remove(struct i2c_client *client)
{
    struct m10mo_data *ddata;

    ddata = i2c_get_clientdata(client);
    sysfs_remove_group(&client->dev.kobj, &m10mo_attr_group);
    i2c_set_clientdata(client, NULL);
    printk(KERN_ALERT"%s: m10mo removed\n", __func__);

    return 0;
}
static int m10mo_spi_probe(struct spi_device *spi)
{
   printk(KERN_ALERT"%s spi probe %p\n",__func__, spi);
   m10mo_spi = spi;
   return 0;
}
static int m10mo_spi_remove(struct spi_device *spi)
{
    printk(KERN_ALERT"%s spi remove %p\n",__func__,m10mo_spi);
    m10mo_spi = NULL;
    return 0;
}
static const struct i2c_device_id m10mo_i2c_id[] = {
    { M10MO_NAME, 0 },
    { },
};

static struct i2c_driver m10mo_i2c_driver = {
    .driver = {
        .name = M10MO_NAME,
        .owner = THIS_MODULE,
    },
    .probe		= m10mo_i2c_probe,
	.suspend	= m10mo_i2c_suspend,
	.resume 	= m10mo_i2c_resume,
    .remove		= m10mo_i2c_remove,
    .id_table	= m10mo_i2c_id,
};

static struct spi_driver m10mo_spi_driver = {
    .driver = {
        .name = M10MO_NAME,
    },
    .probe = m10mo_spi_probe,
    .remove = m10mo_spi_remove,
};


static int m10mo_cdev_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int m10mo_cdev_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t m10mo_cdev_read(struct file *file, char *data, size_t size, loff_t *offset)
{
    return size;
}

static ssize_t m10mo_cdev_write(struct file *file, const char *data, size_t size, loff_t *offset)
{
    return size;
}

static long m10mo_cdev_ioctl_ver_get(unsigned long arg)
{
    if (copy_to_user((void __user *) arg, &m10mo_ver, sizeof(m10mo_ver)))
        return -EFAULT;

    return 0;
}

static long m10mo_cdev_ioctl_reg_set(unsigned long arg)
{
    struct i2c_client *client = m10mo_des->client;
    struct m10mo_reg m10mo_reg;

    memset(&m10mo_reg, 0x0, sizeof(m10mo_reg));
	if (copy_from_user(&m10mo_reg, (void __user *) arg, sizeof(m10mo_reg)))
		return -EFAULT;

    printk(KERN_INFO"m10mo: Set register - Format 0x%x 0x%x 0x%x 0x%x\n",
           m10mo_reg.cnt, 
           m10mo_reg.catagory, 
           m10mo_reg.offset, 
           m10mo_reg.value);

    switch(m10mo_reg.cnt)
    {
        case 1 :            
            M10MO_WriteOneByteCRAM(client, m10mo_reg.catagory, m10mo_reg.offset, m10mo_reg.value);
            break;
        case 2 :
            M10MO_WriteOneHalfwordCRAM(client, m10mo_reg.catagory, m10mo_reg.offset, m10mo_reg.value);
            break;
        case 4 :
            M10MO_WriteOneWordCRAM(client, m10mo_reg.catagory, m10mo_reg.offset, m10mo_reg.value);
            break;					
        case 0 :
            {
                unsigned char tmp_addr[9];
                tmp_addr[8] = m10mo_reg.value;
                M10MO_WriteMemory(client, tmp_addr, 9, m10mo_reg.catagory, 1);
            }
            break;
        default:
            break;
    }

    return 0;
}

static long m10mo_cdev_ioctl_reg_get(unsigned long arg)
{
    struct i2c_client *client = m10mo_des->client;
    struct m10mo_reg m10mo_reg;

    memset(&m10mo_reg, 0x0, sizeof(m10mo_reg));
	if (copy_from_user(&m10mo_reg, (void __user *) arg, sizeof(m10mo_reg)))
		return -EFAULT;
   
    printk(KERN_INFO"m10mo: Get register - Format 0x%x 0x%x 0x%x \n",
           m10mo_reg.cnt, 
           m10mo_reg.catagory, 
           m10mo_reg.offset);

    switch(m10mo_reg.cnt)
    {
        case 1 :               
             m10mo_reg.value = M10MO_ReadOneByteCRAM(client, m10mo_reg.catagory, m10mo_reg.offset);
            break;
        case 2 :
             m10mo_reg.value = M10MO_ReadOneHalfwordCRAM(client, m10mo_reg.catagory, m10mo_reg.offset);
            break;
        case 4 :
            m10mo_reg.value = M10MO_ReadOneWordCRAM(client, m10mo_reg.catagory, m10mo_reg.offset);
            break;	
        default:
            break;
    }

    if (copy_to_user((void __user *) arg, &m10mo_reg, sizeof(m10mo_reg)))
        return -EFAULT;
    
    printk(KERN_INFO"m10mo: Get register - Format 0x%x 0x%x 0x%x 0x%x\n",
           m10mo_reg.cnt, 
           m10mo_reg.catagory, 
           m10mo_reg.offset,
           m10mo_reg.value);

    return 0;
}

static long m10mo_cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) 
    {
	    case M10MO_VER_GET:
           return m10mo_cdev_ioctl_ver_get(arg);

        case M10MO_REG_SET:
           return m10mo_cdev_ioctl_reg_set(arg);

        case M10MO_REG_GET:
           return m10mo_cdev_ioctl_reg_get(arg);

        default:
            return -ENOTTY;
    }

    return 0;
}

static struct file_operations m10mo_cdev_fops = 
{
    .owner = THIS_MODULE,
    .open = m10mo_cdev_open,
    .release = m10mo_cdev_release,
    .read = m10mo_cdev_read,
    .write = m10mo_cdev_write,
	.unlocked_ioctl	= m10mo_cdev_ioctl,
};

static int __init m10mo_init(void)
{
    int ret;

    printk(KERN_ALERT"m10mo %s:\n", __func__);

    printk(KERN_ALERT"m10mo %s: init ISP_RST GPIO pin %d to 1\n", __func__, ISP_RESET_L);
    gpio_request(ISP_RESET_L, "M10MO_ISP_REST");
    gpio_direction_output(ISP_RESET_L, 1);	/* active high */

    memset(&m10mo_ver, 0x0, sizeof(m10mo_ver));

    m10mo_des= kzalloc(sizeof(struct m10mo_data), GFP_KERNEL);
    if (m10mo_des == NULL) {
        printk("%s: allocation error\n", __func__);
        ret = -ENOMEM;
        return ret;
    }
    
    ret = i2c_add_driver(&m10mo_i2c_driver);
    if( ret < 0 )
    {
        printk(KERN_ERR"%s cannot register Fujistu M10MO I2C driver\n",__func__);
        kfree(m10mo_des);
        return ret;
    }

    ret = spi_register_driver(&m10mo_spi_driver);
    if( ret < 0 )
    {
        printk(KERN_ERR"%s Cannot register Fujistu M10MO SPI driver\n",__func__);
        i2c_del_driver(&m10mo_i2c_driver);
        kfree(m10mo_des);
        return ret;
    }
    
    ret = alloc_chrdev_region(&m10mo_dev_t, 0, 1, "m10mo");
    if (ret < 0)
    {
        printk(KERN_ERR"%s Cannot alloc Fujistu M10MO chrdev_region\n",__func__);
        i2c_del_driver(&m10mo_i2c_driver);
        spi_unregister_driver(&m10mo_spi_driver);
        kfree(m10mo_des);
        return ret;
    }

    if ((m10mo_cls = class_create(THIS_MODULE, "m10mo")) == NULL)
    {
        printk(KERN_ERR"%s Cannot create Fujistu M10MO class\n",__func__);
        i2c_del_driver(&m10mo_i2c_driver);
        spi_unregister_driver(&m10mo_spi_driver);
        unregister_chrdev_region(m10mo_dev_t, 1);
        kfree(m10mo_des);
        return -1;
    }
    
    if (device_create(m10mo_cls, NULL, m10mo_dev_t, NULL, "m10mo") == NULL)
    {
        printk(KERN_ERR"%s Cannot create Fujistu M10MO device\n",__func__);
        i2c_del_driver(&m10mo_i2c_driver);
        spi_unregister_driver(&m10mo_spi_driver);
        class_destroy(m10mo_cls);
        unregister_chrdev_region(m10mo_dev_t, 1);
        kfree(m10mo_des);
        return -1;
    }

    m10mo_cdev = cdev_alloc();
    if (!m10mo_cdev) 
    {
        printk(KERN_ERR"%s Cannot alloc Fujistu M10MO cdev\n",__func__);
        i2c_del_driver(&m10mo_i2c_driver);
        spi_unregister_driver(&m10mo_spi_driver);
        class_destroy(m10mo_cls);
        unregister_chrdev_region(m10mo_dev_t, 1);
        kfree(m10mo_des);
        return -ENOMEM;
    }

    cdev_init(m10mo_cdev, &m10mo_cdev_fops);
    ret = cdev_add(m10mo_cdev, m10mo_dev_t, 1);
    if (ret < 0)
    {
        printk(KERN_ERR"%s Cannot add Fujistu M10MO cdev\n",__func__);
        i2c_del_driver(&m10mo_i2c_driver);
        spi_unregister_driver(&m10mo_spi_driver);
        class_destroy(m10mo_cls);
        unregister_chrdev_region(m10mo_dev_t, 1);
        kfree(m10mo_des);
    }

    return 0; 
}

static void __exit m10mo_exit(void)
{
    printk(KERN_ALERT"%s\n", __func__);

    i2c_del_driver(&m10mo_i2c_driver);
    spi_unregister_driver(&m10mo_spi_driver);
    cdev_del(m10mo_cdev);
    unregister_chrdev_region(m10mo_dev_t, 1);
    kfree(m10mo_des);
}

module_init(m10mo_init);
module_exit(m10mo_exit);

MODULE_DESCRIPTION("Fujistu M10MO firmware load");
MODULE_LICENSE("GPL");

