#include <sys/types.h>
#include <stdio.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include "audiolinxtypes.h"
#include "audiolinx.h"

static int linkid=-1;

static int unix_fifo_conn()
{
	int fd;
	if ((fd =open(DEFAULT_FIFO_NAME,O_RDWR)) < 0)    /* create a fifo */
	{
		printf("Error[%d] open error:%s.\n",errno,strerror(errno));
		return(-1);
	}

	return fd;
}

static void unix_socket_close(int fd)
{
	close(fd);
}

int audio_linx_open(const char *name)
{
	int fd;
	fd = unix_fifo_conn();
	linkid = fd;
	return fd;
}

int audio_linx_get_id()
{
	return linkid;
}

int audio_linx_write(int linkid, short *buffers, int size)
{
	int fd = linkid;
	int wsize = -1;
	int err;
	if(fd)
	{
		wsize = write(fd, buffers, size, 0);
		if(wsize>=0)
		{
			//printf("Data[%d] Sended:%c.\n",size,buffers[0]);
			err = 0;
		}
		if(wsize==-1)
		{
			printf("Error[%d] when Sending Data:%s.\n",errno,strerror(errno));
			err = errno;
		}
	}

	return err;
}

int audio_linx_close(linkid)
{
	int fd = linkid;
	if(fd)
		unix_socket_close(fd);
	return 0;
}

/*
   main(argc, argv)
   int argc;
   char *argv[];
   {
   int linkid;
   linkid = audio_linx_open("audiosocket");
   audio_linx_write(linkid);
   audio_linx_close(linkid);
   }
   */
