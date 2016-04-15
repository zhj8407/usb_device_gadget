#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <string.h>
#include "audiolinxtypes.h"
#include "audiolinx.h"

#define DATA "Half a league, half a league . . ."
static int linkid=-1;

/* Create a client endpoint and connect to a server.   Returns fd if all OK, <0 on error. */
static int unix_socket_conn(const char *servername)
{
    int fd;
    if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0)    /* create a UNIX domain stream socket */
    {
        return(-1);
    }
    int len, rval;
    struct sockaddr_un un;
    memset(&un, 0, sizeof(un));            /* fill socket address structure with our address */
    un.sun_family = AF_UNIX;
    sprintf(un.sun_path, "/tmp/scktmp%05d", getpid());
    len = offsetof(struct sockaddr_un, sun_path) + strlen(un.sun_path);
    unlink(un.sun_path);               /* in case it already exists */
    if (bind(fd, (struct sockaddr *)&un, len) < 0)
    {
        printf("Error[%d] when Sending Data:%s.\n",errno,strerror(errno));
        rval = -2;
    }
    else
    {
        /* fill socket address structure with server's address */
        memset(&un, 0, sizeof(un));
        un.sun_family = AF_UNIX;
        strcpy(un.sun_path, servername);
        len = offsetof(struct sockaddr_un, sun_path) + strlen(servername);
        if (connect(fd, (struct sockaddr *)&un, len) < 0)
        {
            rval= -4;
        }
        else
        {
            return (fd);
        }
    }
    int err;
    err = errno;
    close(fd);
    errno = err;
    return rval;
}

static void unix_socket_close(int fd)
{
    close(fd);
}

int audio_linx_open(const char *name)
{
    int sock;
    sock = unix_socket_conn(name);
    linkid = sock;
    return sock;
}

int audio_linx_get_id()
{
    return linkid;
}

int audio_linx_write(int linkid, short *buffers, int size)
{
    int sock = linkid;
    int wsize = -1;
    int err;
    if(sock)
    {
        wsize = send(sock, buffers, size, 0);
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
    int sock = linkid;
    if(sock)
        unix_socket_close(sock);
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
