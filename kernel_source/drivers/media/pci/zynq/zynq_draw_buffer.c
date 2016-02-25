
#define TO_Y(r, g, b) \
	(((16829 * r + 33039 * g + 6416 * b  + 32768) >> 16) + 16)
/* RGB to  V(Cr) Color transform */
#define TO_V(r, g, b) \
	(((28784 * r - 24103 * g - 4681 * b  + 32768) >> 16) + 128)
/* RGB to  U(Cb) Color transform */
#define TO_U(r, g, b) \
	(((-9714 * r - 19070 * g + 28784 * b + 32768) >> 16) + 128)

static int draw_yuv422p(void *vbuf, int r0, int g0, int b0, int r1, int g1, int b1, int width, int height, int stride, int count)
{
    unsigned char *y_buffer = (unsigned char *)vbuf;
    unsigned int u_offset = stride * height;
    unsigned int v_offset =u_offset + (stride /2) * height;
    unsigned char *u_buffer = y_buffer + u_offset; //Cb
    unsigned char *v_buffer = y_buffer + v_offset;//Cr
    int  i = 0, j =0;
    int r =  0;
    int g = 0;
    int b = 0;

    if (!vbuf) return -1;

    if ((count % 2) == 0) {
        r =  r0;
        g = g0;
        b = b0;
    } else {
        r = r1;
        g = g1;
        b = b1;
    }

    for (i = 0; i < height ; i++)
        for (j = 0 ; j < width; j++ )
            *(y_buffer+i*stride+j) = TO_Y(r, g, b);

    for (i = 0; i < height ; i++)
        for (j = 0 ; j < (width/2); j++ )
            *(u_buffer+i*(stride/2)+j) = TO_U(r, g, b);

    for (i = 0; i < height ; i++)
        for (j = 0 ; j < (width/2); j++ )
            *(v_buffer+i*(stride/2)+j) = TO_V(r, g, b);

    return 0;
}

static int draw_yuv420(void *vbuf, int r0, int g0, int b0, int r1, int g1, int b1, int width, int height, int stride, int count)
{

    unsigned char *y_buffer = (unsigned char *)vbuf;
    unsigned int u_offset = stride * height;
    unsigned int v_offset =u_offset + (stride /2) * (height/2);
    unsigned char *u_buffer = y_buffer + u_offset; //Cb
    unsigned char *v_buffer = y_buffer + v_offset;//Cr
    int  i = 0, j =0;
    int r =  0;
    int g = 0;
    int b = 0;

    if (!vbuf) return -1;

    if ((count % 2) == 0) {
        r =  r0;
        g = g0;
        b = b0;
    } else {
        r = r1;
        g = g1;
        b = b1;
    }

    for (i = 0; i < height ; i++)
        for (j = 0 ; j < width; j++ )
            *(y_buffer+i*stride+j) = TO_Y(r, g, b);

    for (i = 0; i < height/2 ; i++)
        for (j = 0 ; j < (width/2); j++ )
            *(u_buffer+i*(stride/2)+j) = TO_U(r, g, b);

    for (i = 0; i < height /2; i++)
        for (j = 0 ; j < (width/2); j++ )
            *(v_buffer+i*(stride/2)+j) = TO_V(r, g, b);

    return 0;
}