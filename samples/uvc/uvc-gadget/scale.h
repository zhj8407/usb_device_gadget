#ifndef __SCALE_H__
#define __SCALE_H__
#include <stdint.h>

#define MULTI_THREAD 1

#define COLOR_COMPONENTS    3
struct scaler_info_t {
    uint32_t m_column_id;
    uint32_t m_row_id;
    //struct video_buffer_t* v_in_buf;
    //struct video_buffer_t* v_out_buf;
    //struct block_param_t* v_out_blk_par;

};
#ifdef MULTI_THREAD
pthread_cond_t scale_start;
pthread_cond_t scale_finish;
pthread_mutex_t main_lock;
extern int scale_done_count;
#endif

#define ROW_NUM 4
#define COLUMN_NUM 4
struct scaler_info_t s_info[ROW_NUM][COLUMN_NUM];
pthread_t pid[ROW_NUM][COLUMN_NUM];

struct video_buffer_t {
    uint8_t * m_buffer;
    uint32_t m_width;
    uint32_t m_height;
};

struct video_buffer_t g_v_in_buf;
struct video_buffer_t g_v_out_buf;


//uint8_t * pOutBuf;


//extern void bilineary_scale(char * inBuffer, uint32_t inWidth, uint32_t inHeight, char * outBuffer, uint32_t outWidth, uint32_t outHeight, uint32_t inStep, uint32_t outStep);
extern void scaleNV12(uint8_t * inBuffer, unsigned int inWidth, unsigned int inHeight, uint8_t * outBuffer, uint32_t outWidth, uint32_t outHeight);
extern void NV12toI420(uint8_t * inBuffer, unsigned int inWidth, unsigned int inHeight, uint8_t * outBuffer, uint32_t outWidth, uint32_t outHeight);
extern void NV12toI420NonCpy(uint8_t * inBuffer, unsigned int inWidth, unsigned int inHeight, uint32_t outWidth, uint32_t outHeight);

extern void NV12toI420scale(uint8_t * inBuffer, unsigned int inWidth, unsigned int inHeight, uint8_t * outBuffer, uint32_t outWidth, uint32_t outHeight);
extern void NV12toYUY2(uint8_t * inBuffer, unsigned int inWidth, unsigned int inHeight, uint8_t * outBuffer, uint32_t outWidth, uint32_t outHeight);
extern void NV12toYUY2scale(uint8_t * inBuffer, unsigned int inWidth, unsigned int inHeight, uint8_t * outBuffer, uint32_t outWidth, uint32_t outHeight);

extern void yuvtorgb(unsigned int Y, unsigned int U, unsigned int V, unsigned int *r_ptr, unsigned int *g_ptr, unsigned int *b_ptr);
extern void get_rgb(unsigned char *src,
                    unsigned int x,
                    unsigned int  y,
                    unsigned int width,
                    unsigned int height,
                    unsigned int *r_value,
                    unsigned int *g_value,
                    unsigned int *b_value,
                    unsigned int format);

extern void get_rgb(unsigned char *src,
                    unsigned int x,
                    unsigned int  y,
                    unsigned int width,
                    unsigned int height,
                    unsigned int *r_value,
                    unsigned int *g_value,
                    unsigned int *b_value,
                    unsigned int format);

extern int NV12toRGBA(unsigned char *yuv_buffer,
                      unsigned int width, unsigned int height,
                      unsigned char *rgb_buffer);

void create_scaler_thread(uint32_t colNum, uint32_t rowNum);


#endif

