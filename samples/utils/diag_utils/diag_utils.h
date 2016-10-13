#ifndef UVC_DIAG_UTILS_H
#define UVC_DIAG_UTILS_H
#include <sys/time.h>
#include <time.h>

#define STATS_PRINT_INTERVAL 300
static inline long GetTimeInMicroSec()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    long ret = tv.tv_sec * 1000 * 1000 + tv.tv_usec;
    return ret;
}
static inline unsigned long GetTimeInMilliSec()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}
static inline unsigned long GetTimeInUSec()
{
    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);
    return (tv.tv_sec * 1000 * 1000) + (tv.tv_nsec);
}

struct stats_collector {
    char mod_name[64];
    unsigned long stats_interval_ms;
    unsigned long stats_interval_cnt;
    unsigned long pkt_count_1;
    unsigned long pkt_count_2;
    unsigned long pkt_time_1_ms;
    unsigned long pkt_time_2_ms;
};

void printStatsNow(struct stats_collector * sc);


#endif
