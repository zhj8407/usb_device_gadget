#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "diag_utils.h"

void printStatsNow(struct stats_collector * sc)
{
    if (NULL == sc) {
        printf("[Stats Collector]: invalid argument\n");
        return;
    }

    char stats_str[1024];
    memset(stats_str, 0, sizeof(stats_str));
    printf("[%s]: delta count=%lu-%lu=%lu, delta time=%lu-%lu=%lu ms\n"
           , sc->mod_name
           , sc->pkt_count_2, sc->pkt_count_1, sc->pkt_count_2 - sc->pkt_count_1
           , sc->pkt_time_2_ms, sc->pkt_time_2_ms, sc->pkt_time_2_ms - sc->pkt_time_1_ms
          );
}