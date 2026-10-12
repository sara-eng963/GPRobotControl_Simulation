/* POSIX recorder smoke test: produce task events and confirm background CSV
 * publication. Host-only; does not assert scheduling deadlines. */
#define _POSIX_C_SOURCE 200809L
#include "host_timing_trace.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int main(void)
{
    const char *path = "host_timing_trace_test.csv";
    remove(path);
    if (!gp_timing_init(path)) return 1;
    for (unsigned i = 0; i < 64U; ++i) {
        gp_timing_begin(GP_TIMING_SUPERVISOR);
        gp_timing_end(GP_TIMING_SUPERVISOR, true);
        gp_timing_begin(GP_TIMING_CAN_TARGET);
        gp_timing_end(GP_TIMING_CAN_TARGET, (i % 7U) != 0U);
    }
    struct timespec ts = {.tv_sec = 0, .tv_nsec = 150000000L};
    nanosleep(&ts, NULL); /* Let the writer drain before inspecting. */
    FILE *file = fopen(path, "r");
    if (!file) return 2;
    char line[512];
    unsigned lines = 0U, supervisor = 0U, can = 0U;
    if (!fgets(line, sizeof(line), file) ||
        strstr(line, "execution_budget_overrun") == NULL) return 3;
    while (fgets(line, sizeof(line), file)) {
        ++lines;
        if (strstr(line, ",supervisor,periodic,")) ++supervisor;
        if (strstr(line, ",can_targets,event,")) ++can;
    }
    fclose(file);
    if (lines != 128U || supervisor != 64U || can != 64U) {
        fprintf(stderr,"expected 128 measurements, got %u (%u supervisor, %u CAN)\n",
                lines, supervisor, can);
        return 4;
    }
    puts("PASS: async writer committed 128 correctly labeled timing CSV rows");
    return 0;
}
