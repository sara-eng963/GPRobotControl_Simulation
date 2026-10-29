#ifndef VIRTUAL_FT_CLOCK_H
#define VIRTUAL_FT_CLOCK_H
#include <stdint.h>
#include <time.h>
/* Linux/WSL host clock, independent of POSIX-port tick catch-up. */
static inline uint64_t virtual_ft_monotonic_us(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC,&t)!=0) return 0;
    return (uint64_t)t.tv_sec*1000000U+(uint64_t)t.tv_nsec/1000U;
}
#endif
