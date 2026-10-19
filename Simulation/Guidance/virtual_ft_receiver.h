#ifndef GP_VIRTUAL_FT_RECEIVER_H
#define GP_VIRTUAL_FT_RECEIVER_H

#include "virtual_ft_protocol.h"

/* Host-only POSIX UDP, polled in the existing simulation input task.
 * No socket operation may occur in a Supervisor tick. */
typedef struct {
    int fd;
    GpVirtualFtLimits limits;
    GpVirtualFtState state;
    bool rejected_reported;
} GpVirtualFtReceiver;

bool gp_virtual_ft_receiver_init(GpVirtualFtReceiver *receiver,
                                 const GpVirtualFtLimits *limits);
void gp_virtual_ft_receiver_poll(GpVirtualFtReceiver *receiver);
bool gp_virtual_ft_receiver_read(GpVirtualFtReceiver *receiver,
                                 uint32_t now_us, double wrench[6]);
void gp_virtual_ft_receiver_close(GpVirtualFtReceiver *receiver);
#endif
