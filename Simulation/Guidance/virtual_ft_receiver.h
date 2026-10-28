#ifndef VIRTUAL_FT_RECEIVER_H
#define VIRTUAL_FT_RECEIVER_H

#include "virtual_ft_protocol.h"

/* Host-only POSIX UDP, polled in the existing simulation input task.
 * No socket operation may occur in a Supervisor tick. */
typedef struct {
    int fd;
    VirtualFtLimits limits;
    VirtualFtState state;
    bool rejected_reported;
} VirtualFtReceiver;

bool virtual_ft_receiver_init(VirtualFtReceiver *receiver,
                                 const VirtualFtLimits *limits);
void virtual_ft_receiver_poll(VirtualFtReceiver *receiver);
bool virtual_ft_receiver_read(VirtualFtReceiver *receiver,
                                 uint32_t now_us, double wrench[6]);
void virtual_ft_receiver_close(VirtualFtReceiver *receiver);
#endif
