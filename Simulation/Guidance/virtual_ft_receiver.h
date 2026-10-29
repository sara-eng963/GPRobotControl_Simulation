#ifndef VIRTUAL_FT_RECEIVER_H
#define VIRTUAL_FT_RECEIVER_H

#include "virtual_ft_protocol.h"
#include "virtual_ft_clock.h"

typedef struct {
    uint64_t polls, accepted, rejected, last_poll_us, max_poll_gap_us;
    uint64_t last_accept_us, max_accept_gap_us, last_kernel_age_us;
    uint32_t last_packet_sequence;
    unsigned last_batch;
} VirtualFtRxDiagnostics;
typedef struct {
    bool captured;
    VirtualFtState before;
    VirtualFtFault reason;
    uint64_t check_mono_us;
    uint32_t cycle_us, age_us;
    VirtualFtRxDiagnostics rx;
} VirtualFtFirstFailure;

/* Host-only POSIX UDP, polled in the existing simulation input task.
 * No socket operation may occur in a Supervisor tick. */
typedef struct {
    int fd;
    VirtualFtLimits limits;
    VirtualFtState state;
    bool rejected_reported;
    VirtualFtRxDiagnostics diagnostics;
    VirtualFtFirstFailure first_failure;
} VirtualFtReceiver;

bool virtual_ft_receiver_init(VirtualFtReceiver *receiver,
                                 const VirtualFtLimits *limits);
void virtual_ft_receiver_poll(VirtualFtReceiver *receiver);
bool virtual_ft_receiver_read(VirtualFtReceiver *receiver,
                                 uint32_t now_us, double wrench[6]);
void virtual_ft_receiver_close(VirtualFtReceiver *receiver);
/* Admission is read-only: stale input while IDLE rejects selection, never
 * rearms a latch or invents a healthy sample. New sessions require zero. */
bool virtual_ft_receiver_ready(VirtualFtReceiver *receiver);
void virtual_ft_receiver_diagnostics(VirtualFtReceiver *receiver,
    VirtualFtRxDiagnostics *rx, VirtualFtFirstFailure *first);
#endif
