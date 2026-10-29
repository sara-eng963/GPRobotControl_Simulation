#ifndef VIRTUAL_FT_SENDER_H
#define VIRTUAL_FT_SENDER_H
/* Host GUI only. The worker owns the socket and sequence for its lifetime. */
#include "virtual_ft_client.h"
#include <pthread.h>
#define VIRTUAL_FT_SEND_PERIOD_US 20000U
#define VIRTUAL_FT_COMMAND_LEASE_US 80000U
typedef struct {
    uint64_t sent, last_send_us, max_send_gap_us, missed_releases;
    uint32_t last_sequence;
    double last_wrench[6];
    bool failed, command_expired;
} VirtualFtSenderStatus;
typedef struct {
    pthread_t thread;
    pthread_mutex_t lock;
    VirtualFtClient client;
    double command[6];
    uint64_t updated_us;
    VirtualFtSenderStatus status;
    bool running, stop;
} VirtualFtSender;
bool virtual_ft_sender_start(VirtualFtSender *sender);
void virtual_ft_sender_update(VirtualFtSender *sender, const double wrench[6]);
void virtual_ft_sender_status(VirtualFtSender *sender, VirtualFtSenderStatus *status);
void virtual_ft_sender_stop(VirtualFtSender *sender);
#endif
