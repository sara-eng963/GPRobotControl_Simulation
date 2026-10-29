#ifndef VIRTUAL_FT_CLIENT_H
#define VIRTUAL_FT_CLIENT_H

/* Standalone host-only UDP sender. Sends force samples, never joint targets. */
#include "virtual_ft_protocol.h"
#include <netinet/in.h>

typedef struct {
    int socket_fd;
    uint32_t next_sequence;
    struct sockaddr_in destination;
} VirtualFtClient;

bool virtual_ft_client_open(VirtualFtClient *client);
bool virtual_ft_client_send(VirtualFtClient *client,
                               const double wrench_base_tcp[6]);
bool virtual_ft_client_send_enabled(VirtualFtClient *client, bool enabled,
                                       const double wrench_base_tcp[6]);
/* On normal exit send zero samples before closing. A stopped GUI's stream
 * still expires in the Supervisor watchdog; this is NOT a safety stop. */
void virtual_ft_client_close(VirtualFtClient *client);
#endif
