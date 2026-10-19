#ifndef GP_VIRTUAL_FT_CLIENT_H
#define GP_VIRTUAL_FT_CLIENT_H

/* Standalone host-only UDP sender. Sends force samples, never joint targets. */
#include "virtual_ft_protocol.h"
#include <netinet/in.h>

typedef struct {
    int socket_fd;
    uint32_t next_sequence;
    struct sockaddr_in destination;
} GpVirtualFtClient;

bool gp_virtual_ft_client_open(GpVirtualFtClient *client);
bool gp_virtual_ft_client_send(GpVirtualFtClient *client,
                               const double wrench_base_tcp[6]);
/* On normal exit send zero samples before closing. A stopped GUI's stream
 * still expires in the Supervisor watchdog; this is NOT a safety stop. */
void gp_virtual_ft_client_close(GpVirtualFtClient *client);
#endif
