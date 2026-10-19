#include "virtual_ft_client.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <stddef.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

bool gp_virtual_ft_client_open(GpVirtualFtClient *client)
{
    if (client == NULL) return false;
    memset(client, 0, sizeof(*client));
    client->socket_fd = -1;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(fd);
        return false;
    }
    client->destination.sin_family = AF_INET;
    client->destination.sin_port = htons(GP_VFT_PORT);
    client->destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    client->socket_fd = fd;
    client->next_sequence = 1U;
    return true;
}

bool gp_virtual_ft_client_send(GpVirtualFtClient *client,
                               const double wrench_base_tcp[6])
{
    if (client == NULL || client->socket_fd < 0) return false;
    uint8_t packet[GP_VFT_PACKET_SIZE];
    if (!gp_vft_encode(client->next_sequence, true, wrench_base_tcp, packet))
        return false;
    const ssize_t count = sendto(client->socket_fd, packet, sizeof(packet),
                                MSG_DONTWAIT,
                                (const struct sockaddr *)&client->destination,
                                sizeof(client->destination));
    if (count != (ssize_t)sizeof(packet)) return false;
    ++client->next_sequence;
    return true;
}

void gp_virtual_ft_client_close(GpVirtualFtClient *client)
{
    if (client == NULL) return;
    if (client->socket_fd >= 0) {
        const double zero[6] = {0};
        for (unsigned i=0; i<3U; ++i)
            (void)gp_virtual_ft_client_send(client, zero);
        close(client->socket_fd);
    }
    client->socket_fd = -1;
}
