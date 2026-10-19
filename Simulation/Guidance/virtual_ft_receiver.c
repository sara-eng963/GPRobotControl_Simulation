#include "virtual_ft_receiver.h"

#include "FreeRTOS.h"
#include "task.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

bool gp_virtual_ft_receiver_init(GpVirtualFtReceiver *r,
                                 const GpVirtualFtLimits *limits)
{
    if (r == NULL || limits == NULL) return false;
    memset(r, 0, sizeof(*r));
    r->fd = -1;
    r->limits = *limits;
    r->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (r->fd < 0) { perror("[VFT] socket"); return false; }
    struct sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_port = htons(GP_VFT_PORT);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(r->fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        fcntl(r->fd, F_SETFL, O_NONBLOCK) != 0) {
        perror("[VFT] bind/nonblocking");
        gp_virtual_ft_receiver_close(r);
        return false;
    }
    fprintf(stderr, "[VFT] interactive base/TCP wrench UDP listening on "
                    "127.0.0.1:%u; watchdog=%u us\n",
            (unsigned)GP_VFT_PORT,(unsigned)r->limits.timeout_us);
    return true;
}

void gp_virtual_ft_receiver_poll(GpVirtualFtReceiver *r, uint32_t now_us)
{
    if (r == NULL || r->fd < 0) return;
    /* Bounded work at HMI priority; last accepted packet wins. */
    for (unsigned i=0; i<16U; ++i) {
        uint8_t packet[GP_VFT_PACKET_SIZE+1U];
        struct sockaddr_in peer={0};
        socklen_t len=sizeof(peer);
        const ssize_t got=recvfrom(r->fd, packet, sizeof(packet),
                                   MSG_DONTWAIT, (struct sockaddr*)&peer,&len);
        if (got < 0) break; /* Nonblocking empty/error; watchdog still applies */
        const bool local=peer.sin_family==AF_INET &&
            peer.sin_addr.s_addr==htonl(INADDR_LOOPBACK);
        taskENTER_CRITICAL();
        const bool accepted=local && gp_vft_accept(&r->state,&r->limits,
                                     packet,(size_t)got,now_us);
        if (!local) {
            /* Local-only bind already rejects remote datagrams; fail closed
             * even if an unexpected peer reached this socket. */
            (void)gp_vft_accept(&r->state,&r->limits,NULL,0U,now_us);
        }
        const bool latched=r->state.fault_latched;
        taskEXIT_CRITICAL();
        if (!accepted && latched && !r->rejected_reported) {
            fprintf(stderr, "[VFT] input protocol/sequence fault latched; "
                            "restart simulation to rearm\n");
            r->rejected_reported=true;
        }
    }
}

bool gp_virtual_ft_receiver_read(GpVirtualFtReceiver *r, uint32_t now_us,
                                 double wrench[6])
{
    if (wrench == NULL) return false;
    memset(wrench, 0, 6U*sizeof(double));
    if (r == NULL || r->fd < 0) return false;
    taskENTER_CRITICAL();
    const bool ok=gp_vft_read(&r->state,&r->limits,now_us,wrench);
    taskEXIT_CRITICAL();
    return ok;
}

void gp_virtual_ft_receiver_close(GpVirtualFtReceiver *r)
{
    if (r == NULL) return;
    if (r->fd >= 0) close(r->fd);
    r->fd=-1;
}
