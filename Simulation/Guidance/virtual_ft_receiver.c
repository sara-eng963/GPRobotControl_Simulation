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

bool virtual_ft_receiver_init(VirtualFtReceiver *r,
                                 const VirtualFtLimits *limits)
{
    if (r == NULL || limits == NULL) return false;
    memset(r, 0, sizeof(*r));
    r->fd = -1;
    r->limits = *limits;
    r->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (r->fd < 0) { perror("[VFT] socket"); return false; }
    struct sockaddr_in address = {0};
    address.sin_family = AF_INET;
    address.sin_port = htons(VIRTUAL_FT_PORT);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(r->fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        fcntl(r->fd, F_SETFL, O_NONBLOCK) != 0) {
        perror("[VFT] bind/nonblocking");
        virtual_ft_receiver_close(r);
        return false;
    }
    fprintf(stderr, "[VFT] interactive base/TCP wrench UDP listening on "
                    "127.0.0.1:%u; watchdog=%u us\n",
            (unsigned)VIRTUAL_FT_PORT,(unsigned)r->limits.timeout_us);
    return true;
}

void virtual_ft_receiver_poll(VirtualFtReceiver *r)
{
    if (r == NULL || r->fd < 0) return;
    /* Bounded input acquisition; last accepted packet wins. */
    for (unsigned i=0; i<16U; ++i) {
        uint8_t packet[VIRTUAL_FT_PACKET_SIZE+1U];
        struct sockaddr_in peer={0};
        socklen_t len=sizeof(peer);
        const ssize_t got=recvfrom(r->fd, packet, sizeof(packet),
                                   MSG_DONTWAIT, (struct sockaddr*)&peer,&len);
        if (got < 0) break; /* Nonblocking empty/error; watchdog still applies */
        const bool local=peer.sin_family==AF_INET &&
            peer.sin_addr.s_addr==htonl(INADDR_LOOPBACK);
        taskENTER_CRITICAL();
        /* Timestamp acceptance, not entry to a possibly preempted poll. */
        const uint32_t now_us = (uint32_t)(
            (uint64_t)xTaskGetTickCount() * UINT64_C(1000000) /
            (uint64_t)configTICK_RATE_HZ);
        const bool accepted=local && virtual_ft_accept(&r->state,&r->limits,
                                     packet,(size_t)got,now_us);
        if (!local) {
            /* Local-only bind already rejects remote datagrams; fail closed
             * even if an unexpected peer reached this socket. */
            (void)virtual_ft_accept(&r->state,&r->limits,NULL,0U,now_us);
        }
        const bool latched=r->state.fault_latched;
        const VirtualFtFault reason=r->state.fault_reason;
        taskEXIT_CRITICAL();
        if (!accepted && latched && !r->rejected_reported) {
            fprintf(stderr, "[VFT] input fault latched (%s); "
                            "restart simulation to rearm\n",
                    virtual_ft_fault_name(reason));
            r->rejected_reported=true;
        }
    }
}

bool virtual_ft_receiver_read(VirtualFtReceiver *r, uint32_t now_us,
                                 double wrench[6])
{
    if (wrench == NULL) return false;
    memset(wrench, 0, 6U*sizeof(double));
    if (r == NULL || r->fd < 0) return false;
    taskENTER_CRITICAL();
    const bool ok=virtual_ft_read(&r->state,&r->limits,now_us,wrench);
    taskEXIT_CRITICAL();
    return ok;
}

void virtual_ft_receiver_close(VirtualFtReceiver *r)
{
    if (r == NULL) return;
    if (r->fd >= 0) close(r->fd);
    r->fd=-1;
}
