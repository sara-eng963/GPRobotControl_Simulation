#define _GNU_SOURCE
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
#include <errno.h>
#include <time.h>

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
    const int timestamping=1;
    if (setsockopt(r->fd,SOL_SOCKET,SO_TIMESTAMPNS,&timestamping,sizeof(timestamping))!=0 ||
        bind(r->fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
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

static void capture(VirtualFtReceiver *r, const VirtualFtState *before,
                    VirtualFtFault reason, uint64_t now, uint32_t cycle)
{
    if (r->first_failure.captured) return;
    r->first_failure=(VirtualFtFirstFailure){.captured=true,.before=*before,
        .reason=reason,.check_mono_us=now,.cycle_us=cycle,
        .age_us=(uint32_t)now-before->received_us,.rx=r->diagnostics};
}

void virtual_ft_receiver_poll(VirtualFtReceiver *r)
{
    if (r==NULL || r->fd<0) return;
    const uint64_t poll=virtual_ft_monotonic_us();
    taskENTER_CRITICAL();
    VirtualFtRxDiagnostics *d=&r->diagnostics;
    const uint64_t gap=d->last_poll_us ? poll-d->last_poll_us : 0;
    if (gap>d->max_poll_gap_us) d->max_poll_gap_us=gap;
    d->last_poll_us=poll; ++d->polls;d->last_batch=0;
    taskEXIT_CRITICAL();
    for (unsigned i=0;i<16U;++i) {
        uint8_t packet[VIRTUAL_FT_PACKET_SIZE+1U];
        struct sockaddr_in peer={0};
        union { struct cmsghdr align; unsigned char bytes[CMSG_SPACE(sizeof(struct timespec))]; } control;
        struct iovec io={packet,sizeof(packet)};
        struct msghdr msg={.msg_name=&peer,.msg_namelen=sizeof(peer),
            .msg_iov=&io,.msg_iovlen=1,.msg_control=control.bytes,.msg_controllen=sizeof(control.bytes)};
        const ssize_t got=recvmsg(r->fd,&msg,MSG_DONTWAIT);
        if (got<0 && (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR)) break;
        struct timespec arrival={0},wall={0};
        bool stamped=false;
        if (got>=0 && !(msg.msg_flags&(MSG_TRUNC|MSG_CTRUNC)))
            for (struct cmsghdr *c=CMSG_FIRSTHDR(&msg);c;c=CMSG_NXTHDR(&msg,c))
                if (c->cmsg_level==SOL_SOCKET && c->cmsg_type==SCM_TIMESTAMPNS &&
                    c->cmsg_len>=CMSG_LEN(sizeof(arrival))) {
                    memcpy(&arrival,CMSG_DATA(c),sizeof(arrival));stamped=true;
                }
        /* Timestamp kernel arrival, NOT dequeue. An old buffered nonzero
         * packet must never become fresh just because polling resumes.
         * Linux SO_TIMESTAMPNS uses realtime; convert its AGE conservatively
         * to the local monotonic domain. Backward/inconsistent wall samples
         * fail closed. No wall-clock value enters the controller pipeline. */
        const uint64_t now=virtual_ft_monotonic_us();
        const bool wall_ok=clock_gettime(CLOCK_REALTIME,&wall)==0;
        const int64_t age_ns=((int64_t)wall.tv_sec-arrival.tv_sec)*INT64_C(1000000000)+wall.tv_nsec-arrival.tv_nsec;
        const bool time_ok=stamped && wall_ok && now!=0 && age_ns>=0 &&
            (uint64_t)age_ns/1000U<UINT32_C(0x80000000) && (uint64_t)age_ns/1000U<=now;
        const uint64_t queue_age=time_ok ? ((uint64_t)age_ns+999U)/1000U : 0;
        taskENTER_CRITICAL();
        const VirtualFtState before=r->state;
        ++d->last_batch;
        d->last_kernel_age_us=queue_age;
        if (got>=12) d->last_packet_sequence=((uint32_t)packet[8]<<24)|((uint32_t)packet[9]<<16)|((uint32_t)packet[10]<<8)|packet[11];
        const bool local=peer.sin_family==AF_INET && peer.sin_addr.s_addr==htonl(INADDR_LOOPBACK);
        bool accepted=false;
        if (!time_ok && !r->state.fault_latched) {
            r->state.fault_latched=true;r->state.fault_reason=VIRTUAL_FT_FAULT_CLOCK;
            r->state.enabled=false;memset(r->state.wrench_base_tcp,0,sizeof(r->state.wrench_base_tcp));
        } else if (time_ok) {
            accepted=virtual_ft_accept(&r->state,&r->limits,local ? packet : NULL,
                got>=0 ? (size_t)got : 0,(uint32_t)(now-queue_age));
        }
        if (accepted) {
            const uint64_t accept_gap=d->last_accept_us ? now-d->last_accept_us : 0;
            if (accept_gap>d->max_accept_gap_us) d->max_accept_gap_us=accept_gap;
            d->last_accept_us=now;++d->accepted;
        } else {
            ++d->rejected;
            capture(r,&before,r->state.fault_reason,now,0);
        }
        const bool latched=r->state.fault_latched;
        const VirtualFtFault reason=r->state.fault_reason;
        taskEXIT_CRITICAL();
        if (latched && !r->rejected_reported) {
            fprintf(stderr,"[VFT] input fault latched (%s); restart simulation to rearm\n",virtual_ft_fault_name(reason));
            r->rejected_reported=true;
        }
        if (got<0) break;
    }
}

bool virtual_ft_receiver_read(VirtualFtReceiver *r,uint32_t cycle_us,double wrench[6])
{
    if (!wrench) return false;
    memset(wrench,0,6U*sizeof(double));
    if (!r || r->fd<0) return false;
    taskENTER_CRITICAL();
    const VirtualFtState before=r->state;
    /* Capture now together with state. The cycle timestamp can precede an
     * input-task preemption/acceptance and must not create a wrapped age. */
    const uint64_t now=virtual_ft_monotonic_us();
    const bool ok=virtual_ft_read(&r->state,&r->limits,(uint32_t)now,wrench);
    if (!ok) capture(r,&before,r->state.fault_reason,now,cycle_us);
    taskEXIT_CRITICAL();
    return ok;
}

bool virtual_ft_receiver_ready(VirtualFtReceiver *r)
{
    if (!r || r->fd<0) return false;
    taskENTER_CRITICAL();
    const uint32_t age=(uint32_t)virtual_ft_monotonic_us()-r->state.received_us;
    bool ready=r->state.connected && r->state.enabled && !r->state.fault_latched &&
        age<UINT32_C(0x80000000) && age<=r->limits.timeout_us;
    for (unsigned i=0;i<6;i++) if (r->state.wrench_base_tcp[i]!=0.0) ready=false;
    taskEXIT_CRITICAL();
    return ready;
}
void virtual_ft_receiver_diagnostics(VirtualFtReceiver *r,
    VirtualFtRxDiagnostics *rx,VirtualFtFirstFailure *first)
{
    taskENTER_CRITICAL();
    if (rx) *rx=r->diagnostics;
    if (first) *first=r->first_failure;
    taskEXIT_CRITICAL();
}

void virtual_ft_receiver_close(VirtualFtReceiver *r)
{
    if (r == NULL) return;
    if (r->fd >= 0) close(r->fd);
    r->fd=-1;
}
