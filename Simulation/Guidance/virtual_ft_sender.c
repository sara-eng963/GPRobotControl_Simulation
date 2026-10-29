#define _POSIX_C_SOURCE 200809L
#include "virtual_ft_sender.h"
#include <errno.h>
#include <string.h>
#include <time.h>

static uint64_t mono_us(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC,&t)!=0) return 0;
    return (uint64_t)t.tv_sec*1000000U+(uint64_t)t.tv_nsec/1000U;
}
static bool nonzero(const double w[6])
{
    for (unsigned i=0;i<6;i++) if (w[i]!=0.0) return true;
    return false;
}
static void *send_loop(void *arg)
{
    VirtualFtSender *s=arg;
    uint64_t next=mono_us();
    for (;;) {
        double w[6];
        const uint64_t now=mono_us();
        pthread_mutex_lock(&s->lock);
        const bool stop=s->stop;
        memcpy(w,s->command,sizeof(w));
        /* A zero request can be repeated without inventing a new force.
         * A nonzero request is leased to a responsive input loop. Expiry
         * explicitly DISABLES input on the wire, causing the normal latched
         * fault if guidance is active. Never disguise expiry as healthy zero. */
        if (nonzero(w) && (now<s->updated_us ||
            now-s->updated_us>VIRTUAL_FT_COMMAND_LEASE_US))
            s->status.command_expired=true;
        const bool enabled=!s->status.command_expired;
        pthread_mutex_unlock(&s->lock);
        if (stop) break;
        if (!enabled) memset(w,0,sizeof(w));
        const bool ok=now!=0 && virtual_ft_client_send_enabled(&s->client,enabled,w);
        const uint64_t finished=mono_us();
        pthread_mutex_lock(&s->lock);
        if (!ok) s->status.failed=true;
        else {
            const uint64_t gap=s->status.last_send_us ? finished-s->status.last_send_us : 0;
            if (gap>s->status.max_send_gap_us) s->status.max_send_gap_us=gap;
            s->status.last_send_us=finished;
            s->status.last_sequence=s->client.next_sequence-1U;
            ++s->status.sent;
            memcpy(s->status.last_wrench,w,sizeof(w));
        }
        next+=VIRTUAL_FT_SEND_PERIOD_US;
        if (next<=finished) {
            ++s->status.missed_releases;
            next=finished+VIRTUAL_FT_SEND_PERIOD_US; /* no catch-up burst */
        }
        pthread_mutex_unlock(&s->lock);
        if (!ok) break; /* Receiver watchdog detects transmitter failure. */
        const struct timespec deadline={(time_t)(next/1000000U),(long)(next%1000000U)*1000L};
        int rc;
        do { rc=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&deadline,NULL); } while(rc==EINTR);
        if (rc!=0) {
            pthread_mutex_lock(&s->lock);s->status.failed=true;pthread_mutex_unlock(&s->lock);
            break;
        }
    }
    /* The sequence/socket owner also performs the final zero publication. */
    virtual_ft_client_close(&s->client);
    return NULL;
}
bool virtual_ft_sender_start(VirtualFtSender *s)
{
    if (!s) return false;
    memset(s,0,sizeof(*s));
    if (pthread_mutex_init(&s->lock,NULL)!=0) return false;
    if (!virtual_ft_client_open(&s->client)) {pthread_mutex_destroy(&s->lock);return false;}
    s->updated_us=mono_us();
    if (!s->updated_us || pthread_create(&s->thread,NULL,send_loop,s)!=0) {
        virtual_ft_client_close(&s->client);pthread_mutex_destroy(&s->lock);return false;
    }
    s->running=true;
    return true;
}
void virtual_ft_sender_update(VirtualFtSender *s,const double w[6])
{
    if (!s || !s->running || !w) return;
    pthread_mutex_lock(&s->lock);
    memcpy(s->command,w,sizeof(s->command));s->updated_us=mono_us();
    /* GUI must inhibit held keys on expiry, then publish a released zero. */
    if (!nonzero(w)) s->status.command_expired=false;
    pthread_mutex_unlock(&s->lock);
}
void virtual_ft_sender_status(VirtualFtSender *s,VirtualFtSenderStatus *out)
{
    pthread_mutex_lock(&s->lock);*out=s->status;pthread_mutex_unlock(&s->lock);
}
void virtual_ft_sender_stop(VirtualFtSender *s)
{
    if (!s || !s->running) return;
    pthread_mutex_lock(&s->lock);s->stop=true;pthread_mutex_unlock(&s->lock);
    pthread_join(s->thread,NULL);s->running=false;
    pthread_mutex_destroy(&s->lock);
}
