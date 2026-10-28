/* Production policy header, deterministic same-core clocks, no fake firmware. */
#include <assert.h>
#include <stdio.h>
#include "runtime_evidence.h"
typedef struct { bool ok; uint32_t session, sequence; } Source;
static bool verify(void *ctx, const RobotSafetySnapshot *s, uint32_t *id, uint32_t *seq)
{ Source *v=ctx; (void)s; if (!v->ok) return false; *id=v->session; *seq=v->sequence; return true; }
static bool issue(void *ctx,uint32_t *s) { *s=*(uint32_t*)ctx; return true; }
static IpcMessage hello(uint32_t session)
{ IpcMessage m={0}; m.type=IPC_CM4_HELLO; m.sequence=1; m.data[0]=session;
  m.data[1]=RUNTIME_PEER_PROTOCOL_VERSION; m.data[2]=1; return m; }
static IpcMessage response(uint32_t seq, uint32_t session, uint32_t challenge)
{ IpcMessage m={0}; m.type=IPC_CM4_PROGRESS; m.sequence=seq;
  m.timestamp_ms=UINT32_MAX; m.data[0]=session; m.data[1]=RUNTIME_PEER_PROTOCOL_VERSION;
  m.data[2]=3; m.data[3]=1; m.data[4]=1; m.data[5]=1;
  m.data[6]=challenge; return m; }
int main(void)
{
 uint32_t boot_session=17, issued=0;
 RuntimeIpcSessionPort issuer={issue,&boot_session};
 assert(!runtime_session_issue(NULL,&issued) && issued==0);
 assert(runtime_session_issue(&issuer,&issued) && issued==17);
 boot_session=0; assert(!runtime_session_issue(&issuer,&issued));
 Source v={true,7,1}; RuntimeSafetyPort port={verify,&v,10};
 RobotSafetySnapshot s={.timestampMs=100,.statusValid=true,.motionPermitted=true,.communicationHealthy=true};
 RuntimeSafetyEvidence a={0};
 assert(!runtime_safety_fresh(&a,&port,100));
 assert(!runtime_safety_accept(&a,NULL,&s,100));
 assert(runtime_safety_accept(&a,&port,&s,100));
 assert(runtime_safety_fresh(&a,&port,110));
 assert(!runtime_safety_fresh(&a,&port,111));
 assert(!runtime_safety_accept(&a,&port,&s,105));
 assert(!runtime_safety_fresh(&a,&port,111));
 ++v.sequence; s.timestampMs=113;
 assert(!runtime_safety_accept(&a,&port,&s,112));
 s.timestampMs=99; assert(!runtime_safety_accept(&a,&port,&s,112));
 s.timestampMs=111; assert(runtime_safety_accept(&a,&port,&s,112));
 ++v.session; ++v.sequence; s.timestampMs=112;
 assert(!runtime_safety_accept(&a,&port,&s,112));
 v.ok=false; assert(!runtime_safety_accept(&a,&port,&s,113));
 assert(!runtime_safety_fresh(&a,&port,123));
 v.ok=true; v.session=7; ++v.sequence;
 s.timestampMs=113; s.estopActive=true;
 assert(!runtime_safety_accept(&a,&port,&s,114)); /* Contradictory safety. */
 s.estopActive=false; s.timestampMs=114; s.statusValid=false;
 assert(!runtime_safety_accept(&a,&port,&s,114));
 RuntimeSafetyEvidence w={0}; Source ws={true,5,UINT32_MAX};
 RuntimeSafetyPort wp={verify,&ws,10};
 RobotSafetySnapshot x={.timestampMs=UINT32_MAX-2U,.statusValid=true};
 assert(runtime_safety_accept(&w,&wp,&x,UINT32_MAX-1U));
 ws.sequence=0; x.timestampMs=1;
 assert(runtime_safety_accept(&w,&wp,&x,2));
 ws.sequence=1; x.timestampMs=2;
 assert(runtime_safety_accept(&w,&wp,&x,3));
 assert(runtime_safety_fresh(&w,&wp,3));

 RuntimePeerEvidence p={.expected_session=99};
 IpcMessage h=hello(99); uint32_t challenge=0;
 assert(!runtime_peer_fresh(&p,10));
 assert(!runtime_peer_challenge(&p,10,&challenge));
 assert(runtime_peer_on_hello(&p,&h));
 assert(runtime_peer_challenge(&p,10,&challenge) && challenge==1);
 IpcMessage r=response(10,99,challenge);
 assert(!runtime_peer_accept(&p,&r,41)); /* Delayed beyond issue + RTT. */
 assert(!runtime_peer_fresh(&p,41));
 assert(runtime_peer_challenge(&p,42,&challenge) && challenge==2);
 assert(!runtime_peer_accept(&p,&r,43)); /* Buffered older challenge. */
 r=response(11,99,challenge);
 assert(runtime_peer_accept(&p,&r,43));
 assert(runtime_peer_fresh(&p,43));
 assert(!runtime_peer_accept(&p,&r,44)); /* Replayed completed sequence. */
 assert(runtime_peer_challenge(&p,49,&challenge) && challenge==3);
 assert(runtime_peer_challenge(&p,50,&challenge) && challenge==3);
 r=response(11,99,challenge);
 assert(!runtime_peer_accept(&p,&r,50)); /* Forward challenge, REPLAY sequence. */
 r=response(12,99,challenge);
 assert(runtime_peer_accept(&p,&r,50));
 assert(runtime_peer_fresh(&p,50));
 assert(!runtime_peer_fresh(&p,101)); /* CM4 stall, no further work. */
 assert(runtime_peer_challenge(&p,102,&challenge) && challenge==4);
 r=response(13,99,challenge); r.data[0]=98;
 assert(!runtime_peer_accept(&p,&r,103)); /* Wrong controller session. */
 r.data[0]=99; r.data[1]=1;
 assert(!runtime_peer_accept(&p,&r,103));
 r.data[1]=RUNTIME_PEER_PROTOCOL_VERSION; r.data[3]=0;
 assert(!runtime_peer_accept(&p,&r,103));
 r.data[3]=1;
 assert(runtime_peer_accept(&p,&r,103));
 assert(!runtime_peer_on_hello(&p,&h)); /* Fast CM4 reboot BEFORE timeout. */
 assert(p.compromised && !runtime_peer_fresh(&p,103));
 assert(!runtime_peer_challenge(&p,104,&challenge));

 RuntimePeerEvidence np={.expected_session=100};
 assert(!runtime_peer_on_hello(&np,&h));
 h=hello(100); assert(runtime_peer_on_hello(&np,&h));
 assert(runtime_peer_challenge(&np,100,&challenge));
 assert(!runtime_peer_accept(&np,&r,101)); /* Old CM7 session. */
 r=response(1,100,challenge); assert(runtime_peer_accept(&np,&r,101));
 RuntimePeerEvidence pw={.expected_session=3};
 h=hello(3); assert(runtime_peer_on_hello(&pw,&h));
 assert(runtime_peer_challenge(&pw,UINT32_MAX-3U,&challenge));
 r=response(UINT32_MAX,3,challenge);
 assert(runtime_peer_accept(&pw,&r,UINT32_MAX-2U));
 assert(runtime_peer_challenge(&pw,UINT32_MAX-1U,&challenge));
 r=response(0,3,challenge);
 assert(runtime_peer_accept(&pw,&r,1));
 assert(runtime_peer_challenge(&pw,2,&challenge));
 r=response(1,3,challenge); assert(runtime_peer_accept(&pw,&r,3));
 assert(runtime_peer_fresh(&pw,3));
 puts("PASS: P03 challenge, HELLO, delay, replay, wrap, expiry and safety policy");
 return 0;
}
