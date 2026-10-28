/* Tests ACTUAL CM4 application_ipc_service.c. Only FreeRTOS, scheduling,
 * the shared mailbox and peripheral-side status arrival are mocked. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "application_ipc_service.c"

typedef struct { unsigned count; size_t size; uint8_t data[128]; } TestQueue;
static TestQueue command_q={0, sizeof(SupervisorHmiCommand),{0}};
static TestQueue status_q={0,sizeof(StatusSnapshot),{0}};
static IpcShared ipc_region;
static IpcMessage to_cm4[32], from_cm4[32];
static unsigned rpos,wpos, sent, blocked_sends, critical_depth;
static bool block_send;
static TickType_t fake_tick;
static jmp_buf jump;
void p03_enter_critical(void) { ++critical_depth; }
void p03_exit_critical(void) { assert(critical_depth); --critical_depth; }
TickType_t xTaskGetTickCount(void) { return fake_tick; }
void vTaskDelayUntil(TickType_t *previous,TickType_t period)
{ (void)previous; fake_tick+=period; longjmp(jump,1); }
void vTaskDelay(TickType_t p) { fake_tick+=p; }
TaskHandle_t xTaskCreateStatic(void (*fn)(void *),const char *n,unsigned stack,
 void *arg,UBaseType_t priority,StackType_t *buf,StaticTask_t *ctl)
{ (void)fn;(void)n;(void)stack;(void)arg;(void)priority;(void)buf;(void)ctl; return &ipc_region; }
QueueHandle_t xQueueCreateStatic(UBaseType_t n,UBaseType_t size,uint8_t *store,StaticQueue_t *ctrl)
{ (void)n;(void)size;(void)store;(void)ctrl; return NULL; }
BaseType_t xQueuePeek(QueueHandle_t q,void *out,TickType_t wait)
{ (void)wait; TestQueue *t=q; if(!t->count) return pdFAIL; memcpy(out,t->data,t->size); return pdPASS; }
BaseType_t xQueueOverwrite(QueueHandle_t q,const void *in)
{ TestQueue *t=q; memcpy(t->data,in,t->size); t->count=1; return pdPASS; }
BaseType_t xQueueSend(QueueHandle_t q,const void *in,TickType_t wait)
{ (void)wait; TestQueue *t=q; if(t->count) return pdFAIL; memcpy(t->data,in,t->size); t->count=1; return pdPASS; }
BaseType_t xQueueReceive(QueueHandle_t q,void *out,TickType_t wait)
{ (void)wait; TestQueue *t=q; if(!t->count) return pdFAIL; memcpy(out,t->data,t->size); t->count=0; return pdPASS; }
BaseType_t xQueueReset(QueueHandle_t q)
{ ((TestQueue*)q)->count=0; return pdPASS; }
bool ipc_valid(const IpcShared *sh) { return sh!=NULL; }
bool ipc_m4_receive(IpcShared *sh,IpcMessage *out)
{ assert(sh==&ipc_region); if(rpos==wpos) return false; *out=to_cm4[rpos++]; return true; }
bool ipc_m4_send(IpcShared *sh,const IpcMessage *msg)
{ assert(sh==&ipc_region); if(block_send){++blocked_sends;return false;}
  assert(sent<32); from_cm4[sent++]=*msg; return true; }
static void deliver_status(unsigned seq,unsigned session,unsigned challenge,unsigned confirmed)
{ IpcMessage status={0}; status.type=IPC_CONTROLLER_STATUS;
  status.sequence=seq; status.data[0]=ROBOT_STATE_IDLE;
  status.data[7]=session; status.data[8]=RUNTIME_PEER_PROTOCOL_VERSION;
  status.data[9]=challenge; status.data[10]=confirmed;
  assert(wpos<32); to_cm4[wpos++]=status; }
static void one_cycle(void)
{ if (setjmp(jump)==0) application_ipc_service_run(); assert(critical_depth==0); }
int main(void)
{
 shared_ipc=&ipc_region;
 command_queue=(QueueHandle_t)&command_q;
 status_queue=(QueueHandle_t)&status_q;
 deliver_status(1,71,0,0);
 one_cycle();
 assert(sent==1 && from_cm4[0].type==IPC_CM4_HELLO && from_cm4[0].data[0]==71);
 deliver_status(2,71,1,0);
 one_cycle();
 assert(sent==2 && from_cm4[1].type==IPC_CM4_PROGRESS);
 assert(from_cm4[1].data[6]==1 && from_cm4[1].data[0]==71);
 assert(from_cm4[1].sequence>=2);
 one_cycle(); assert(sent==2); /* No free-running progress flood. */
 deliver_status(3,71,0,1);
 one_cycle();
 assert(application_ipc_service_submit_hmi(SUP_HMI_HOME));
 one_cycle();
 assert(sent==3 && from_cm4[2].type==IPC_HMI_COMMAND);
 assert(from_cm4[2].data[1]==71 && from_cm4[2].data[2]==1);
 /* An M7 reboot invalidates prior queued HMI and forces another HELLO. */
 deliver_status(1,72,0,0); block_send=true;
 one_cycle(); assert(sent==3 && blocked_sends>0 && !hello_sent);
 block_send=false; one_cycle();
 assert(sent==4 && from_cm4[3].type==IPC_CM4_HELLO && from_cm4[3].data[0]==72);
 deliver_status(2,72,1,0); one_cycle();
 assert(sent==5 && from_cm4[4].type==IPC_CM4_PROGRESS && from_cm4[4].data[0]==72);
 /* Simulate CM4 RAM reinitialization while current CM7 session stays 72.
  * First CM4 app message after reset MUST be HELLO, regardless of any
  * subsequent progress sequence (even if it advances past its old value). */
 hello_sent=false; current_m7_session=0; last_status_sequence=0;
 pending_challenge=last_seen_challenge=last_sent_challenge=confirmed_challenge=0;
 deliver_status(3,72,2,0); one_cycle();
 assert(sent==6 && from_cm4[5].type==IPC_CM4_HELLO);
 puts("PASS: actual CM4 service HELLO, challenge completion, HMI tagging, full ring and reset ordering");
 return 0;
}
