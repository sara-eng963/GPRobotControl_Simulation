#include "hmi_task.h"
static HmiTaskConfig config;
static QueueHandle_t queue;
static TaskHandle_t task;
static SupervisorHmiCommand translate(HmiEvent event)
{
    switch(event) {
    case HMI_EVENT_SELECT_LINE:return SUP_HMI_LINE;
    case HMI_EVENT_SELECT_ARC:return SUP_HMI_ARC;
    case HMI_EVENT_SELECT_CIRCLE:return SUP_HMI_CIRCLE;
    case HMI_EVENT_RECORD:return SUP_HMI_RECORD;
    case HMI_EVENT_VALIDATE_PREVIEW:return SUP_HMI_VALIDATE_PREVIEW;
    case HMI_EVENT_SPEED_INCREASE:return SUP_HMI_SPEED_UP;
    case HMI_EVENT_SPEED_DECREASE:return SUP_HMI_SPEED_DOWN;
    case HMI_EVENT_SPEED_DEFAULT:return SUP_HMI_SPEED_DEFAULT;
    case HMI_EVENT_START:return SUP_HMI_START_REPLAY;
    case HMI_EVENT_PAUSE:return SUP_HMI_PAUSE;
    case HMI_EVENT_RESUME:return SUP_HMI_RESUME;
    case HMI_EVENT_RESET:return SUP_HMI_RESET;
    case HMI_EVENT_HOME:return SUP_HMI_HOME;
    default:return SUP_HMI_COMMAND_COUNT;
    }
}
static void entry(void *arg)
{
    (void)arg;
    HmiEvent pending=HMI_EVENT_NONE;
    TickType_t wake=xTaskGetTickCount();
    for(;;) {
        /* Retain an event when Supervisor's queue is full; never drop Record. */
        if(pending==HMI_EVENT_NONE) {
            if(xQueueReceive(queue,&pending,0)!=pdPASS && config.receive_event)
                (void)config.receive_event(config.context,&pending);
        }
        if(pending!=HMI_EVENT_NONE) {
            SupervisorHmiCommand command=translate(pending);
            if(command==SUP_HMI_COMMAND_COUNT || supervisor_task_post_hmi(command,0))
                pending=HMI_EVENT_NONE;
        }
        SupervisorDiagnostics status;
        if(config.publish_status && supervisor_task_get_diagnostics(&status))
            config.publish_status(config.context,&status);
        vTaskDelayUntil(&wake,config.period_ticks);
    }
}
bool hmi_task_init(const HmiTaskConfig *c)
{
    if(!c || !c->period_ticks || queue || task)return false;
    config=*c;queue=xQueueCreate(32,sizeof(HmiEvent));return queue!=NULL;
}
bool hmi_task_start(UBaseType_t priority)
{
    return queue && !task && xTaskCreate(entry,"HMI",1024,NULL,priority,&task)==pdPASS;
}
bool hmi_task_submit(HmiEvent event,TickType_t wait)
{
    return queue && hmi_event_is_valid(event) && xQueueSend(queue,&event,wait)==pdPASS;
}
