#ifndef STATE_MACHINE_TYPES_H
#define STATE_MACHINE_TYPES_H


/*
 * Result returned whenever one robot state
 * executes one step of its logic.
 */
typedef enum
{
    STATE_STEP_RUNNING = 0,
    STATE_STEP_COMPLETE,
    STATE_STEP_FAILED

} StateStepResult;


#endif /* STATE_MACHINE_TYPES_H */