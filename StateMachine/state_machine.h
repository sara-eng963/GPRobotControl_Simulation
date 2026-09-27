#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H


#include "state_machine_types.h"

#include "States/state_boot.h"
#include "States/state_homing.h"
#include "States/state_idle.h"
#include "States/state_teaching.h"
#include "States/state_path_validation.h"
#include "States/state_approach.h"

#include "../ControlCore/Config/robot_config.h"
#include "../EtherCATComm/ethercat_types.h"

#include <stdint.h>


/* ============================================================================
 * GLOBAL STATE-MACHINE DEPENDENCIES
 * ============================================================================
 *
 * These objects are owned elsewhere.
 *
 * The FSM coordinates them; it does not duplicate their data.
 */

typedef struct
{
    const EtherCATMasterConfig *ethercat_config;

    const RobotConfig *robot;

    const HomingConfig *homing_config;

    const TeachingConfig *teaching_config;

    const PathValidationConfig *path_validation_config;

    const PathValidationServices *path_validation_services;

    PathValidationWorkspace *path_validation_workspace;

    const PathValidationStorage *path_validation_storage;

    ValidatedTrajectory *validated_trajectory;

} StateMachineDependencies;


/* ============================================================================
 * RUNTIME INPUTS
 * ============================================================================
 *
 * Commands/events that may change every supervisory cycle.
 */

typedef struct
{
    IdleCommand idle_command;

    TeachingEvent teaching_event;

    TeachingRuntimeInputs teaching_runtime;

    /*
     * Maximum number of trajectory samples Path Validation may process
     * during one call.
     */
    uint16_t path_validation_sample_budget;

} StateMachineInputs;


/* ============================================================================
 * GLOBAL STATE MACHINE
 * ============================================================================ */

typedef struct
{
    RobotState current_state;
    RobotState previous_state;

    BootState boot;

    HomingState homing;

    IdleState idle;

    TeachingState teaching;

    PathValidationState path_validation;


    /*
     * Latest outputs that higher-level code / HMI may inspect.
     */
    TeachingOutputs teaching_outputs;

    PathValidationOutputs path_validation_outputs;


    /*
     * Program ID used when entering a new Teaching session.
     */
    uint32_t teaching_program_id;


    bool initialized;

} StateMachine;


/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

bool state_machine_init(
    StateMachine *machine,
    uint32_t teaching_program_id
);


StateStepResult state_machine_step(
    StateMachine *machine,
    const StateMachineDependencies *dependencies,
    const StateMachineInputs *inputs
);


RobotState state_machine_current_state(
    const StateMachine *machine
);


const char *state_machine_state_name(
    RobotState state
);


#endif /* STATE_MACHINE_H */