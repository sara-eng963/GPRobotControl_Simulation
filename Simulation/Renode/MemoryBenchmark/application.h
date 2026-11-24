#ifndef MEMBENCH_APPLICATION_H
#define MEMBENCH_APPLICATION_H
/* Isolated ARM test composition. No physical provider or authorization. */
#include "../../../Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.h"
#include "../../../Firmware/STM32H745/CM7/Application/Intercore/trajectory_stream_path_port.h"
#include "../../../Firmware/STM32H745/CM7/Core/Inc/application_ft_service.h"
#include "../../../Firmware/STM32H745/CM4/Core/Inc/application_ipc_service.h"
#include "../../../Firmware/STM32H745/CM4/Core/Inc/application_validation_service.h"
#include "../../../HandGuiding/supervisor_dry_run_adapter.h"
#include "../../../Firmware/STM32H745/Common/Inc/sram4_memory_map.h"

enum {
    MB_MAGIC, MB_VERSION, MB_STATUS, MB_FAILURE_LINE, MB_PHASE, MB_COMPLETED,
    MB_ASSERTIONS, MB_SUPERVISOR_CYCLES, MB_CAN_TARGETS, MB_CAN_SYNCS,
    MB_FT_FRAMES, MB_GUIDANCE_OK, MB_VALIDATIONS, MB_VALIDATED_SAMPLES,
    MB_PLAYBACKS, MB_PLAYBACK_SAMPLES, MB_MAX_VALIDATION_QUEUE,
    MB_MAX_TRAJECTORY_RING, MB_MAX_FT_RING, MB_MAX_PROGRAM_SEGMENTS,
    MB_MAX_SUPERVISOR_QUEUE, MB_TRANSITIONS, MB_SAFE_OUTPUT_CALLS,
    MB_STORAGE_WRITES, MB_STORAGE_READS, MB_STORAGE_COMMITS, MB_LAST_STATE,
    MB_LAST_BOOT_PHASE, MB_LAST_FAULT, MB_LAST_VALIDATION_RESULT,
    MB_LAST_VALIDATION_ERROR, MB_MAX_RAW_GEOMETRY, MB_FT_FAULT_VERIFIED,
    MB_RESTART_DENIED, MB_PHASE_TICK, MB_LAST_EXECUTION_ERROR,
    MB_LAST_EXECUTION_PHASE, MB_PEER_PROGRESS, MB_C_HEAP_GROWTH,
    MB_FAILURE_CALLER, MB_WAITING_STATE, MB_EXPECTED_PLAYBACK_SAMPLES,
    MB_SKIPPED_PLAYBACK_SAMPLES,
    MB_STORAGE_READ_FAULT_INJECTED,
    MB_HEAP_AFTER_FIRST, MB_HEAP_AFTER_SECOND, MB_HEAP_AT_START, MB_TASK_COUNT,
    MB_WORDS = 48
};
enum {
    MB_STARTUP=1, MB_IDLE, MB_TEACHING, MB_VALIDATION, MB_PLAYBACK,
    MB_TRANSITIONS_PHASE, MB_CAPACITY, MB_FAULTS
};
extern volatile uint32_t membench_application[MB_WORDS];
extern SupervisorTaskConfig mb_config;
extern ValidatedTrajectory mb_artifact;
extern HandGuidingDryRunState mb_guidance;
extern volatile bool mb_operator_pressed;
extern volatile bool mb_ft_stopped;
extern volatile bool mb_estop;
void membench_application_start(void);
void mb_providers_initialize(void);
void mb_set_manual_pose(unsigned endpoint);
bool mb_cache_first_sample(void);
void mb_ft_capacity(void);
void mb_fail(uint32_t line);
void mb_scenario(void *unused);
#define MB_ASSERT(x) do { __atomic_fetch_add(&membench_application[MB_ASSERTIONS],1,__ATOMIC_RELAXED); if (!(x)) mb_fail(__LINE__); } while (0)
#endif
