/*
 * ============================================================================
 * REAL ROBOT ENTRY POINT - RESERVED
 * ============================================================================
 *
 * This file is intentionally reserved for the real robot application that
 * will run on the final controller (STM32 + FreeRTOS + real hardware).
 *
 * The PC SIL Kit simulator entry point is:
 *
 *      Simulation/supervisor_silkit_main.c
 *
 * The simulator and the future real main must use the same shared modules:
 *
 *      StateMachine/state_machine.c
 *      StateMachine/States/*
 *      ControlCore/*
 *      CANComm/CANopen/*
 *      ServoDrive/*
 *
 * Simulator-only code (SIL Kit participants, desktop teaching window,
 * host UDP plumbing, MATLAB telemetry, RAM-backed validation storage) must not
 * be copied into this file as real-robot logic.
 *
 * The real main.c will be implemented by the teammate responsible for the
 * target controller/runtime integration.
 * ============================================================================
 */
