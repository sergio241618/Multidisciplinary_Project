/*
 * File: simulink_control.h
 *
 * Code generated for Simulink model 'simulink_control'.
 * Model version                  : 1.4
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sun Oct 19 10:54:02 2025
 *
 * Purpose: This file is the public interface for the generated PID controller.
 * It declares the data structures for inputs, outputs, and internal states,
 * as well as the main functions needed to initialize and run the controller step.
 */

#ifndef simulink_control_h_ // header guard
#define simulink_control_h_

#ifndef simulink_control_COMMON_INCLUDES_ //inner guard for common includes
#define simulink_control_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif    // inner guard for common includes

#include "simulink_control_types.h"

/* --- BLOCK STATES (INTERNAL MEMORY) DATA STRUCTURE --- */
typedef struct {
  // The state variable for the derivative term's filter. It needs to remember
  // its previous value to calculate the next one.
  real_T FilterDifferentiatorTF_states;

  // The state variable for the integral term. This is the most important state,
  // as it accumulates the sum of the error over time.
  real_T Integrator_DSTATE;
} DW_simulink_control_T;

/* --- EXTERNAL INPUTS DATA STRUCTURE --- */
typedef struct {
  // The input for the error signal.
  real_T error_signal;
} ExtU_simulink_control_T;

/* --- EXTERNAL OUTPUTS DATA STRUCTURE --- */
typedef struct {
  // The output for the control signal.
  real_T u_k;
} ExtY_simulink_control_T;

/* --- GLOBAL VARIABLE DECLARATIONS --- */
// The 'extern' keyword tells other files (like main.c) that these global variables
// exist and can be used. Their actual definition and memory allocation are in simulink_control.c.

// Declaration of the global structure for the controller's states.
extern DW_simulink_control_T simulink_control_DW;

// Declaration of the global structure for the controller's inputs.
extern ExtU_simulink_control_T simulink_control_U;

// Declaration of the global structure for the controller's outputs.
extern ExtY_simulink_control_T simulink_control_Y;

/* --- MODEL ENTRY-POINT FUNCTION DECLARATIONS --- */
/**
 * @brief Initializes the controller's states to their starting values (usually zero).
 */
extern void simulink_control_initialize(void);

/**
 * @brief Executes one computational step of the PID controller.
 * Call this function repeatedly at your fixed sample rate (10ms).
 */
extern void simulink_control_step(void);

/**
 * @brief Terminates the model execution (optional cleanup).
 */
extern void simulink_control_terminate(void);

#endif  // header guard