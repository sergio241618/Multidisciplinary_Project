/*
 * File: simulink_control.c
 *
 * Code generated for Simulink model 'simulink_control'.
 * Model version                  : 1.19
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sun Oct 19 18:04:12 2025
 *
 * Purpose: This file contains the core logic for the PID controller. It defines
 * the global variables for inputs, outputs, and states, and implements the
 * main 'step' function that calculates the control signal based on the error.
 */

#include "simulink_control.h"
#include "rtwtypes.h"

/* --- Global Variable Definitions --- */

// Defines the actual storage for the block states (or "Data Work" vector).
// essentially the "memory" of the PID controller (e.g., the integrator sum).
DW_simulink_control_T simulink_control_DW;

// Defines the storage for the external inputs to the model.
// This structure is how we pass data INTO the controller (the error signal).
ExtU_simulink_control_T simulink_control_U;

// Defines the storage for the external outputs from the model.
// This structure is how we get the result FROM the controller (the control signal u_k).
ExtY_simulink_control_T simulink_control_Y;

/* MODEL STEP FUNCTION (CORE LOGIC)
/**
 * @brief Executes one step of the discrete-time PID controller.
 * This function should be called at a fixed interval (every 10ms).
 */
void simulink_control_step(void)
{
  // A temporary variable of type 'real_T' (double) for intermediate calculations.
  real_T denAccum;

  /* --- 1. CALCULATE THE DERIVATIVE (D) TERM --- */
  // This block implements a discrete-time derivative with a low-pass filter.
  // The filter helps to reduce the effect of high-frequency noise on the derivative term.
  // 'denAccum' holds the newly calculated state of the derivative filter.
  denAccum = 0.11452 * simulink_control_U.error_signal - -0.009931682274340237 *  // 0.11452 is the Kd entered in the discrete PID block
    simulink_control_DW.FilterDifferentiatorTF_states;

  /* --- 2. CALCULATE THE INTEGRAL (I) TERM --- */
  // This block implements the discrete-time integrator. It accumulates the error over time.
  // The equation is equivalent to: I(k) = I(k-1) + Ki * error(k) * sample_time
  // Here, Ki is 32.962 and the sample time is 10ms.
  simulink_control_DW.Integrator_DSTATE += 32.962 *
    simulink_control_U.error_signal * 0.01;

  /* --- 3. CALCULATE THE FINAL CONTROL OUTPUT (u_k) --- */
  // This is the main PID equation: u_k = Kp * (P_term + I_term + D_term)
  // It combines all three components to generate the final control signal.
  simulink_control_Y.u_k = (
      // --- D Term Output ---
      // This complex expression calculates the final output of the derivative filter block.
      // It uses the new state (denAccum) and the previous state to compute the filtered derivative.
      // 9968.7876673585 is the constant derived from the filter design parameters.
      (denAccum - simulink_control_DW.FilterDifferentiatorTF_states) * 0.009931682274340237 * 9968.7876673585
      
      // --- P and I Term Sum ---
      + (simulink_control_U.error_signal   // This is the Proportional (P) term: the current error.
         + simulink_control_DW.Integrator_DSTATE) // This is the Integral (I) term: the accumulated error sum.
    
    // --- Global Proportional Gain (Kp) ---
    // The entire sum (P + I + D) is multiplied by the overall proportional gain, Kp.
    ) * 0.36931;

  /* --- 4. UPDATE STATE FOR NEXT ITERATION --- */
  // The current state of the derivative filter ('denAccum') is saved into the state variable.
  // This becomes the "previous" state for the calculation in the next time step.
  simulink_control_DW.FilterDifferentiatorTF_states = denAccum;
}


/* === INITIALIZATION AND TERMINATION FUNCTIONS === */
/**
 * @brief Initializes the model's states.
 * This function should be called once at startup or to reset the controller.
 */
void simulink_control_initialize(void)
{
  // In this case, Simulink determined that the default initial values (zero) for the
  // states (Integrator_DSTATE and FilterDifferentiatorTF_states) are sufficient.
}