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
DW_simulink_control_T simulink_control_DW;
ExtU_simulink_control_T simulink_control_U;
ExtY_simulink_control_T simulink_control_Y;

/* ... (Internal Real-Time Model structure definitions) ... */

/**
 * @brief Executes one step of the discrete-time PID controller.
 * This function should be called at a fixed interval (e.g., every 10ms).
 */
void simulink_control_step(void)
{
  real_T denAccum;

  /* --- 1. CALCULATE THE DERIVATIVE (D) TERM --- */
  // This block implements a discrete-time derivative with a low-pass filter.
  denAccum = 0.11452 * simulink_control_U.error_signal - -0.009931682274340237 *
    simulink_control_DW.FilterDifferentiatorTF_states;

  /* --- 2. CALCULATE THE INTEGRAL (I) TERM --- */
  // This block implements the discrete-time integrator. It accumulates the error over time.
  // Equation: I(k) = I(k-1) + Ki * error(k) * sample_time
  simulink_control_DW.Integrator_DSTATE += 32.962 *
    simulink_control_U.error_signal * 0.01;

  /* --- 3. CALCULATE THE FINAL CONTROL OUTPUT (u_k) --- */
  // Main PID equation: u_k = Kp * (P_term + I_term + D_term)
  simulink_control_Y.u_k = (
      // --- D Term Output ---
      (denAccum - simulink_control_DW.FilterDifferentiatorTF_states) * 0.009931682274340237 * 9968.7876673585
      
      // --- P and I Term Sum ---
      + (simulink_control_U.error_signal           // Proportional (P) term
         + simulink_control_DW.Integrator_DSTATE)  // Integral (I) term
    
    // --- Global Proportional Gain (Kp) ---
    ) * 0.36931;

  /* --- 4. UPDATE STATE FOR NEXT ITERATION --- */
  // The current state of the derivative filter becomes the "previous" state for the next step.
  simulink_control_DW.FilterDifferentiatorTF_states = denAccum;
}

/**
 * @brief Initializes the model's states.
 * Call this function once at startup or to reset the controller.
 */
void simulink_control_initialize(void)
{
  // No explicit initialization code is required here, as states default to zero.
}