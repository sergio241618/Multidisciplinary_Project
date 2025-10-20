#ifndef MOTOR_CONTROL_H //header guard
#define MOTOR_CONTROL_H

// --- PWM Configuration ---
#define PWM_PIN           13
#define PWM_CHANNEL       0
#define PWM_FREQ          100000 //kHz
#define PWM_RESOLUTION    8 //bits  
#define DUTY_CYCLE_MIN    10.0f
#define DUTY_CYCLE_MAX    90.0f

/**
 * @brief Initializes the motor control PWM settings.
 */
void motor_init();

/**
 * @brief Establishes the PWM signal for motor control.
 * @param percentage The desired duty cycle percentage (0.0 to 100.0) is constrained between DUTY_CYCLE_MIN and DUTY_CYCLE_MAX.
 * @return float The actual duty cycle that was set.
 */
float motor_set_duty_cycle(float percentage);

#endif //header guard