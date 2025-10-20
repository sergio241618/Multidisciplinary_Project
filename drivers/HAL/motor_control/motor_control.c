#include "motor_control.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"

/**
 * @brief A helper function to constrain a floating-point value within a specified range.
 * @param val The value to constrain.
 * @param min The minimum allowed value.
 * @param max The maximum allowed value.
 * @return The constrained value.
 */
static float constrain_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max; 
    return val;                
}

/**
 * @brief Initializes the LEDC peripheral to generate a PWM signal for the motor.
 */
void motor_init() {
    // --- Step 1: Configure the LEDC Timer ---
    // The timer is the source of the PWM signal's frequency and resolution.
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE, // Use high-speed mode for better performance.
        .timer_num        = LEDC_TIMER_0,         // Select one of the available hardware timers.
        .duty_resolution  = PWM_RESOLUTION,       // 8 bit
        .freq_hz          = PWM_FREQ,             // 100kHz frequency
        .clk_cfg          = LEDC_AUTO_CLK         // Let the driver automatically select the clock source.
    };
    // Apply the timer configuration.
    ledc_timer_config(&ledc_timer);

    // --- Step 2: Configure the LEDC Channel ---
    // The channel connects the timer to a specific GPIO pin.
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE, // Must match the timer's speed mode.
        .channel        = PWM_CHANNEL,          // 0
        .timer_sel      = LEDC_TIMER_0,         // Link this channel to the timer we just configured.
        .intr_type      = LEDC_INTR_DISABLE,    // We don't need interrupts for this simple PWM setup.
        .gpio_num       = PWM_PIN,              // 13
        .duty           = 0,                    // The initial duty cycle (0 = off).
        .hpoint         = 0                     // Advanced feature for phase shifting, set to 0.
    };
    // Apply the channel configuration.
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief Sets the duty cycle of the PWM signal as a percentage.
 * @param percentage The desired duty cycle (0.0 to 100.0). The value will be automatically
 * clamped between DUTY_CYCLE_MIN and DUTY_CYCLE_MAX.
 * @return The actual percentage that was set after clamping.
 */
float motor_set_duty_cycle(float percentage) {
    // First, clamp the requested percentage to the safe operating range.
    float constrained_percentage = constrain_float(percentage, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

    // Convert the percentage (0-100) into a raw integer value based on the PWM resolution.
    uint32_t dutyValue = (constrained_percentage / 100.0) * ((1 << PWM_RESOLUTION) - 1);

    // Set the new duty cycle value in the hardware register. This prepares the change.
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, dutyValue);

    // Apply the change. This command makes the new duty cycle active on the output pin.
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);

    // Return the actual percentage that was applied after converting.
    return constrained_percentage;
}