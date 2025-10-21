#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "motor_control.h"
#include "encoder_reader.h"
#include "trajectory_generator.h"
#include "simulink_control.h"

// --- Tuning and Mode Parameters ---
// A factor to scale down the error before it enters the PID controller.
#define ERROR_SCALING_FACTOR 400.0f

// A switch to enable/disable the motor simulation.
// Set to 1 to use the internal motor simulation (no hardware needed).
// Set to 0 to use the real encoder readings.
#define SIMULATE_ENCODER 0

// The sample time of the main control loop, in milliseconds.
const int TS_MS = 10;

// The GPIO pin connected to the physical reset button (e.g., the 'BOOT' button).
#define RESET_BUTTON_PIN GPIO_NUM_0

/**
 * @brief Configures the GPIO pin for the reset button.
 */
static void configure_reset_button(void) {
    // Configuration structure for the GPIO pin.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,      // We don't need interrupts for this button.
        .mode = GPIO_MODE_INPUT,             // Set the pin as an input.
        .pin_bit_mask = (1ULL << RESET_BUTTON_PIN), // Specify which pin to configure.
        .pull_up_en = 1,                     // Enable the internal pull-up resistor.
        .pull_down_en = 0,                   // Disable the internal pull-down resistor.
    };
    // Apply the configuration.
    gpio_config(&io_conf);
    printf("Reset button configured on GPIO %d\n", RESET_BUTTON_PIN);
}

/**
 * @brief Main application entry point.
 */
void app_main(void) {
    // --- 1. Initialization ---
    // Initialize the PWM hardware for motor control.
    motor_init();

    // Conditionally initialize the encoder hardware only if we are not in simulation mode.
    #if !SIMULATE_ENCODER
    encoder_init();
    #endif

    // Initialize the GPIO for the reset button.
    configure_reset_button();

    // Initialize the states of the Simulink-generated controller model.
    simulink_control_initialize();

    printf("Initializing system with Simulink control...\n");
    #if SIMULATE_ENCODER
    printf("!!! ENCODER SIMULATION MODE ACTIVE !!!\n");
    #endif
    printf("---------------------------------------------------------\n");

    // A simple software counter to keep track of time, more reliable than system timestamps for this loop.
    uint32_t time_counter_ms = 0;
    
    // State variable for the motor simulation, retains its value between loops.
    static float simulated_rpm = 0.0f;

    // --- 2. Main Control Loop ---
    while (1) {
        
        // --- Reset Logic ---
        // Check if the reset button is being pressed (reads low because of the pull-up resistor).
        if (gpio_get_level(RESET_BUTTON_PIN) == 0) {
            printf("--- RESET ---\n");        // Send a reset message to the Python plotter.
            time_counter_ms = 0;             // Reset the software time counter.
            simulink_control_initialize();   // Reset the PID controller's internal states (e.g., the integrator).
            simulated_rpm = 0.0f;            // Reset the simulated motor speed.
            vTaskDelay(pdMS_TO_TICKS(200));  // A small delay to prevent multiple resets from a single long press.
        }

        // --- Control Calculations ---
        // Calculate the current time in seconds from our software counter.
        float t_seconds = time_counter_ms / 1000.0f;
        // Get the target speed for the current time from the trajectory generator.
        float reference_rpm = trajectory_get_reference_rpm(t_seconds);
        
        // This variable will hold the speed measurement (either real or simulated).
        float measured_rpm;
        
        // Use conditional compilation to select the source of the speed measurement.
        #if SIMULATE_ENCODER
            // In simulation mode, use the value from our simple motor model.
            measured_rpm = simulated_rpm;
        #else
            // In real mode, get the speed from the physical encoder.
            measured_rpm = encoder_get_rpm(TS_MS);
        #endif

        // Calculate the error between the target speed and the measured speed.
        float error = reference_rpm - measured_rpm;
        // Scale down the error to make the controller less aggressive.
        float scaled_error = error / ERROR_SCALING_FACTOR;
        // Feed the scaled error into the input structure of the Simulink model.
        simulink_control_U.error_signal = scaled_error;

        // Execute one step of the PID calculation.
        simulink_control_step();
        // Retrieve the calculated control signal (from 0.0 to 1.0) from the model's output structure.
        float u_k = simulink_control_Y.u_k;

        // --- Actuation ---
        // Saturate the control signal to ensure it stays within the valid 0.0-1.0 range.
        if (u_k > 1.0) u_k = 1.0;
        if (u_k < 0.0) u_k = 0.0;
        
        // Map the normalized control signal (0.0-1.0) to the desired PWM duty cycle range (e.g., 10%-90%).
        float duty_cycle_to_set = DUTY_CYCLE_MIN + (u_k * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN));
        // Apply the new duty cycle to the motor.
        motor_set_duty_cycle(duty_cycle_to_set);
        
        // --- Simulation Update ---
        // If in simulation mode, update the simulated RPM for the next loop cycle.
        #if SIMULATE_ENCODER
            // This is a simple first-order discrete model:
            // next_speed = (inertia * current_speed) + (gain * control_signal)
            simulated_rpm = (0.95f * simulated_rpm) + (19.5f * u_k);
        #endif

        // --- Telemetry ---
        // Send the key data points to the serial port for the Python plotter.
        // Format: Reference_RPM,Measured_RPM,Control_Signal_u_k
        printf("%.2f,%.2f,%.2f\n", reference_rpm, measured_rpm, u_k);
        
        // --- Loop Timing ---
        // Increment our software time counter by the sample time.
        time_counter_ms += TS_MS;

        // Wait for the amount of time defined by TS_MS to ensure the loop runs at a fixed frequency.
        vTaskDelay(pdMS_TO_TICKS(TS_MS));
    }
}