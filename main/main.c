#include <stdio.h>
#include <math.h> // Para pow()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "motor_control.h"
#include "encoder_reader.h"
#include "trajectory_generator.h"
#include "simulink_control.h"
#include "PID_Difuso.h"

// ===================================================================
// ===== CONTROLLER SELECTION ========================================
#define USE_FUZZY_PID 0 // 0 = Conventional PID, 1 = Fuzzy PID
// ===================================================================

#define SIMULATE_ENCODER 0 // 1 = Simulate, 0 = Real Encoder

const int TS_MS = 10;
#define RESET_BUTTON_PIN GPIO_NUM_0

// --- Global variables for MSE calculation ---
static double sum_squared_error = 0.0;
static long sample_count = 0;

// --- Configure Reset Button (no changes) ---
static void configure_reset_button(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << RESET_BUTTON_PIN),
        .pull_up_en = 1,
        .pull_down_en = 0,
    };
    gpio_config(&io_conf);
    printf("Reset button configured on GPIO %d\n", RESET_BUTTON_PIN);
}

void app_main(void) {
    // --- Initializations (no changes) ---
    motor_init();
    #if !SIMULATE_ENCODER
    encoder_init();
    #endif
    configure_reset_button();
    simulink_control_initialize();
    PID_Difuso_initialize();

    #if USE_FUZZY_PID
    printf("Initializing system with FUZZY PID control...\n");
    #else
    printf("Initializing system with Conventional PID control...\n");
    #endif
    #if SIMULATE_ENCODER
    printf("!!! ENCODER SIMULATION MODE ACTIVE !!!\n");
    #endif
    printf("---------------------------------------------------------\n");

    uint32_t time_counter_ms = 0;
    static float simulated_rpm = 0.0f;
    sum_squared_error = 0.0;
    sample_count = 0;

    while (1) {
        // --- Reset Logic and MSE Calculation ---
        if (gpio_get_level(RESET_BUTTON_PIN) == 0) {
            // --- CAMBIO AQUÍ: Calcular y ENVIAR MSE por UART ---
            if (sample_count > 0) {
                double mse = sum_squared_error / sample_count;
                // Send MSE result in a specific format for Python to catch
                printf("MSE_RESULT:%.4f\n", mse); // <--- MENSAJE ESPECIAL
            }
            // --- FIN DEL CAMBIO ---

            printf("--- RESET ---\n"); // Send reset signal to Python
            time_counter_ms = 0;
            simulink_control_initialize();
            PID_Difuso_initialize();
            simulated_rpm = 0.0f;
            sum_squared_error = 0.0;
            sample_count = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // --- Control Loop Logic (no changes below this point) ---
        float t_seconds = time_counter_ms / 1000.0f;
        float reference_rpm = trajectory_get_reference_rpm(t_seconds);

        float measured_rpm;
        #if SIMULATE_ENCODER
            measured_rpm = simulated_rpm;
        #else
            measured_rpm = encoder_get_rpm(TS_MS);
        #endif

        float error = reference_rpm - measured_rpm;

        // --- Accumulate error for MSE ---
        if (t_seconds <= 40.0f) { // Adjust duration if needed
             sum_squared_error += pow(error, 2);
             sample_count++;
        }

        float u_k = 0.0f;
        #if USE_FUZZY_PID
            PID_Difuso_U.error_signal = error;
            PID_Difuso_step();
            float u_fuzzy_pi = PID_Difuso_Y.out;
            u_k = u_fuzzy_pi / 60.0f;
        #else
            simulink_control_U.error_signal = error;
            simulink_control_step();
            u_k = simulink_control_Y.u_k;
        #endif

        if (u_k > 1.0) u_k = 1.0;
        if (u_k < 0.0) u_k = 0.0;

        float duty_cycle_to_set = DUTY_CYCLE_MIN + (u_k * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN));
        motor_set_duty_cycle(duty_cycle_to_set);

        #if SIMULATE_ENCODER
            simulated_rpm = (0.95f * simulated_rpm) + (25.0f * u_k);
        #endif

        // Send telemetry data
        printf("%.2f,%.2f,%.2f\n", reference_rpm, measured_rpm, u_k);

        time_counter_ms += TS_MS;
        vTaskDelay(pdMS_TO_TICKS(TS_MS));
    }
}