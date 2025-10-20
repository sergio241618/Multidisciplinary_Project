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

#define ERROR_SCALING_FACTOR 400.0f
#define SIMULATE_ENCODER 0

const int TS_MS = 10;
#define RESET_BUTTON_PIN GPIO_NUM_0

static void configure_reset_button(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << RESET_BUTTON_PIN),
        .pull_up_en = 1,
        .pull_down_en = 0,
    };
    gpio_config(&io_conf);
    printf("Botón de reset configurado en GPIO %d\n", RESET_BUTTON_PIN);
}

void app_main(void) {
    motor_init();
    #if !SIMULATE_ENCODER
    encoder_init();
    #endif
    configure_reset_button();
    simulink_control_initialize();

    printf("Inicializando sistema con control de Simulink...\n");
    #if SIMULATE_ENCODER
    printf("¡¡¡ MODO SIMULACIÓN DE ENCODER ACTIVO !!!\n");
    #endif
    printf("---------------------------------------------------------\n");

    // --- CAMBIO: Usaremos nuestro propio contador para el tiempo ---
    uint32_t time_counter_ms = 0;
    
    static float simulated_rpm = 0.0f;

    while (1) {
        if (gpio_get_level(RESET_BUTTON_PIN) == 0) {
            printf("--- RESET ---\n");
            // --- CAMBIO: Reiniciamos nuestro contador ---
            time_counter_ms = 0; 
            simulink_control_initialize(); 
            simulated_rpm = 0.0f;
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }

        // --- CAMBIO: Calculamos t_seconds desde nuestro contador ---
        float t_seconds = time_counter_ms / 1000.0f;
        
        float reference_rpm = trajectory_get_reference_rpm(t_seconds);
        
        float measured_rpm;
        #if SIMULATE_ENCODER
            measured_rpm = simulated_rpm;
        #else
            measured_rpm = encoder_get_rpm(TS_MS);
        #endif

        float error = reference_rpm - measured_rpm;
        float scaled_error = error / ERROR_SCALING_FACTOR;
        simulink_control_U.error_signal = scaled_error;

        simulink_control_step();
        float u_k = simulink_control_Y.u_k;

        if (u_k > 1.0) u_k = 1.0;
        if (u_k < 0.0) u_k = 0.0;
        
        float duty_cycle_to_set = DUTY_CYCLE_MIN + (u_k * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN));
        motor_set_duty_cycle(duty_cycle_to_set);
        
        #if SIMULATE_ENCODER
            // Mantenemos la simulación con la ganancia ajustada
            simulated_rpm = (0.95f * simulated_rpm) + (19.5f * u_k);
        #endif

        // Enviamos: Referencia, Medida, Señal de Control (u_k)
        printf("%.2f,%.2f,%.2f\n", reference_rpm, measured_rpm, u_k);
        
        // --- CAMBIO: Incrementamos nuestro contador ---
        time_counter_ms += TS_MS;

        vTaskDelay(pdMS_TO_TICKS(TS_MS));
    }
}