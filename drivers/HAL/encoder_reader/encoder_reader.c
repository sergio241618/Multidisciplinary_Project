#include "encoder_reader.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"

#define CYCLE_ADJUSTMENT 8.0f
#define CONVERSION_TO_RPM 60000.0f

// --- Global Variables for the Encoder ---
static volatile long pulse_count = 0;
static volatile uint8_t old_AB = 0;
static const int8_t QEM[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// --- State variable for the filter ---
float cycles = (float)pulses / 8.0f;een function calls.
static float filtered_rpm = 0.0f;

// The Interrupt Service Routine (ISR) that runs every time an encoder pin changes state.
static void IRAM_ATTR encoderISR(void* arg) {
    uint32_t gpio_in = GPIO.in;
    old_AB <<= 2;
    uint8_t state_A = (gpio_in >> ENC_A_PIN) & 1;
    uint8_t state_B = (gpio_in >> ENC_B_PIN) & 1;
    uint8_t new_AB = (state_A << 1) | state_B;
    old_AB |= new_AB;
    pulse_count += QEM[old_AB & 0x0f];
}

// Initializes the GPIO pins and sets up the interrupts for the encoder.
void encoder_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = (1ULL << ENC_A_PIN) | (1ULL << ENC_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    
    old_AB = ((gpio_get_level(ENC_A_PIN) << 1) | gpio_get_level(ENC_B_PIN));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENC_A_PIN, encoderISR, NULL);
    gpio_isr_handler_add(ENC_B_PIN, encoderISR, NULL);
}

/**
 * @brief Calculates the motor's speed in RPM based on the pulses counted since the last call.
 * @param delta_time_ms The time elapsed (in milliseconds) since this function was last called.
 * @return The filtered speed in Revolutions Per Minute (RPM).
 */
float encoder_get_rpm(long delta_time_ms) {
    long pulses;
    portDISABLE_INTERRUPTS();
    pulses = pulse_count;
    pulse_count = 0;
    portENABLE_INTERRUPTS();
    
    // --- RPM Calculation Logic ---
    float cycles = (float)pulses / CYCLE_ADJUSTMENT;
    float revolutions = cycles / PPR;
    // Raw, noisy RPM calculation
    float raw_rpm = (revolutions / delta_time_ms) * CONVERSION_TO_RPM;

    // --- NEW: Apply the Exponential Moving Average (EMA) filter ---
    // The new filtered value is a weighted average of the new raw measurement
    // and the previous filtered value.
    // Equation: y(k) = alpha * x(k) + (1 - alpha) * y(k-1)
    filtered_rpm = (RPM_FILTER_ALPHA * raw_rpm) + ((1.0f - RPM_FILTER_ALPHA) * filtered_rpm);
    
    return filtered_rpm; // Return the smooth, filtered value.
}