#include "encoder_reader.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"

// --- Global Variables for the Encoder ---

// Stores the accumulated encoder ticks.
static volatile long pulse_count = 0;

// Stores the history of the last two states of pins A and B.
// It's an 8-bit integer, where the lower 4 bits are used to store the
// previous state (2 bits) and the current state (2 bits).
static volatile uint8_t old_AB = 0;

// Quadrature Encoder Matrix (QEM)
// It translates the 4-bit state history (old_AB) into a direction of movement.
//  +1: Clockwise rotation
//  -1: Counter-clockwise rotation
//   0: No change or an invalid state transition (bounce)
static const int8_t QEM[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};


//The Interrupt Service Routine (ISR) that runs every time an encoder pin changes state.
static void IRAM_ATTR encoderISR(void* arg) {
    // Read the state of ALL GPIO pins at once directly from the hardware register.
    uint32_t gpio_in = GPIO.in;

    // Shift the previous state two positions to the left.
    old_AB <<= 2;

    // Read the current state of pin A.
    uint8_t state_A = (gpio_in >> ENC_A_PIN) & 1;

    // Read the current state of pin B
    uint8_t state_B = (gpio_in >> ENC_B_PIN) & 1;

    // Combine the new states of A and B into a single 2-bit value.
    uint8_t new_AB = (state_A << 1) | state_B;

    // Combine the shifted old state with the new state to create a 4-bit history.
    old_AB |= new_AB;

    // Use the lower 4 bits of the state history as an index into the QEM lookup table.
    // The result (+1, -1, or 0) is added to the global pulse counter.
    pulse_count += QEM[old_AB & 0x0f];
}


//Initializes the GPIO pins and sets up the interrupts for the encoder.
void encoder_init() {
    gpio_config_t io_conf = {
        // Trigger the interrupt on any edge (rising or falling). This enables 4x decoding.
        .intr_type = GPIO_INTR_ANYEDGE,
        // Create a bitmask to specify which pins we are configuring (A and B).
        .pin_bit_mask = (1ULL << ENC_A_PIN) | (1ULL << ENC_B_PIN),
        // Set the pins as inputs.
        .mode = GPIO_MODE_INPUT,
        // Enable the internal pull-up resistors. This is crucial to prevent the pins
        // from "floating" when the encoder switches are open.
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    // Apply the configuration to the specified GPIO pins.
    gpio_config(&io_conf);
    
    // Initialize the 'old_AB' state variable by reading the current state of the pins.
    old_AB = ((gpio_get_level(ENC_A_PIN) << 1) | gpio_get_level(ENC_B_PIN));

    // Install the global ISR service for the GPIO driver.
    gpio_install_isr_service(0);    // (revolutions / delta_time_ms) gives revs/ms.
    // * 60000.0 (which is 60 seconds/min * 1000 ms/sec) converts this to revs/min.

    // Attach our specific ISR function ('encoderISR') to the A pin.
    gpio_isr_handler_add(ENC_A_PIN, encoderISR, NULL);
    // Attach the SAME ISR function to the B pin. Any change on either pin will now trigger the ISR.
    gpio_isr_handler_add(ENC_B_PIN, encoderISR, NULL);
}

/**
 * @brief Calculates the motor's speed in RPM based on the pulses counted since the last call.
 * @param delta_time_ms The time elapsed (in milliseconds) since this function was last called.
 * @return The calculated speed in Revolutions Per Minute (RPM).
 */
float encoder_get_rpm(long delta_time_ms) {
    long pulses;
    // Disable all interrupts temporarily. This is crucial to ensure that the 'pulse_count'
    // variable is not modified by the ISR while we are reading and resetting it.
    portDISABLE_INTERRUPTS();
    pulses = pulse_count; // Copy the accumulated pulse count to a local variable.
    pulse_count = 0;      // Reset the global counter for the next interval.
    portENABLE_INTERRUPTS();
    
    // --- RPM Calculation Logic ---
    float cycles = (float)pulses / 8.0f;
    
    // Convert cycles to full revolutions using the Pulses Per Revolution (PPR) constant.
    float revolutions = cycles / PPR;
    
    // Convert revolutions per millisecond to Revolutions Per Minute (RPM).
    float rpm = (revolutions / delta_time_ms) * 60000.0f;
    
    return rpm;
}