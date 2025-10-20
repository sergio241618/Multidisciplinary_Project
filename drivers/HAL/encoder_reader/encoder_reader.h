#ifndef ENCODER_READER_H //header guard
#define ENCODER_READER_H

// --- Encoder Parameters ---
// Channel A output.
#define ENC_A_PIN   25     

// Channel B output.
#define ENC_B_PIN   26     

// Pulses Per Revolution for this specific encoder.
#define PPR         199.0f 

/**
 * @brief Initializes the GPIO pins and interrupts for the encoder.
 */
void encoder_init();
/**
 * @brief Calculates the current speed of the motor in RPM.
 * @note This function should be called at regular intervals (e.g., every 10ms).
 * @param delta_time_ms The time elapsed (in milliseconds) since the last time this function was called.
 * @return The calculated speed in Revolutions Per Minute (RPM) as a float.
 */
float encoder_get_rpm(long delta_time_ms);

#endif //header guard