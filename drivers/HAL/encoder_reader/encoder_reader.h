#ifndef ENCODER_READER_H //header guard
#define ENCODER_READER_H

// --- Encoder Parameters ---
#define ENC_A_PIN   25     
#define ENC_B_PIN   26     
#define PPR         199.0f // Pulses Per Revolution

// --- NEW: RPM Filter Parameter ---
// The smoothing factor for the Exponential Moving Average filter.
// Value is between 0.0 and 1.0.
// A smaller value results in a smoother (but slower to react) signal.
#define RPM_FILTER_ALPHA 0.5f

/**
 * @brief Initializes the GPIO pins and interrupts for the encoder.
 */
void encoder_init();

/**
 * @brief Calculates the current filtered speed of the motor in RPM.
 * @note This function should be called at regular intervals (e.g., every 10ms).
 * @param delta_time_ms The time elapsed (in milliseconds) since the last time this function was called.
 * @return The filtered speed in Revolutions Per Minute (RPM) as a float.
 */
float encoder_get_rpm(long delta_time_ms);

#endif //header guard