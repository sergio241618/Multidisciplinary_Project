#ifndef ENCODER_READER_H
#define ENCODER_READER_H

// --- Encoder Parameters ---
#define ENC_A_PIN   25
#define ENC_B_PIN   26
#define PPR         199.0f

// --- RPM Filter Parameter ---
// The smoothing factor for the Exponential Moving Average filter.
// A smaller value (e.g., 0.1) results in a smoother (but slower) signal.
#define RPM_FILTER_ALPHA 0.1f

// --- RPM Calculation Constants ---
// This factor likely accounts for 4x decoding and another project-specific calibration.
#define CYCLE_ADJUSTMENT 8.0f
// Constant to convert revolutions per millisecond to RPM.
#define CONVERSION_TO_RPM 60000.0f

/**
 * @brief Initializes the GPIO pins and interrupts for the encoder.
 */
void encoder_init();

/**
 * @brief Calculates the current filtered speed of the motor in RPM.
 * @param delta_time_ms The time elapsed (in milliseconds) since the last call.
 * @return The filtered speed in Revolutions Per Minute (RPM).
 */
float encoder_get_rpm(long delta_time_ms);

#endif // ENCODER_READER_H