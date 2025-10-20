#include "trajectory_generator.h"
#include <math.h>

/**
 * @brief A helper function that calculates a specific 5th-order Bezier polynomial.
 * @param K1 A normalized time value (from 0.0 to 1.0) representing the progress
 * along the curve segment.
 * @return The output of the polynomial, which scales the velocity during ramps.
 * @note 'static' means this function is only visible and usable within this file.
 */
static float Bezier(float K1) {
    // These are the constant coefficients of the 5th-order polynomial.
    const float r1 = 252.0f, r2 = 1050.0f, r3 = 1800.0f;
    const float r4 = 1575.0f, r5 = 700.0f, r6 = 126.0f;

    // Pre-calculate powers of K1 for efficiency, avoiding repeated pow() calls.
    float K1_pow_2 = K1 * K1;
    float K1_pow_3 = K1_pow_2 * K1;
    float K1_pow_4 = K1_pow_3 * K1;
    float K1_pow_5 = K1_pow_4 * K1;

    // The polynomial equation: V = K1^5 * (r1 - r2*K1 + r3*K1^2 - ...)
    return K1_pow_5 * (r1 - (r2 * K1) + (r3 * K1_pow_2) - (r4 * K1_pow_3) + (r5 * K1_pow_4) - (r6 * K1_pow_5));
}

/**
 * @brief Calculates the reference speed in RPM for a given time 't'.
 * @param t_seconds The time elapsed since the start of the profile, in seconds.
 * @return The calculated reference speed in Revolutions Per Minute (RPM).
 */
float trajectory_get_reference_rpm(float t_seconds) {
    // --- Curve Parameters ---
    // KF is the base maximum velocity of the profile, in radians per second.
    const float KF_rad_s = 24.0f;

    // These constants define the shape of the original, unscaled time profile.
    const float shift_orig = 0.0f;
    const float tajuste_orig = 0.5f;
    const float duracion_original = 2.8f + shift_orig + tajuste_orig; // Results in 3.3s

    // The desired total duration for the entire trajectory.
    const float duracion_deseada = 40.0f; 

    // Calculate the scaling factor to stretch the 3.3s profile to the desired duration.
    const float factor_escala = duracion_deseada / duracion_original;

    // --- Scaled Time Markers ---

    const float T1 = (0.1f + shift_orig) * factor_escala;
    const float T2 = (0.5f + shift_orig) * factor_escala;
    const float T3 = (1.0f + shift_orig + tajuste_orig) * factor_escala;
    const float T4 = (1.7f + shift_orig + tajuste_orig) * factor_escala;
    const float T5 = (2.7f + shift_orig + tajuste_orig) * factor_escala;
    const float T6 = (2.8f + shift_orig + tajuste_orig) * factor_escala;

    // Variable to hold the calculated velocity in rad/s before the final conversion.
    float v_bezier_rad_s = 0.0f;

    // --- Velocity Profile Logic ---
    if (t_seconds <= T1) {
        // Segment 1: Initial hold at zero speed.
        v_bezier_rad_s = 0;
    } else if (t_seconds <= T2) {
        // Segment 2: Ramp up to 100% of KF using the Bezier curve.
        // K1 is the normalized time (0.0 to 1.0) within this segment.
        float K1 = (t_seconds - T1) / (T2 - T1);
        v_bezier_rad_s = KF_rad_s * Bezier(K1);
    } else if (t_seconds <= T3) {
        // Segment 3: Hold at constant 100% of KF.
        v_bezier_rad_s = KF_rad_s;
    } else if (t_seconds <= T4) {
        // Segment 4: Ramp down to 50% of KF using the Bezier curve.
        float K1 = (t_seconds - T3) / (T4 - T3);
        v_bezier_rad_s = KF_rad_s - KF_rad_s * 0.5f * Bezier(K1);
    } else if (t_seconds <= T5) {
        // Segment 5: Hold at constant 50% of KF.
        v_bezier_rad_s = (KF_rad_s * 0.5f);
    } else if (t_seconds <= T6) {
        // Segment 6: Ramp up to 75% of KF using the Bezier curve.
        float K1 = (t_seconds - T5) / (T6 - T5);
        v_bezier_rad_s = (KF_rad_s * 0.5f) + KF_rad_s * 0.25f * Bezier(K1);
    } else {
        // Final Segment: Hold at 75% of KF indefinitely after the profile ends.
        v_bezier_rad_s = KF_rad_s * (1.0f - 0.25f);
    }

    // --- Final Conversion to RPM ---
    // The calculated velocity in rad/s is converted to RPM.
    // The conversion factor is (60 seconds/minute) / (2 * PI radians/revolution).
    // Custom scaling factor of 1.7 is then applied to increase the final RPM.
    return v_bezier_rad_s * 9.5492965855f * 1.7f;
}