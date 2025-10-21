#include "trajectory_generator.h"
#include <math.h>

//Constant definitions for trajectory generation

// --- Conversion & Scaling Factors ---
// Conversion factor from radians per second to Revolutions Per Minute (RPM)
#define RAD_S_TO_RPM 9.5492965855f
// Custom scaling factor to increase the final target RPM
#define RPM_SCALING_FACTOR 4.0f

// --- Bezier Curve Polynomial Coefficients ---
static const float R1 = 252.0f, R2 = 1050.0f, R3 = 1800.0f;
static const float R4 = 1575.0f, R5 = 700.0f, R6 = 126.0f;

// --- Base Velocity Profile Parameters ---
// Base maximum velocity of the profile, in radians per second
static const float KF_RAD_S = 24.0f;
// Original, unscaled time profile parameters
static const float ORIGINAL_SHIFT = 0.0f;
static const float ORIGINAL_ADJUSTMENT = 0.5f;
static const float ORIGINAL_DURATION = 2.8f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT;
// The desired total duration for the entire trajectory
static const float DESIRED_DURATION = 40.0f;

/**
 * @brief A helper function that calculates a specific 5th-order Bezier polynomial.
 * @param k1 A normalized time value (from 0.0 to 1.0) representing the progress
 * along the curve segment.
 * @return The output of the polynomial, which scales the velocity during ramps.
 */
static float Bezier(float k1) {
    // Pre-calculate powers of k1 for efficiency, avoiding repeated pow() calls.
    float k1_pow_2 = k1 * k1;
    float k1_pow_3 = k1_pow_2 * k1;
    float k1_pow_4 = k1_pow_3 * k1;
    float k1_pow_5 = k1_pow_4 * k1;

    // The polynomial equation: V = k1^5 * (r1 - r2*k1 + r3*k1^2 - ...)
    return k1_pow_5 * (R1 - (R2 * k1) + (R3 * k1_pow_2) - (R4 * k1_pow_3) + (R5 * k1_pow_4) - (R6 * k1_pow_5));
}

/**
 * @brief Calculates the reference speed in RPM for a given time 't'.
 * @param t_seconds The time elapsed since the start of the profile, in seconds.
 * @return The calculated reference speed in Revolutions Per Minute (RPM).
 */
float trajectory_get_reference_rpm(float t_seconds) {
    // Calculate the scaling factor to stretch the original profile to the desired duration.
    const float scale_factor = DESIRED_DURATION / ORIGINAL_DURATION;

    // --- Scaled Time Markers ---
    // An array holds the key time points that define each segment of the profile.
    const float time_markers[6] = {
        (0.1f + ORIGINAL_SHIFT) * scale_factor,                               // T1: End of initial hold
        (0.5f + ORIGINAL_SHIFT) * scale_factor,                               // T2: End of first ramp-up
        (1.0f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor,         // T3: End of first constant speed hold
        (1.7f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor,         // T4: End of ramp-down
        (2.7f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor,         // T5: End of second constant speed hold
        (2.8f + ORIGINAL_SHIFT + ORIGINAL_ADJUSTMENT) * scale_factor          // T6: End of second ramp-up
    };

    // --- Velocity Profile Logic ---
    float target_velocity_rad_s = 0.0f;
    int segment = 0;

    // Determine which segment the current time falls into.
    for (int i = 0; i < 6; i++) {
        if (t_seconds <= time_markers[i]) {
            segment = i + 1;
            break;
        }
    }
    // If time is past the last marker, it's the final segment.
    if (segment == 0) {
        segment = 7;
    }

    // Use a switch-case for clean, organized logic based on the current segment.
    switch (segment) {
        case 1: // Segment 1 (t <= T1): Initial hold at zero speed.
            target_velocity_rad_s = 0.0f;
            break;

        case 2: // Segment 2 (T1 < t <= T2): Ramp up to 100% of KF.
            float k1_ramp1 = (t_seconds - time_markers[0]) / (time_markers[1] - time_markers[0]);
            target_velocity_rad_s = KF_RAD_S * Bezier(k1_ramp1);
            break;

        case 3: // Segment 3 (T2 < t <= T3): Hold at constant 100% of KF.
            target_velocity_rad_s = KF_RAD_S;
            break;

        case 4: // Segment 4 (T3 < t <= T4): Ramp down to 50% of KF.
            float k1_ramp2 = (t_seconds - time_markers[2]) / (time_markers[3] - time_markers[2]);
            target_velocity_rad_s = KF_RAD_S - KF_RAD_S * 0.5f * Bezier(k1_ramp2);
            break;

        case 5: // Segment 5 (T4 < t <= T5): Hold at constant 50% of KF.
            target_velocity_rad_s = KF_RAD_S * 0.5f;
            break;

        case 6: // Segment 6 (T5 < t <= T6): Ramp up to 75% of KF.
            float k1_ramp3 = (t_seconds - time_markers[4]) / (time_markers[5] - time_markers[4]);
            target_velocity_rad_s = (KF_RAD_S * 0.5f) + KF_RAD_S * 0.25f * Bezier(k1_ramp3);
            break;

        case 7: // Final Segment (t > T6): Hold at 75% of KF indefinitely.
        default:
            target_velocity_rad_s = KF_RAD_S * (1.0f - 0.25f);
            break;
    }

    // --- Final Conversion to RPM ---
    return target_velocity_rad_s * RAD_S_TO_RPM * RPM_SCALING_FACTOR;
}