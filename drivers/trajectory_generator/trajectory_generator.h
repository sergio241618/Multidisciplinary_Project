#ifndef TRAJECTORY_GENERATOR_H //header guard
#define TRAJECTORY_GENERATOR_H

/**
 * @brief Calculates the reference speed in RPM for a given time 't'.
 *
 * This function implements a scaled Bezier curve to generate a smooth
 * velocity profile over a predefined duration.
 *
 * @param t_seconds The time elapsed since the start of the profile, in seconds.
 * @return float The calculated reference speed in Revolutions Per Minute (RPM).
 */
float trajectory_get_reference_rpm(float t_seconds);

#endif //header guard