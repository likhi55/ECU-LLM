#ifndef ECU_H
#define ECU_H

// SCR1
int compute_engine_state(int ignition_switch);

// SCR2
int parse_max_engine_speed(const char *calib_path, int fallback_rpm);

// SCR3
int parse_brake_gain(const char *calib_path, int fallback_gain);

/**
 * Update engine speed with accel + brake (SCR2 + SCR3).
 * - engine_state: 1 when ignition ON, else 0
 * - acc_pedal_position_deg: 0..45
 * - brake_pedal_position_deg: 0..45
 * - prev_engine_speed_rpm: previous iteration speed (>=0)
 * - max_engine_speed_rpm: clamp upper bound
 * - brake_gain_rpm_per_deg: rpm decrease per brake degree (e.g., 4)
 */
int update_engine_speed(int engine_state,
                        int acc_pedal_position_deg,
                        int brake_pedal_position_deg,
                        int prev_engine_speed_rpm,
                        int max_engine_speed_rpm,
                        int brake_gain_rpm_per_deg);

#endif
