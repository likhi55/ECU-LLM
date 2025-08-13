#ifndef ECU_H
#define ECU_H

// ---------- SCR1 ----------
int compute_engine_state(int ignition_switch);

// ---------- SCR2 ----------
int parse_max_engine_speed(const char *calib_path, int fallback_rpm);

// ---------- SCR3 ----------
int parse_brake_gain(const char *calib_path, int fallback_gain);

// ---------- SCR4 ----------
/**
 * Load per-gear acceleration multipliers from calibration.
 * Expects keys:
 *   gear_acc_multiplier_g1 .. gear_acc_multiplier_g5
 * Defaults are used if keys are missing.
 * gear_mult[1..5] will be filled (index 0 unused).
 * Returns number of keys successfully parsed.
 */
int parse_gear_multipliers(const char *calib_path, double gear_mult[6]);

/**
 * Update engine speed using accel (gear-scaled) and brake.
 * - engine_state: 1 when ignition ON, else 0
 * - acc_deg, brake_deg: 0..45
 * - current_gear: 1..5 (clamped if out of range)
 * - prev_speed_rpm >= 0
 * - max_speed_rpm: clamp upper bound
 * - brake_gain_rpm_per_deg: rpm decrease per brake degree (e.g., 4)
 * - gear_mult[1..5]: multiplier per gear (index 0 unused)
 */
int update_engine_speed(int engine_state,
                        int acc_deg,
                        int brake_deg,
                        int current_gear,
                        int prev_speed_rpm,
                        int max_speed_rpm,
                        int brake_gain_rpm_per_deg,
                        const double gear_mult[6]);

#endif
