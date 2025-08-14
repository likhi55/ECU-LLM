#ifndef ECU_H
#define ECU_H

// ---------- SCR1 ----------
int compute_engine_state(int ignition_switch);

// ---------- SCR2 ----------
int parse_max_engine_speed(const char *calib_path, int fallback_rpm);

// ---------- SCR3 ----------
int parse_brake_gain(const char *calib_path, int fallback_gain);

// ---------- SCR4 ----------
int parse_gear_multipliers(const char *calib_path, double gear_mult[6]);
int update_engine_speed(int engine_state,
                        int acc_deg,
                        int brake_deg,
                        int current_gear,
                        int prev_speed_rpm,
                        int max_speed_rpm,
                        int brake_gain_rpm_per_deg,
                        const double gear_mult[6]);

// ---------- SCR5 ----------
int parse_cc_params(const char *calib_path,
                    double *cc_kp,
                    int *cc_max_step_per_iter,
                    int *cc_activation_gear_min,
                    int *cc_target_min,
                    int *cc_target_max);
int update_engine_speed_cc(int engine_state,
                           int acc_deg,
                           int brake_deg,
                           int current_gear,
                           int prev_speed_rpm,
                           int max_speed_rpm,
                           int brake_gain_rpm_per_deg,
                           const double gear_mult[6],
                           int cruise_enable,
                           int cruise_target_speed,
                           double cc_kp,
                           int cc_max_step_per_iter,
                           int cc_activation_gear_min,
                           int cc_target_min,
                           int cc_target_max);

// ---------- SCR6 ----------
int parse_drag_rpm_per_iter(const char *calib_path, int fallback_drag);
int update_engine_speed_cc_drag(int engine_state,
                                int acc_deg,
                                int brake_deg,
                                int current_gear,
                                int prev_speed_rpm,
                                int max_speed_rpm,
                                int brake_gain_rpm_per_deg,
                                const double gear_mult[6],
                                int cruise_enable,
                                int cruise_target_speed,
                                double cc_kp,
                                int cc_max_step_per_iter,
                                int cc_activation_gear_min,
                                int cc_target_min,
                                int cc_target_max,
                                int drag_rpm_per_iter);

// ---------- SCR7 ----------
int parse_idle_params(const char *calib_path,
                      int *idle_target_speed,
                      double *idle_kp,
                      int *idle_max_step_per_iter,
                      int *idle_activation_gear_max);
int update_engine_speed_cc_drag_idle(int engine_state,
                                     int acc_deg,
                                     int brake_deg,
                                     int current_gear,
                                     int prev_speed_rpm,
                                     int max_speed_rpm,
                                     int brake_gain_rpm_per_deg,
                                     const double gear_mult[6],
                                     int cruise_enable,
                                     int cruise_target_speed,
                                     double cc_kp,
                                     int cc_max_step_per_iter,
                                     int cc_activation_gear_min,
                                     int cc_target_min,
                                     int cc_target_max,
                                     int drag_rpm_per_iter,
                                     int idle_target_speed,
                                     double idle_kp,
                                     int idle_max_step_per_iter,
                                     int idle_activation_gear_max);

// ---------- SCR8 ----------
int parse_slew_params(const char *calib_path,
                      int *slew_max_rise_per_iter,
                      int *slew_max_fall_per_iter);
int apply_slew_limit(int engine_state,
                     int prev_output_speed_rpm,
                     int provisional_speed_rpm,
                     int max_speed_rpm,
                     int slew_max_rise_per_iter,
                     int slew_max_fall_per_iter);

// ---------- SCR9 (Pedal plausibility & limp mode) ----------
int parse_limp_params(const char *calib_path,
                      int *acc_overlap_deg,
                      int *brk_overlap_deg,
                      int *limp_rows_confirm,
                      int *limp_max_speed,
                      double *limp_acc_gain_scale,
                      int *limp_clear_on_ignition_off);

/**
 * Update limp-mode latch & counter based on current row.
 * - Mutates *limp_mode_io and *overlap_run_count_io.
 */
void update_limp_state(int engine_state,
                       int acc_deg,
                       int brake_deg,
                       int acc_overlap_deg,
                       int brk_overlap_deg,
                       int limp_rows_confirm,
                       int limp_clear_on_ignition_off,
                       int *limp_mode_io,
                       int *overlap_run_count_io);

/**
 * Apply limp cap to provisional speed (minimal implementation).
 * Returns min(provisional, min(limp_max_speed, max_engine_speed)) if limp_mode==1, else provisional.
 */
int apply_limp_cap(int provisional_speed_rpm,
                   int max_engine_speed,
                   int limp_mode,
                   int limp_max_speed);

#endif
