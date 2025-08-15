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

// ---------- SCR9 ----------
int parse_limp_params(const char *calib_path,
                      int *acc_overlap_deg,
                      int *brk_overlap_deg,
                      int *limp_rows_confirm,
                      int *limp_max_speed,
                      double *limp_acc_gain_scale,
                      int *limp_clear_on_ignition_off);
void update_limp_state(int engine_state,
                       int acc_deg,
                       int brake_deg,
                       int acc_overlap_deg,
                       int brk_overlap_deg,
                       int limp_rows_confirm,
                       int limp_clear_on_ignition_off,
                       int *limp_mode_io,
                       int *overlap_run_count_io);
int apply_limp_cap(int provisional_speed_rpm,
                   int max_engine_speed,
                   int limp_mode,
                   int limp_max_speed);

// ---------- SCR10 ----------
int parse_rev_params(const char *calib_path,
                     int *rev_soft_limit,
                     int *rev_hard_limit,
                     int *rev_hysteresis,
                     int *rev_hard_cut_step,
                     int *rev_cut_cooldown_rows);
int apply_rev_limiter(int engine_state,
                      int prev_output_speed_rpm,
                      int provisional_speed_rpm,
                      int max_engine_speed,
                      int *hard_cut_active_io,
                      int *hard_cut_cooldown_io,
                      int rev_soft_limit,
                      int rev_hard_limit,
                      int rev_hysteresis,
                      int rev_hard_cut_step,
                      int rev_cut_cooldown_rows);

// ---------- SCR11 (BTO) ----------
int parse_bto_params(const char *calib_path,
                     int *bto_brake_deg,
                     int *bto_acc_min_deg,
                     double *bto_acc_scale);
/** Returns the effective accelerator angle after BTO scaling/cut. */
int apply_bto_effective_acc(int acc_deg,
                            int brake_deg,
                            int bto_brake_deg,
                            int bto_acc_min_deg,
                            double bto_acc_scale);
							
// ---------- SCR12 (BTO release ramp) ----------
int parse_bto_release_params(const char *calib_path,
                             int *bto_release_ramp_rows,
                             int *bto_release_reset_on_ign_off);

int apply_bto_release_ramp(int engine_state,
                           int raw_acc_deg,
                           int prev_eff_acc_deg,
                           int bto_release_ramp_rows,
                           int bto_release_reset_on_ign_off,
                           int *ramp_rows_left_io);
#endif
