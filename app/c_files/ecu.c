#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ecu.h"

#define ACC_BASE_GAIN_RPM_PER_DEG 2.0  // SCR2 base gain

static int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static double clamp_double(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static int round_to_int(double x) {
    return (int)(x + (x >= 0.0 ? 0.5 : -0.5));
}

// ---------- SCR1 ----------
int compute_engine_state(int ignition_switch) {
    return ignition_switch ? 1 : 0;
}

// ---------- SCR2 ----------
int parse_max_engine_speed(const char *calib_path, int fallback_rpm) {
    FILE *f = fopen(calib_path, "r");
    if (!f) return fallback_rpm;

    char line[512];
    int maxrpm = fallback_rpm;
    while (fgets(line, sizeof(line), f)) {
        int val = 0;
        if (sscanf(line, " max_engine_speed %*[^0-9]%d", &val) == 1 && val > 0) {
            maxrpm = val;
            break;
        }
    }
    fclose(f);
    return maxrpm;
}

// ---------- SCR3 ----------
int parse_brake_gain(const char *calib_path, int fallback_gain) {
    FILE *f = fopen(calib_path, "r");
    if (!f) return fallback_gain;

    char line[512];
    int gain = fallback_gain;
    while (fgets(line, sizeof(line), f)) {
        int val = 0;
        if (sscanf(line, " brake_gain_rpm_per_deg %*[^0-9]%d", &val) == 1 && val >= 0) {
            gain = val;
            break;
        }
    }
    fclose(f);
    return gain;
}

// ---------- SCR4 ----------
int parse_gear_multipliers(const char *calib_path, double gear_mult[6]) {
    // defaults
    gear_mult[0] = 0.0;   // unused
    gear_mult[1] = 0.60;
    gear_mult[2] = 0.85;
    gear_mult[3] = 1.00;
    gear_mult[4] = 1.10;
    gear_mult[5] = 1.20;

    FILE *f = fopen(calib_path, "r");
    if (!f) return 0;

    int parsed = 0;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        double v;
        if (sscanf(line, " gear_acc_multiplier_g1 %*[^0-9.-]%lf", &v) == 1) { gear_mult[1] = v; parsed++; continue; }
        if (sscanf(line, " gear_acc_multiplier_g2 %*[^0-9.-]%lf", &v) == 1) { gear_mult[2] = v; parsed++; continue; }
        if (sscanf(line, " gear_acc_multiplier_g3 %*[^0-9.-]%lf", &v) == 1) { gear_mult[3] = v; parsed++; continue; }
        if (sscanf(line, " gear_acc_multiplier_g4 %*[^0-9.-]%lf", &v) == 1) { gear_mult[4] = v; parsed++; continue; }
        if (sscanf(line, " gear_acc_multiplier_g5 %*[^0-9.-]%lf", &v) == 1) { gear_mult[5] = v; parsed++; continue; }
    }
    fclose(f);
    return parsed;
}

int update_engine_speed(int engine_state,
                        int acc_deg,
                        int brake_deg,
                        int current_gear,
                        int prev_speed_rpm,
                        int max_speed_rpm,
                        int brake_gain_rpm_per_deg,
                        const double gear_mult[6])
{
    if (engine_state == 0) return 0;

    int acc   = clamp_int(acc_deg,   0, 45);
    int brake = clamp_int(brake_deg, 0, 45);
    int gear  = clamp_int(current_gear, 1, 5);
    int prev  = (prev_speed_rpm < 0) ? 0 : prev_speed_rpm;

    double gain_deg = ACC_BASE_GAIN_RPM_PER_DEG * gear_mult[gear];
    double delta_acc = (double)acc * gain_deg;
    double delta_brk = (double)brake * (double)brake_gain_rpm_per_deg;

    double next = (double)prev + delta_acc - delta_brk;
    next = clamp_double(next, 0.0, (double)max_speed_rpm);
    return round_to_int(next);
}

// ---------- SCR5 ----------
int parse_cc_params(const char *calib_path,
                    double *cc_kp,
                    int *cc_max_step_per_iter,
                    int *cc_activation_gear_min,
                    int *cc_target_min,
                    int *cc_target_max)
{
    // defaults
    if (cc_kp)                  *cc_kp = 0.2;
    if (cc_max_step_per_iter)   *cc_max_step_per_iter = 30;
    if (cc_activation_gear_min) *cc_activation_gear_min = 3;
    if (cc_target_min)          *cc_target_min = 300;
    if (cc_target_max)          *cc_target_max = 2000;

    FILE *f = fopen(calib_path, "r");
    if (!f) return 0;

    int parsed = 0;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        double dv; int iv;
        if (cc_kp && sscanf(line, " cc_kp %*[^0-9.-]%lf", &dv) == 1) { *cc_kp = dv; parsed++; continue; }
        if (cc_max_step_per_iter && sscanf(line, " cc_max_step_per_iter %*[^0-9-]%d", &iv) == 1) { *cc_max_step_per_iter = iv; parsed++; continue; }
        if (cc_activation_gear_min && sscanf(line, " cc_activation_gear_min %*[^0-9-]%d", &iv) == 1) { *cc_activation_gear_min = iv; parsed++; continue; }
        if (cc_target_min && sscanf(line, " cc_target_min %*[^0-9-]%d", &iv) == 1) { *cc_target_min = iv; parsed++; continue; }
        if (cc_target_max && sscanf(line, " cc_target_max %*[^0-9-]%d", &iv) == 1) { *cc_target_max = iv; parsed++; continue; }
    }
    fclose(f);
    return parsed;
}

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
                           int cc_target_max)
{
    if (engine_state == 0) return 0;

    int acc   = clamp_int(acc_deg,   0, 45);
    int brake = clamp_int(brake_deg, 0, 45);
    int gear  = clamp_int(current_gear, 1, 5);
    int prev  = (prev_speed_rpm < 0) ? 0 : prev_speed_rpm;

    // Baseline (SCR2–SCR4)
    double gain_deg = ACC_BASE_GAIN_RPM_PER_DEG * gear_mult[gear];
    double delta_acc = (double)acc * gain_deg;
    double delta_brk = (double)brake * (double)brake_gain_rpm_per_deg;
    double next = (double)prev + delta_acc - delta_brk;

    // Cruise (SCR5)
    int cruise_ok = (cruise_enable == 1) &&
                    (brake == 0) &&
                    (acc == 0) &&
                    (gear >= cc_activation_gear_min);

    if (cruise_ok) {
        int tgt_hi = (cc_target_max < max_speed_rpm) ? cc_target_max : max_speed_rpm;
        int target = clamp_int(cruise_target_speed, cc_target_min, tgt_hi);
        double error = (double)target - (double)prev; // based on prev
        double delta_cc = cc_kp * error;
        if (delta_cc > (double)cc_max_step_per_iter)   delta_cc = (double)cc_max_step_per_iter;
        if (delta_cc < (double)(-cc_max_step_per_iter)) delta_cc = (double)(-cc_max_step_per_iter);
        next += delta_cc;
    }

    next = clamp_double(next, 0.0, (double)max_speed_rpm);
    return round_to_int(next);
}

// ---------- SCR6 ----------
int parse_drag_rpm_per_iter(const char *calib_path, int fallback_drag) {
    FILE *f = fopen(calib_path, "r");
    if (!f) return fallback_drag;

    char line[512];
    int drag = fallback_drag;
    while (fgets(line, sizeof(line), f)) {
        int val = 0;
        if (sscanf(line, " drag_rpm_per_iter %*[^0-9-]%d", &val) == 1 && val >= 0) {
            drag = val;
            break;
        }
    }
    fclose(f);
    return drag;
}

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
                                int drag_rpm_per_iter)
{
    if (engine_state == 0) return 0;

    // Baseline + cruise (SCR5)
    int speed_after_scr5 = update_engine_speed_cc(
        engine_state, acc_deg, brake_deg, current_gear, prev_speed_rpm,
        max_speed_rpm, brake_gain_rpm_per_deg, gear_mult,
        cruise_enable, cruise_target_speed,
        cc_kp, cc_max_step_per_iter, cc_activation_gear_min, cc_target_min, cc_target_max
    );

    // Apply coastdown only when: ON, no pedals, cruise disabled
    int acc   = clamp_int(acc_deg,   0, 45);
    int brake = clamp_int(brake_deg, 0, 45);
    if (acc == 0 && brake == 0 && cruise_enable == 0) {
        int next = speed_after_scr5 - (drag_rpm_per_iter >= 0 ? drag_rpm_per_iter : 0);
        if (next < 0) next = 0;
        if (next > max_speed_rpm) next = max_speed_rpm;
        return next;
    }

    return speed_after_scr5;
}

// ---------- SCR7 ----------
int parse_idle_params(const char *calib_path,
                      int *idle_target_speed,
                      double *idle_kp,
                      int *idle_max_step_per_iter,
                      int *idle_activation_gear_max)
{
    // defaults
    if (idle_target_speed)        *idle_target_speed = 600;
    if (idle_kp)                  *idle_kp = 0.2;
    if (idle_max_step_per_iter)   *idle_max_step_per_iter = 15;
    if (idle_activation_gear_max) *idle_activation_gear_max = 5;

    FILE *f = fopen(calib_path, "r");
    if (!f) return 0;

    int parsed = 0;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        double dv; int iv;
        if (idle_target_speed && sscanf(line, " idle_target_speed %*[^0-9-]%d", &iv) == 1) { *idle_target_speed = iv; parsed++; continue; }
        if (idle_kp && sscanf(line, " idle_kp %*[^0-9.-]%lf", &dv) == 1) { *idle_kp = dv; parsed++; continue; }
        if (idle_max_step_per_iter && sscanf(line, " idle_max_step_per_iter %*[^0-9-]%d", &iv) == 1) { *idle_max_step_per_iter = iv; parsed++; continue; }
        if (idle_activation_gear_max && sscanf(line, " idle_activation_gear_max %*[^0-9-]%d", &iv) == 1) { *idle_activation_gear_max = iv; parsed++; continue; }
    }
    fclose(f);
    return parsed;
}

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
                                     int idle_activation_gear_max)
{
    if (engine_state == 0) return 0;

    // First: baseline + cruise + coastdown (SCR2..SCR6)
    int after_drag = update_engine_speed_cc_drag(
        engine_state, acc_deg, brake_deg, current_gear, prev_speed_rpm,
        max_speed_rpm, brake_gain_rpm_per_deg, gear_mult,
        cruise_enable, cruise_target_speed,
        cc_kp, cc_max_step_per_iter, cc_activation_gear_min, cc_target_min, cc_target_max,
        drag_rpm_per_iter
    );

    // Idle control conditions (use prev speed for the error)
    int acc   = clamp_int(acc_deg,   0, 45);
    int brake = clamp_int(brake_deg, 0, 45);
    int gear  = clamp_int(current_gear, 1, 5);

    int idle_ok = (engine_state == 1) &&
                  (acc == 0) && (brake == 0) &&
                  (cruise_enable == 0) &&
                  (gear <= idle_activation_gear_max) &&
                  (prev_speed_rpm < idle_target_speed);

    if (!idle_ok) {
        return after_drag;
    }

    int error = idle_target_speed - (prev_speed_rpm < 0 ? 0 : prev_speed_rpm);
    double delta_idle_d = idle_kp * (double)error;
    if (delta_idle_d < 0.0) delta_idle_d = 0.0;
    if (delta_idle_d > (double)idle_max_step_per_iter) delta_idle_d = (double)idle_max_step_per_iter;

    int next = after_drag + round_to_int(delta_idle_d);
    if (next < 0) next = 0;
    if (next > max_speed_rpm) next = max_speed_rpm;
    return next;
}

// ---------- SCR8 ----------
int parse_slew_params(const char *calib_path,
                      int *slew_max_rise_per_iter,
                      int *slew_max_fall_per_iter)
{
    // defaults
    if (slew_max_rise_per_iter) *slew_max_rise_per_iter = 50;
    if (slew_max_fall_per_iter) *slew_max_fall_per_iter = 80;

    FILE *f = fopen(calib_path, "r");
    if (!f) return 0;

    int parsed = 0;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        int iv;
        if (slew_max_rise_per_iter &&
            sscanf(line, " slew_max_rise_per_iter %*[^0-9-]%d", &iv) == 1) {
            *slew_max_rise_per_iter = (iv < 0) ? 0 : iv; parsed++; continue;
        }
        if (slew_max_fall_per_iter &&
            sscanf(line, " slew_max_fall_per_iter %*[^0-9-]%d", &iv) == 1) {
            *slew_max_fall_per_iter = (iv < 0) ? 0 : iv; parsed++; continue;
        }
    }
    fclose(f);
    return parsed;
}

int apply_slew_limit(int engine_state,
                     int prev_output_speed_rpm,
                     int provisional_speed_rpm,
                     int max_speed_rpm,
                     int slew_max_rise_per_iter,
                     int slew_max_fall_per_iter)
{
    if (engine_state == 0) return 0;

    if (slew_max_rise_per_iter < 0) slew_max_rise_per_iter = 0;
    if (slew_max_fall_per_iter < 0) slew_max_fall_per_iter = 0;

    int prev = prev_output_speed_rpm < 0 ? 0 : prev_output_speed_rpm;
    int tmp  = provisional_speed_rpm;
    if (tmp < 0) tmp = 0;
    if (tmp > max_speed_rpm) tmp = max_speed_rpm;

    long delta = (long)tmp - (long)prev;
    if (delta > (long)slew_max_rise_per_iter) {
        tmp = prev + slew_max_rise_per_iter;
    } else if (delta < -(long)slew_max_fall_per_iter) {
        tmp = prev - slew_max_fall_per_iter;
    }
    if (tmp < 0) tmp = 0;
    if (tmp > max_speed_rpm) tmp = max_speed_rpm;
    return tmp;
}

// ---------- SCR9 ----------
int parse_limp_params(const char *calib_path,
                      int *acc_overlap_deg,
                      int *brk_overlap_deg,
                      int *limp_rows_confirm,
                      int *limp_max_speed,
                      double *limp_acc_gain_scale,
                      int *limp_clear_on_ignition_off)
{
    // defaults
    if (acc_overlap_deg)            *acc_overlap_deg = 10;
    if (brk_overlap_deg)            *brk_overlap_deg = 10;
    if (limp_rows_confirm)          *limp_rows_confirm = 2;
    if (limp_max_speed)             *limp_max_speed = 300;
    if (limp_acc_gain_scale)        *limp_acc_gain_scale = 0.3;
    if (limp_clear_on_ignition_off) *limp_clear_on_ignition_off = 1;

    FILE *f = fopen(calib_path, "r");
    if (!f) return 0;

    int parsed = 0; char line[512];
    while (fgets(line, sizeof(line), f)) {
        int iv; double dv;
        if (acc_overlap_deg && sscanf(line, " acc_overlap_deg %*[^0-9-]%d", &iv) == 1) { *acc_overlap_deg = (iv<0?0:iv); parsed++; continue; }
        if (brk_overlap_deg && sscanf(line, " brk_overlap_deg %*[^0-9-]%d", &iv) == 1) { *brk_overlap_deg = (iv<0?0:iv); parsed++; continue; }
        if (limp_rows_confirm && sscanf(line, " limp_rows_confirm %*[^0-9-]%d", &iv) == 1) { *limp_rows_confirm = (iv<1?1:iv); parsed++; continue; }
        if (limp_max_speed && sscanf(line, " limp_max_speed %*[^0-9-]%d", &iv) == 1) { *limp_max_speed = (iv<0?0:iv); parsed++; continue; }
        if (limp_acc_gain_scale && sscanf(line, " limp_acc_gain_scale %*[^0-9.-]%lf", &dv) == 1) {
            if (dv < 0.0) dv = 0.0; if (dv > 1.0) dv = 1.0; *limp_acc_gain_scale = dv; parsed++; continue;
        }
        if (limp_clear_on_ignition_off && sscanf(line, " limp_clear_on_ignition_off %*[^0-9-]%d", &iv) == 1) {
            *limp_clear_on_ignition_off = (iv ? 1 : 0); parsed++; continue;
        }
    }
    fclose(f);
    return parsed;
}

// ---------- SCR10 ----------
int parse_rev_params(const char *calib_path,
                     int *rev_soft_limit,
                     int *rev_hard_limit,
                     int *rev_hysteresis,
                     int *rev_hard_cut_step,
                     int *rev_cut_cooldown_rows)
{
    // defaults
    if (rev_soft_limit)         *rev_soft_limit = 1800;
    if (rev_hard_limit)         *rev_hard_limit = 1950;
    if (rev_hysteresis)         *rev_hysteresis = 50;
    if (rev_hard_cut_step)      *rev_hard_cut_step = 60;
    if (rev_cut_cooldown_rows)  *rev_cut_cooldown_rows = 2;

    FILE *f = fopen(calib_path, "r");
    if (!f) return 0;

    int parsed = 0; char line[512];
    while (fgets(line, sizeof(line), f)) {
        int iv;
        if (rev_soft_limit && sscanf(line, " rev_soft_limit %*[^0-9-]%d", &iv) == 1) { *rev_soft_limit = (iv<0?0:iv); parsed++; continue; }
        if (rev_hard_limit && sscanf(line, " rev_hard_limit %*[^0-9-]%d", &iv) == 1) { *rev_hard_limit = (iv<0?0:iv); parsed++; continue; }
        if (rev_hysteresis && sscanf(line, " rev_hysteresis %*[^0-9-]%d", &iv) == 1) { *rev_hysteresis = (iv<0?0:iv); parsed++; continue; }
        if (rev_hard_cut_step && sscanf(line, " rev_hard_cut_step %*[^0-9-]%d", &iv) == 1) { *rev_hard_cut_step = (iv<0?0:iv); parsed++; continue; }
        if (rev_cut_cooldown_rows && sscanf(line, " rev_cut_cooldown_rows %*[^0-9-]%d", &iv) == 1) { *rev_cut_cooldown_rows = (iv<0?0:iv); parsed++; continue; }
    }
    fclose(f);
    return parsed;
}

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
                      int rev_cut_cooldown_rows)
{
    if (engine_state == 0) {
        if (hard_cut_active_io)  *hard_cut_active_io = 0;
        if (hard_cut_cooldown_io) *hard_cut_cooldown_io = 0;
        return 0;
    }

    // Sanitize calibrations and bound to max
    if (rev_soft_limit < 0) rev_soft_limit = 0;
    if (rev_hard_limit < 0) rev_hard_limit = 0;
    if (rev_hysteresis < 0) rev_hysteresis = 0;
    if (rev_hard_cut_step < 0) rev_hard_cut_step = 0;
    if (rev_cut_cooldown_rows < 0) rev_cut_cooldown_rows = 0;

    if (rev_soft_limit > max_engine_speed) rev_soft_limit = max_engine_speed;
    if (rev_hard_limit > max_engine_speed) rev_hard_limit = max_engine_speed;
    if (rev_soft_limit >= rev_hard_limit) {
        rev_soft_limit = (rev_hard_limit > 0) ? (rev_hard_limit - 1) : 0;
    }

    int prev_out = prev_output_speed_rpm < 0 ? 0 : prev_output_speed_rpm;
    int tmp = provisional_speed_rpm;
    if (tmp < 0) tmp = 0;
    if (tmp > max_engine_speed) tmp = max_engine_speed;

    int hard_active = (hard_cut_active_io && *hard_cut_active_io) ? 1 : 0;
    int cooldown    = (hard_cut_cooldown_io ? *hard_cut_cooldown_io : 0);

    // Hard limiter logic with hysteresis
    if (hard_active) {
        int pull = prev_out - rev_hard_cut_step;
        if (tmp > pull) tmp = pull;
        if (cooldown > 0) cooldown--;
        if (prev_out <= (rev_hard_limit - rev_hysteresis) && cooldown == 0) {
            hard_active = 0; // release
        }
    } else {
        if (tmp > rev_hard_limit || prev_out > rev_hard_limit) {
            hard_active = 1;
            cooldown = rev_cut_cooldown_rows;
            if (tmp > rev_hard_limit) tmp = rev_hard_limit;
        }
    }

    // Soft ceiling
    if (tmp > rev_soft_limit) tmp = rev_soft_limit;

    // Clamp to [0, max]
    if (tmp < 0) tmp = 0;
    if (tmp > max_engine_speed) tmp = max_engine_speed;

    // Write back state
    if (hard_cut_active_io)   *hard_cut_active_io = hard_active;
    if (hard_cut_cooldown_io) *hard_cut_cooldown_io = cooldown;

    return tmp;
}
