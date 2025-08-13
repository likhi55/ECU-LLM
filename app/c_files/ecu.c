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

    // First, compute baseline + cruise (reuse SCR5 function)
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
