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
    if (engine_state == 0) {
        return 0; // ignition OFF
    }

    int acc   = clamp_int(acc_deg,   0, 45);
    int brake = clamp_int(brake_deg, 0, 45);
    int gear  = clamp_int(current_gear, 1, 5);
    int prev  = (prev_speed_rpm < 0) ? 0 : prev_speed_rpm;

    // Acceleration scaled by gear
    double gain_deg = ACC_BASE_GAIN_RPM_PER_DEG * gear_mult[gear];
    double delta_acc = (double)acc * gain_deg;

    // Brake deceleration
    double delta_brk = (double)brake * (double)brake_gain_rpm_per_deg;

    // Next speed (rounded to nearest int after clamping)
    double next_d = (double)prev + delta_acc - delta_brk;
    if (next_d < 0.0) next_d = 0.0;
    if (next_d > (double)max_speed_rpm) next_d = (double)max_speed_rpm;

    int next_i = (int)(next_d + 0.5);  // round half up
    if (next_i < 0) next_i = 0;
    if (next_i > max_speed_rpm) next_i = max_speed_rpm;
    return next_i;
}
