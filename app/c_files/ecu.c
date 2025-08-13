#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ecu.h"

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

int update_engine_speed(int engine_state,
                        int acc_pedal_position_deg,
                        int brake_pedal_position_deg,
                        int prev_engine_speed_rpm,
                        int max_engine_speed_rpm,
                        int brake_gain_rpm_per_deg)
{
    if (engine_state == 0) {
        // ignition OFF → speed forced to zero
        return 0;
    }

    // sanitize inputs
    int acc   = clamp_int(acc_pedal_position_deg,   0, 45);
    int brake = clamp_int(brake_pedal_position_deg, 0, 45);
    int prev  = prev_engine_speed_rpm < 0 ? 0 : prev_engine_speed_rpm;

    // SCR2 accel: +2 rpm per degree; SCR3 brake: -gain rpm per degree
    long delta_acc = (long)acc * 2L;
    long delta_brk = (long)brake * (long)brake_gain_rpm_per_deg;
    long next = (long)prev + delta_acc - delta_brk;

    if (next < 0) next = 0;
    if (next > max_engine_speed_rpm) next = max_engine_speed_rpm;

    return (int)next;
}
