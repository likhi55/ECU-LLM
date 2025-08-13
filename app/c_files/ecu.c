#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ecu.h"

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
        // support lines like: max_engine_speed = 2000
        int val = 0;
        if (sscanf(line, " max_engine_speed %*[^0-9]%d", &val) == 1 && val > 0) {
            maxrpm = val;
            break;
        }
    }
    fclose(f);
    return maxrpm;
}

int update_engine_speed(int engine_state,
                        int acc_pedal_position_deg,
                        int prev_engine_speed_rpm,
                        int max_engine_speed_rpm)
{
    if (engine_state == 0) {
        // ignition OFF → speed is zero
        return 0;
    }

    // sanitize inputs
    if (acc_pedal_position_deg < 0) acc_pedal_position_deg = 0;
    if (acc_pedal_position_deg > 45) acc_pedal_position_deg = 45;
    if (prev_engine_speed_rpm < 0) prev_engine_speed_rpm = 0;

    // Each degree adds 2 rpm per iteration
    int delta = acc_pedal_position_deg * 2;
    long next = (long)prev_engine_speed_rpm + (long)delta;

    if (next > max_engine_speed_rpm) next = max_engine_speed_rpm;
    if (next < 0) next = 0; // safety

    return (int)next;
}
