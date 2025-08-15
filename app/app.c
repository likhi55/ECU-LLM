#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ecu.h"

#define MAX_LINE 4096
#define MAX_COLS 256

static int split_csv(char *line, char *cols[], int maxcols) {
    int count = 0;
    char *p = line;
    while (*p && count < maxcols) {
        cols[count++] = p;
        while (*p && *p != ',' && *p != '\r' && *p != '\n') p++;
        if (*p == ',') { *p = '\0'; p++; }
    }
    if (count > 0) {
        char *last = cols[count-1];
        size_t n = strlen(last);
        while (n && (last[n-1] == '\n' || last[n-1] == '\r')) last[--n] = '\0';
    }
    return count;
}

static int find_col(char *header_cols[], int ncols, const char *name) {
    for (int i = 0; i < ncols; i++) if (strcmp(header_cols[i], name) == 0) return i;
    return -1;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <input.csv> <output.csv>\n", argv[0]);
        return 2;
    }

    const char *in_path  = argv[1];
    const char *out_path = argv[2];

    FILE *fin = fopen(in_path, "r");
    if (!fin) { perror("open input"); return 3; }
    FILE *fout = fopen(out_path, "w");
    if (!fout) { perror("open output"); fclose(fin); return 4; }

    // --- Calibrations (SCR2..SCR11) ---
    const char *calib_env  = getenv("ECU_CALIB_PATH");
    const char *calib_path = (calib_env && calib_env[0]) ? calib_env : "app/calibration/calibration.txt";
    int max_engine_speed = parse_max_engine_speed(calib_path, 2000);
    int brake_gain       = parse_brake_gain(calib_path, 4);
    double gear_mult[6]; parse_gear_multipliers(calib_path, gear_mult);

    double cc_kp; int cc_max_step, cc_gear_min, cc_tmin, cc_tmax;
    parse_cc_params(calib_path, &cc_kp, &cc_max_step, &cc_gear_min, &cc_tmin, &cc_tmax);

    int drag_rpm = parse_drag_rpm_per_iter(calib_path, 5);

    int idle_target, idle_max_step, idle_gear_max; double idle_kp;
    parse_idle_params(calib_path, &idle_target, &idle_kp, &idle_max_step, &idle_gear_max);

    int slew_rise, slew_fall; parse_slew_params(calib_path, &slew_rise, &slew_fall);

    int acc_overlap_deg, brk_overlap_deg, limp_rows_confirm, limp_max_speed, limp_clear_on_off;
    double limp_acc_gain_scale;
    parse_limp_params(calib_path,
                      &acc_overlap_deg, &brk_overlap_deg, &limp_rows_confirm,
                      &limp_max_speed, &limp_acc_gain_scale, &limp_clear_on_off);

    int rev_soft, rev_hard, rev_hyst, rev_cut_step, rev_cooldown_rows;
    parse_rev_params(calib_path,
                     &rev_soft, &rev_hard, &rev_hyst, &rev_cut_step, &rev_cooldown_rows);

    int bto_brake_deg, bto_acc_min_deg; double bto_acc_scale;
    parse_bto_params(calib_path, &bto_brake_deg, &bto_acc_min_deg, &bto_acc_scale);

    char line[MAX_LINE];
    char *cols[MAX_COLS];

    // --- Header ---
    if (!fgets(line, sizeof(line), fin)) {
        fprintf(stderr, "empty input\n");
        fclose(fin); fclose(fout);
        return 5;
    }
    int hcols      = split_csv(line, cols, MAX_COLS);
    int time_idx   = find_col(cols, hcols, "time");
    int ign_idx    = find_col(cols, hcols, "ignition_switch");
    int acc_idx    = find_col(cols, hcols, "acc_pedal_position");
    int brk_idx    = find_col(cols, hcols, "brake_pedal_position");
    int gear_idx   = find_col(cols, hcols, "current_gear");
    int cc_en_idx  = find_col(cols, hcols, "cruise_enable");
    int cc_tgt_idx = find_col(cols, hcols, "cruise_target_speed");

    if (ign_idx < 0) {
        fprintf(stderr, "input header must contain 'ignition_switch'\n");
        fclose(fin); fclose(fout);
        return 6;
    }

    // --- Output header ---
    fprintf(fout, "time,engine_state,engine_speed\n");

    long tgen = 0;
    int engine_speed = 0; // last *emitted* speed

    // SCR9: persistent limp state
    int limp_mode = 0;
    int overlap_run_count = 0;

    // SCR10: rev limiter state
    int hard_cut_active = 0;
    int hard_cut_cooldown = 0;

    // --- Rows ---
    while (fgets(line, sizeof(line), fin)) {
        int n = split_csv(line, cols, MAX_COLS);
        if (n == 0) continue;

        long t = tgen;
        if (time_idx >= 0 && time_idx < n && cols[time_idx][0]) t = strtol(cols[time_idx], NULL, 10);

        int ign = 0;
        if (ign_idx < n && cols[ign_idx][0]) ign = (int)strtol(cols[ign_idx], NULL, 10);
        int engine_state = compute_engine_state(ign);

        int acc_deg = 0;
        if (acc_idx >= 0 && acc_idx < n && cols[acc_idx][0]) acc_deg = (int)strtol(cols[acc_idx], NULL, 10);

        int brk_deg = 0;
        if (brk_idx >= 0 && brk_idx < n && cols[brk_idx][0]) brk_deg = (int)strtol(cols[brk_idx], NULL, 10);

        int gear = 3;
        if (gear_idx >= 0 && gear_idx < n && cols[gear_idx][0]) gear = (int)strtol(cols[gear_idx], NULL, 10);

        int cc_en = 0;
        if (cc_en_idx >= 0 && cc_en_idx < n && cols[cc_en_idx][0]) cc_en = (int)strtol(cols[cc_en_idx], NULL, 10);

        int cc_tgt = 0;
        if (cc_tgt_idx >= 0 && cc_tgt_idx < n && cols[cc_tgt_idx][0]) cc_tgt = (int)strtol(cols[cc_tgt_idx], NULL, 10);

        // SCR9: update limp state using raw pedals (plausibility)
        update_limp_state(
            engine_state, acc_deg, brk_deg,
            acc_overlap_deg, brk_overlap_deg, limp_rows_confirm, limp_clear_on_off,
            &limp_mode, &overlap_run_count
        );

        // Keep previously *emitted* speed for slew limiting and rev-limiter reference
        int prev_out = engine_speed;

        // SCR11: compute effective accelerator for baseline (non-latching BTO)
        int eff_acc_deg = apply_bto_effective_acc(
            acc_deg, brk_deg,
            bto_brake_deg, bto_acc_min_deg, bto_acc_scale
        );

        // SCR2..SCR7 provisional (use eff_acc_deg)
        int provisional = update_engine_speed_cc_drag_idle(
            engine_state,
            eff_acc_deg, brk_deg, gear,
            prev_out, max_engine_speed,
            brake_gain, gear_mult,
            cc_en, cc_tgt,
            cc_kp, cc_max_step, cc_gear_min, cc_tmin, cc_tmax,
            drag_rpm,
            idle_target, idle_kp, idle_max_step, idle_gear_max
        );

        // SCR9: limp cap (minimal)
        provisional = apply_limp_cap(provisional, max_engine_speed, limp_mode, limp_max_speed);

        // SCR10: rev limiter (after SCR9, before SCR8)
        provisional = apply_rev_limiter(
            engine_state, prev_out, provisional, max_engine_speed,
            &hard_cut_active, &hard_cut_cooldown,
            rev_soft, rev_hard, rev_hyst, rev_cut_step, rev_cooldown_rows
        );

        // SCR8: slew-rate limiting vs prev_out
        engine_speed = apply_slew_limit(
            engine_state, prev_out, provisional, max_engine_speed, slew_rise, slew_fall
        );

        fprintf(fout, "%ld,%d,%d\n", t, engine_state, engine_speed);
        tgen++;
    }

    fclose(fin);
    fclose(fout);
    return 0;
}
