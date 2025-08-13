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
    for (int i = 0; i < ncols; i++) {
        if (strcmp(header_cols[i], name) == 0) return i;
    }
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

    // --- Calibrations (SCR2/3/4) ---
    const char *calib_env  = getenv("ECU_CALIB_PATH");
    const char *calib_path = (calib_env && calib_env[0]) ? calib_env : "app/calibration/calibration.txt";
    int max_engine_speed = parse_max_engine_speed(calib_path, 2000);
    int brake_gain       = parse_brake_gain(calib_path, 4);
    double gear_mult[6];
    parse_gear_multipliers(calib_path, gear_mult); // fills defaults if keys missing

    char line[MAX_LINE];
    char *cols[MAX_COLS];

    // --- Header ---
    if (!fgets(line, sizeof(line), fin)) {
        fprintf(stderr, "empty input\n");
        fclose(fin); fclose(fout);
        return 5;
    }
    int hcols     = split_csv(line, cols, MAX_COLS);
    int time_idx  = find_col(cols, hcols, "time");
    int ign_idx   = find_col(cols, hcols, "ignition_switch");
    int acc_idx   = find_col(cols, hcols, "acc_pedal_position");
    int brk_idx   = find_col(cols, hcols, "brake_pedal_position");
    int gear_idx  = find_col(cols, hcols, "current_gear");

    if (ign_idx < 0) {
        fprintf(stderr, "input header must contain 'ignition_switch'\n");
        fclose(fin); fclose(fout);
        return 6;
    }
    // acc/brake/gear missing → treated as 0 (acc/brake) and 3 (gear)

    // --- Output header (same as SCR2/3) ---
    fprintf(fout, "time,engine_state,engine_speed\n");

    long tgen = 0;
    int engine_speed = 0;

    // --- Rows ---
    while (fgets(line, sizeof(line), fin)) {
        int n = split_csv(line, cols, MAX_COLS);
        if (n == 0) continue;

        long t = tgen;
        if (time_idx >= 0 && time_idx < n && cols[time_idx][0] != '\0')
            t = strtol(cols[time_idx], NULL, 10);

        int ign = 0;
        if (ign_idx < n && cols[ign_idx][0] != '\0')
            ign = (int)strtol(cols[ign_idx], NULL, 10);
        int engine_state = compute_engine_state(ign);

        int acc_deg = 0;
        if (acc_idx >= 0 && acc_idx < n && cols[acc_idx][0] != '\0')
            acc_deg = (int)strtol(cols[acc_idx], NULL, 10);

        int brk_deg = 0;
        if (brk_idx >= 0 && brk_idx < n && cols[brk_idx][0] != '\0')
            brk_deg = (int)strtol(cols[brk_idx], NULL, 10);

        int gear = 3; // default mid gear if column absent
        if (gear_idx >= 0 && gear_idx < n && cols[gear_idx][0] != '\0')
            gear = (int)strtol(cols[gear_idx], NULL, 10);

        engine_speed = update_engine_speed(
            engine_state, acc_deg, brk_deg, gear,
            engine_speed, max_engine_speed, brake_gain, gear_mult
        );

        fprintf(fout, "%ld,%d,%d\n", t, engine_state, engine_speed);
        tgen++;
    }

    fclose(fin);
    fclose(fout);
    return 0;
}
