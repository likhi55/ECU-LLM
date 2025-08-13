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

    const char *in_path = argv[1];
    const char *out_path = argv[2];

    FILE *fin = fopen(in_path, "r");
    if (!fin) { perror("open input"); return 3; }
    FILE *fout = fopen(out_path, "w");
    if (!fout) { perror("open output"); fclose(fin); return 4; }

    char line[MAX_LINE];
    char *cols[MAX_COLS];

    if (!fgets(line, sizeof(line), fin)) { fprintf(stderr, "empty input\n"); fclose(fin); fclose(fout); return 5; }
    int hcols = split_csv(line, cols, MAX_COLS);
    int time_idx = find_col(cols, hcols, "time");
    int ign_idx  = find_col(cols, hcols, "ignition_switch");
    if (ign_idx < 0) { fprintf(stderr, "input header must contain 'ignition_switch'\n"); fclose(fin); fclose(fout); return 6; }

    fprintf(fout, "time,engine_state\n");

    long tgen = 0;
    while (fgets(line, sizeof(line), fin)) {
        int n = split_csv(line, cols, MAX_COLS);
        if (n == 0) continue;

        long t = (time_idx >= 0 && time_idx < n && cols[time_idx][0]) ? strtol(cols[time_idx], NULL, 10) : tgen;
        int ign = (ign_idx < n && cols[ign_idx][0]) ? (int)strtol(cols[ign_idx], NULL, 10) : 0;

        int engine_state = compute_engine_state(ign);
        fprintf(fout, "%ld,%d\n", t, engine_state);
        tgen++;
    }

    fclose(fin); fclose(fout);
    return 0;
}
