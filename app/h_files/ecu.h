#ifndef ECU_H
#define ECU_H

// SCR1
int compute_engine_state(int ignition_switch);

// SCR2
int parse_max_engine_speed(const char *calib_path, int fallback_rpm);
int update_engine_speed(int engine_state,
                        int acc_pedal_position_deg,
                        int prev_engine_speed_rpm,
                        int max_engine_speed_rpm);

#endif
