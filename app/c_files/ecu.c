#include "ecu.h"
int compute_engine_state(int ignition_switch) {
    return ignition_switch ? 1 : 0;
}
