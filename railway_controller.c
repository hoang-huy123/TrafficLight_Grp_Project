/*
 * ============================================================================
 * FILE: railway_controller.c
 * OWNERSHIP NOTE: TRAN VO VUONG (new module)
 *
 * Attribution in this file is based on comparison with the preserved baseline
 * in original_source/. "Proposed contribution" means code added/changed in the
 * improved version and should only be claimed after review, testing, and actual
 * contribution by the named team member.
 * ============================================================================
 */

#include "railway_controller.h"
#include <pthread.h>

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Separate mutex keeps railway state thread-safe. */
static pthread_mutex_t rail_mutex = PTHREAD_MUTEX_INITIALIZER;
static RailState rail_state = RAIL_CLEAR;

/* SECTION: Initialise the independent railway safety state.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
void railway_init(void) {
    pthread_mutex_lock(&rail_mutex);
    rail_state = RAIL_CLEAR;
    pthread_mutex_unlock(&rail_mutex);
}

/* SECTION: Convert railway sensor events into explicit railway operating/fault states.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
void railway_handle_event(SensorEvent event) {
    pthread_mutex_lock(&rail_mutex);
    switch (event) {
        case TRAIN_APPROACHING: rail_state = RAIL_APPROACHING; break;
        case TRAIN_GATE_DOWN:   rail_state = RAIL_GATE_DOWN_STATE; break;
        case TRAIN_FAULT:       rail_state = RAIL_FAULT_STATE; break;
        case TRAIN_GATE_CLEAR:  rail_state = RAIL_CLEAR; break;
        default: break;
    }
    pthread_mutex_unlock(&rail_mutex);
}

/* SECTION: Return a thread-safe snapshot of the current railway state.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
RailState railway_get_state(void) {
    pthread_mutex_lock(&rail_mutex);
    RailState s = rail_state;
    pthread_mutex_unlock(&rail_mutex);
    return s;
}

/* SECTION: Report whether the current railway condition requires road traffic to stop.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
bool railway_requires_road_stop(void) {
    return railway_get_state() != RAIL_CLEAR;
}

/* SECTION: Report whether the boom-gate/railway subsystem is currently in a fault state.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
bool railway_has_fault(void) {
    return railway_get_state() == RAIL_FAULT_STATE;
}

/* SECTION: Convert railway states into readable diagnostic text.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
const char *railway_state_label(RailState state) {
    switch (state) {
        case RAIL_CLEAR: return "CLEAR";
        case RAIL_APPROACHING: return "APPROACH";
        case RAIL_GATE_DOWN_STATE: return "GATE-DOWN";
        case RAIL_FAULT_STATE: return "FAULT";
        default: return "UNKNOWN";
    }
}
