/*
 * ============================================================================
 * FILE: railway_controller.h
 * OWNERSHIP NOTE: TRAN VO VUONG (new module)
 *
 * Attribution in this file is based on comparison with the preserved baseline
 * in original_source/. "Proposed contribution" means code added/changed in the
 * improved version and should only be claimed after review, testing, and actual
 * contribution by the named team member.
 * ============================================================================
 */

#ifndef RAILWAY_CONTROLLER_H_
#define RAILWAY_CONTROLLER_H_

#include <stdbool.h>
#include "ipc.h"

/* [NEW MODULE] Railway safety/fault state owned by this module. */
/* PUBLIC API: Railway safety module used by local_controller.c.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
void railway_init(void);
void railway_handle_event(SensorEvent event);
RailState railway_get_state(void);
bool railway_requires_road_stop(void);
bool railway_has_fault(void);
const char *railway_state_label(RailState state);

#endif
