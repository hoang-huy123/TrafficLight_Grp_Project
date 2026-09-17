/*
 * ============================================================================
 * FILE: ipc.h
 * OWNERSHIP NOTE: MIXED: baseline by DAM HOANG HUY; proposed improvements by TRAN VO VUONG
 *
 * Attribution in this file is based on comparison with the preserved baseline
 * in original_source/. "Proposed contribution" means code added/changed in the
 * improved version and should only be claimed after review, testing, and actual
 * contribution by the named team member.
 * ============================================================================
 */

#ifndef IPC_H_
#define IPC_H_

#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <stdio.h>
#include <stdint.h>

/* SECTION: Shared traffic-light state definitions used by all QNX processes. */
/* [ORIGINAL BASELINE - DAM HOANG HUY] Road-light states retained from the team's implementation. */
typedef enum {
    V_GREEN, V_AMBER, V_RED,
    H_GREEN, H_AMBER, H_RED,
    ALL_RED, PED_CROSS, RAIL_SAFE
} LightState;

/* [ORIGINAL BASELINE - DAM HOANG HUY] Existing operating modes retained. */
typedef enum { MODE_CONGESTION, MODE_SENSOR } OpMode;

/* SECTION: Explicit railway safety/fault state shared across Local, Central and Display. */
/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Railway state is explicit instead of one train_warning boolean. */
typedef enum {
    RAIL_CLEAR,
    RAIL_APPROACHING,
    RAIL_GATE_DOWN_STATE,
    RAIL_FAULT_STATE
} RailState;

typedef union {
    union { _Uint32t sival_int; void *sival_ptr; };
    _Uint32t dummy[4];
} _mysigval;

typedef struct _Mypulse {
    _Uint16t type; _Uint16t subtype; _Int8t code; _Uint8t zero[3];
    _mysigval value; _Uint8t zero2[2]; _Int32t scoid;
} msg_header_t;

typedef enum {
    MSG_STATUS_UPDATE = _IO_MAX + 1,
    MSG_COMMAND_SET_MODE,
    MSG_SENSOR_EVENT,
    /* [NEW] Control-room priority override (e.g. clear path for an emergency
     * vehicle or visiting dignitary), as required by the project brief. */
    MSG_COMMAND_OVERRIDE
} MsgType;

/* [NEW] Override directions carried in CommandMsg.hold_green. */
#define OVERRIDE_NONE        0   /* cancel any active override        */
#define OVERRIDE_VERTICAL    1   /* hold the vertical road green      */
#define OVERRIDE_HORIZONTAL  2   /* hold the horizontal road green    */

/* [NEW] Bounds applied by the local controller so a bad or malicious command
 * can never hold a green indefinitely (fail-safe: overrides always expire). */
#define OVERRIDE_DEFAULT_SECONDS 20
#define OVERRIDE_MAX_SECONDS     120

typedef struct {
    msg_header_t hdr;
    int intersection_id;
    OpMode mode;
    LightState V_light;
    LightState H_light;
    int train_warning;       /* [ORIGINAL/COMPATIBILITY] retained for old display logic. */
    RailState rail_state;    /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] distinguishes approach/gate/fault. */
    int pedestrian_pending;  /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] observable pedestrian demand. */
    int override_dir;        /* [NEW] OVERRIDE_* currently active at this intersection. */
    int override_remaining;  /* [NEW] seconds left on the active override (0 if none). */
} StatusMsg;

typedef struct {
    msg_header_t hdr;
    OpMode target_mode;
    /* [WAS DEAD CODE - NOW IMPLEMENTED] hold_green was previously declared but
     * never read by any controller. It now carries the override direction
     * (OVERRIDE_NONE / OVERRIDE_VERTICAL / OVERRIDE_HORIZONTAL). */
    int hold_green;
    int hold_seconds;        /* [NEW] requested override duration in seconds. */
    int target_intersection; /* [NEW] 0 = all intersections, 1..6 = one only. */
} CommandMsg;

/* [ORIGINAL BASELINE - DAM HOANG HUY] Event names retained. */
typedef enum {
    TRAIN_GATE_DOWN,
    TRAIN_GATE_CLEAR,
    TRAIN_FAULT,
    PEDESTRIAN_PRESSED,
    CAR_DETECTED_V,
    CAR_DETECTED_H,
    TRAIN_APPROACHING       /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] separate approach event. */
} SensorEvent;

typedef struct { msg_header_t hdr; SensorEvent event; } SensorMsg;

#define REPLY_BUF_SIZE 100
typedef struct { msg_header_t hdr; char buf[REPLY_BUF_SIZE]; } AckReply;

typedef union {
    msg_header_t hdr;
    StatusMsg status;
    CommandMsg command;
    SensorMsg sensor;
} Anymsg;

#define NAME_MAXLEN 64
static inline void local_ctrl_name(char *buf, int id) {
    snprintf(buf, NAME_MAXLEN, "local_%d", id);
}
#define CENTRAL_ATTACH_POINT "central_thread"
#define DISPLAY_ATTACH_POINT "display_thread"

#endif
