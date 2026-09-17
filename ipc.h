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
#include <pthread.h>
#include <sched.h>
#include <string.h>

/* ==========================================================================
 * [NEW - VU LUONG MINH TRIET] Real-time scheduling policy and thread priority assignment.
 *
 * Rationale (Gomaa Task Priority Criteria, Lecture 6): time-critical tasks
 * with hard deadlines are given a high priority and kept as separate tasks;
 * non-time-critical tasks are given a low priority so they cannot starve the
 * critical ones. QNX offers 255 priority levels; the default is 10, and a
 * higher number means a higher priority.
 *
 *   21  Local event intake     - railway/sensor events must preempt everything
 *                                else at an intersection. A train event that
 *                                waits behind other work is a safety failure.
 *   19  Local state machine    - enforces amber and clearance timing. Late
 *                                transitions are a safety failure, so it sits
 *                                just below event intake.
 *   14  Central controller     - supervisory only. It never drives a light
 *                                directly, and the locals are designed to keep
 *                                running without it, so it must not compete
 *                                with intersection control.
 *   10  Display                - output only, no deadline. Left at the default
 *                                so redrawing can never delay a controller.
 *   10  Test simulator         - not part of the deployed system.
 *
 * Policy: SCHED_RR. Our threads are not CPU-bound (they block on timed waits
 * and MsgReceive), so round-robin costs nothing and protects against a same-
 * priority thread monopolising the CPU. SCHED_FIFO is the alternative if
 * strictly run-to-block behaviour is preferred.
 * ========================================================================== */
#define RT_SCHED_POLICY     SCHED_RR

#define PRIO_LOCAL_EVENT    21
#define PRIO_LOCAL_CONTROL  19
#define PRIO_CENTRAL        14
#define PRIO_DISPLAY        10
#define PRIO_TEST           10

/* Raise (or set) the calling thread's scheduling priority. Failure is not
 * fatal: the system still runs correctly at the default priority, it simply
 * loses the timing guarantee, so we warn and continue rather than abort. */
static inline int rt_set_self_priority(int priority, const char *label) {
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = priority;

    int rc = pthread_setschedparam(pthread_self(), RT_SCHED_POLICY, &param);
    if (rc != 0) {
        fprintf(stderr, "[rt] WARNING: could not set %s priority to %d (%s); "
                        "continuing at default priority.\n",
                label, priority, strerror(rc));
        return -1;
    }
    printf("[rt] %s at priority %d (SCHED_RR)\n", label, priority);
    fflush(stdout);
    return 0;
}

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
    /* [NEW - VU LUONG MINH TRIET] Control-room priority override (e.g. clear path for an emergency
     * vehicle or visiting dignitary), as required by the project brief. */
    MSG_COMMAND_OVERRIDE
} MsgType;

/* [NEW - VU LUONG MINH TRIET] Override directions carried in CommandMsg.hold_green. */
#define OVERRIDE_NONE        0   /* cancel any active override        */
#define OVERRIDE_VERTICAL    1   /* hold the vertical road green      */
#define OVERRIDE_HORIZONTAL  2   /* hold the horizontal road green    */

/* [NEW - VU LUONG MINH TRIET] Bounds applied by the local controller so a bad or malicious command
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
    int override_dir;        /* [NEW - VU LUONG MINH TRIET] OVERRIDE_* currently active at this intersection. */
    int override_remaining;  /* [NEW - VU LUONG MINH TRIET] seconds left on the active override (0 if none). */
} StatusMsg;

typedef struct {
    msg_header_t hdr;
    OpMode target_mode;
    /* [WAS DEAD CODE - NOW IMPLEMENTED - VU LUONG MINH TRIET] hold_green was previously declared but
     * never read by any controller. It now carries the override direction
     * (OVERRIDE_NONE / OVERRIDE_VERTICAL / OVERRIDE_HORIZONTAL). */
    int hold_green;
    int hold_seconds;        /* [NEW - VU LUONG MINH TRIET] requested override duration in seconds. */
    int target_intersection; /* [NEW - VU LUONG MINH TRIET] 0 = all intersections, 1..6 = one only. */
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
