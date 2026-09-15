/*
 * ============================================================================
 * FILE: local_controller.c
 * OWNERSHIP NOTE: MIXED: baseline by DAM HOANG HUY; proposed improvements by TRAN VO VUONG
 *
 * Attribution in this file is based on comparison with the preserved baseline
 * in original_source/. "Proposed contribution" means code added/changed in the
 * improved version and should only be claimed after review, testing, and actual
 * contribution by the named team member.
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include "ipc.h"
#include "railway_controller.h"
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

/* [ORIGINAL BASELINE - DAM HOANG HUY] Timing structure retained; values should be justified in the Implementation Note. */
#define AMBER_DURATION 4
#define ALL_RED_DURATION 2
#define CLEARANCE_DURATION 10
#define CHECK_INTERVAL 10
/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Explicit pedestrian crossing interval. */
#define PEDESTRIAN_DURATION 8

const int GREEN_DURATION_MODES[2][2] = {
    [MODE_CONGESTION] = {30, 20},
    [MODE_SENSOR] = {30, 30}
};

/* [ORIGINAL BASELINE - DAM HOANG HUY] Shared road-controller state protected by one mutex. */
static pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
static LightState current_state = V_GREEN;
static OpMode current_mode = MODE_SENSOR;
static int intersection_id = 1;
static bool car_waiting_V = false;
static bool car_waiting_H = false;

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Pedestrian demand is now implemented rather than left as a placeholder. */
static bool pedestrian_pending = false;

static name_attach_t *attach;
static int central_coid = -1;

/* SECTION: Read the configured green duration for the current mode and direction.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static int green_duration(bool is_vertical) {
    pthread_mutex_lock(&m);
    OpMode mode = current_mode;
    pthread_mutex_unlock(&m);
    return GREEN_DURATION_MODES[mode][is_vertical ? 0 : 1];
}

/* [ORIGINAL BASELINE - DAM HOANG HUY] Mapping from controller phase to each road's displayed light. */
/* SECTION: Map the shared controller phase to the light shown on each road direction.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static LightState get_state(LightState state, bool is_vertical) {
    switch (state) {
        case V_GREEN: return is_vertical ? V_GREEN : H_RED;
        case V_AMBER: return is_vertical ? V_AMBER : H_RED;
        case H_GREEN: return is_vertical ? V_RED : H_GREEN;
        case H_AMBER: return is_vertical ? V_RED : H_AMBER;
        case PED_CROSS:
        case ALL_RED:
        case RAIL_SAFE:
        default: return is_vertical ? V_RED : H_RED;
    }
}

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Snapshot shared state before IPC so the road mutex is never held across MsgSend(). */
/* SECTION: Snapshot local state and report it to Central without stopping autonomous control if IPC fails.
 * ATTRIBUTION: MIXED - baseline DAM HOANG HUY; safer snapshot/failure diagnostics TRAN VO VUONG. */
static void send_status(void) {
    pthread_mutex_lock(&m);
    LightState state = current_state;
    OpMode mode = current_mode;
    bool ped = pedestrian_pending;
    pthread_mutex_unlock(&m);

    RailState rail = railway_get_state();

    if (central_coid == -1)
        central_coid = name_open(CENTRAL_ATTACH_POINT, 0);

    /* [ORIGINAL BEHAVIOUR] Local operation continues if Central is unavailable. */
    if (central_coid == -1) return;

    StatusMsg msg;
    memset(&msg, 0, sizeof(msg));
    msg.hdr.type = MSG_STATUS_UPDATE;
    msg.intersection_id = intersection_id;
    msg.mode = mode;
    msg.V_light = get_state(state, true);
    msg.H_light = get_state(state, false);
    msg.train_warning = (rail != RAIL_CLEAR);
    msg.rail_state = rail;
    msg.pedestrian_pending = ped ? 1 : 0;

    AckReply reply;
    if (MsgSend(central_coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
        name_close(central_coid);
        central_coid = -1;
        /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Explicit diagnostic demonstrates fail-safe local independence. */
        printf("[I%d] Central unavailable; continuing autonomous control.\n", intersection_id);
        fflush(stdout);
    }
}

/* SECTION: Atomically change the local traffic phase and publish the new status.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static void set_phase(LightState next_state) {
    pthread_mutex_lock(&m);
    current_state = next_state;
    pthread_mutex_unlock(&m);
    printf("[I%d] phase -> %d\n", intersection_id, next_state);
    fflush(stdout);
    send_status();
}

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Railway module replaces the old train_arriving boolean. */
/* SECTION: Query the railway module to decide whether road traffic must stop.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
static bool railway_stop_required(void) {
    return railway_requires_road_stop();
}

/* [ORIGINAL IDEA, IMPROVED NAME] Sleep can be interrupted by railway safety events. */
/* SECTION: Wait in short intervals so safety events can interrupt a normal traffic phase.
 * ATTRIBUTION: MIXED - baseline timing idea DAM HOANG HUY; extended safety checks TRAN VO VUONG. */
static bool interruptible_sleep(int seconds) {
    for (int i = 0; i < seconds; ++i) {
        sleep(1);
        if (railway_stop_required()) return false;
    }
    return true;
}

/* [ORIGINAL BASELINE - DAM HOANG HUY] Sensor mode checks opposite-road demand periodically. */
/* SECTION: Implement sensor-driven green holding and end the phase when opposing/pedestrian demand requires service.
 * ATTRIBUTION: MIXED - sensor baseline DAM HOANG HUY; pedestrian integration TRAN VO VUONG. */
static bool hold_green(bool is_vertical) {
    int elapsed = 0;
    int checkpoint = 0;
    const int maximum = green_duration(is_vertical);

    while (elapsed < maximum) {
        sleep(1);
        if (railway_stop_required()) return false;
        ++elapsed;
        ++checkpoint;

        if (checkpoint >= CHECK_INTERVAL) {
            pthread_mutex_lock(&m);
            bool other_waiting = is_vertical ? car_waiting_H : car_waiting_V;
            bool ped_waiting = pedestrian_pending;
            pthread_mutex_unlock(&m);
            checkpoint = 0;

            /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] A pedestrian request can also end a green phase at a safe checkpoint. */
            if (other_waiting || ped_waiting) return true;
        }
    }
    return true;
}

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Pedestrians are served only when both road directions are red. */
/* SECTION: Serve a pending pedestrian request only while both vehicle directions are safely stopped.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
static bool serve_pedestrian_if_needed(void) {
    pthread_mutex_lock(&m);
    bool pending = pedestrian_pending;
    if (pending) pedestrian_pending = false;
    pthread_mutex_unlock(&m);

    if (!pending) return true;

    set_phase(PED_CROSS);
    for (int i = 0; i < PEDESTRIAN_DURATION; ++i) {
        sleep(1);
        if (railway_stop_required()) return false; /* railway safety has highest priority */
    }
    set_phase(ALL_RED);
    return true;
}

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Railway handling is isolated and explicitly fail-safe. */
/* SECTION: Move the intersection into a fail-safe railway state and resume traffic after clearance.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG, based on the original train-handling concept by DAM HOANG HUY. */
static void handle_railway_event(LightState state, int horizontal_next) {
    if (state == V_GREEN) {
        /* Existing design assumption: clear the railway-adjacent vertical road before stopping. */
        sleep(CLEARANCE_DURATION);
        set_phase(V_AMBER);
        sleep(AMBER_DURATION);
    } else if (state == H_GREEN) {
        set_phase(H_AMBER);
        sleep(AMBER_DURATION);
    } else if (state == ALL_RED && horizontal_next) {
        set_phase(V_GREEN);
        sleep(CLEARANCE_DURATION);
        set_phase(V_AMBER);
        sleep(AMBER_DURATION);
    }

    set_phase(RAIL_SAFE);
    if (railway_has_fault()) {
        printf("[I%d] RAILWAY FAULT: roads held safe; fault reported to Central when available.\n", intersection_id);
        fflush(stdout);
    }

    while (railway_stop_required()) {
        send_status();
        sleep(1);
    }
    set_phase(ALL_RED);
    sleep(ALL_RED_DURATION);
}

/* SECTION: Main local traffic-light state machine for normal, sensor, pedestrian, and railway operation.
 * ATTRIBUTION: MIXED - original road state machine DAM HOANG HUY; pedestrian/railway improvements TRAN VO VUONG. */
static void traffic_light_state(void) {
    LightState state = V_GREEN;
    int horizontal_next = 1;

    while (1) {
        if (railway_stop_required()) {
            handle_railway_event(state, horizontal_next);
            state = V_GREEN; /* design assumption: restart with vertical road */
            continue;
        }

        switch (state) {
            case V_GREEN: {
                set_phase(V_GREEN);
                pthread_mutex_lock(&m);
                OpMode mode = current_mode;
                car_waiting_V = false; /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] current green serves queued demand. */
                pthread_mutex_unlock(&m);
                bool completed = (mode == MODE_SENSOR) ? hold_green(true)
                                                       : interruptible_sleep(green_duration(true));
                if (completed) state = V_AMBER;
                break;
            }
            case V_AMBER:
                set_phase(V_AMBER);
                if (interruptible_sleep(AMBER_DURATION)) state = ALL_RED;
                break;

            case ALL_RED:
                set_phase(ALL_RED);
                if (!interruptible_sleep(ALL_RED_DURATION)) break;
                if (!serve_pedestrian_if_needed()) break;
                if (horizontal_next) { state = H_GREEN; horizontal_next = 0; }
                else { state = V_GREEN; horizontal_next = 1; }
                break;

            case H_GREEN: {
                set_phase(H_GREEN);
                pthread_mutex_lock(&m);
                OpMode mode = current_mode;
                car_waiting_H = false;
                pthread_mutex_unlock(&m);
                bool completed = (mode == MODE_SENSOR) ? hold_green(false)
                                                       : interruptible_sleep(green_duration(false));
                if (completed) state = H_AMBER;
                break;
            }
            case H_AMBER:
                set_phase(H_AMBER);
                if (interruptible_sleep(AMBER_DURATION)) state = ALL_RED;
                break;

            case PED_CROSS:
            case RAIL_SAFE:
            default:
                state = ALL_RED;
                break;
        }
    }
}

/* SECTION: Apply Central commands and simulated sensor events to thread-safe local state.
 * ATTRIBUTION: MIXED - baseline message handling DAM HOANG HUY; pedestrian/railway events TRAN VO VUONG. */
static void handle_message(const Anymsg *msg, AckReply *reply) {
    if (msg->hdr.type == MSG_COMMAND_SET_MODE) {
        if (msg->command.target_mode != MODE_CONGESTION && msg->command.target_mode != MODE_SENSOR) {
            snprintf(reply->buf, REPLY_BUF_SIZE, "Invalid mode %d", msg->command.target_mode);
            return;
        }
        pthread_mutex_lock(&m);
        current_mode = msg->command.target_mode;
        pthread_mutex_unlock(&m);
        snprintf(reply->buf, REPLY_BUF_SIZE, "Mode changed to %d", msg->command.target_mode);
        send_status();
        return;
    }

    if (msg->hdr.type == MSG_SENSOR_EVENT) {
        SensorEvent event = msg->sensor.event;

        /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Railway events are handled by the new railway module. */
        if (event == TRAIN_GATE_DOWN || event == TRAIN_GATE_CLEAR ||
            event == TRAIN_FAULT || event == TRAIN_APPROACHING) {
            railway_handle_event(event);
        } else {
            pthread_mutex_lock(&m);
            if (event == CAR_DETECTED_V) car_waiting_V = true;
            else if (event == CAR_DETECTED_H) car_waiting_H = true;
            else if (event == PEDESTRIAN_PRESSED) pedestrian_pending = true;
            pthread_mutex_unlock(&m);
        }

        snprintf(reply->buf, REPLY_BUF_SIZE, "Sensor event %d processed", event);
        send_status();
        return;
    }

    snprintf(reply->buf, REPLY_BUF_SIZE, "Unknown message type %d", msg->hdr.type);
}

/* SECTION: QNX receive/reply loop that lets IPC events run concurrently with the local state machine.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static void *server_thread(void *arg) {
    (void)arg;
    Anymsg msg;
    while (1) {
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) { perror("MsgReceive"); break; }
        if (rcvid == 0) continue; /* QNX pulse; no shutdown protocol required for this PoC. */

        if (msg.hdr.type == _IO_CONNECT) { MsgReply(rcvid, EOK, NULL, 0); continue; }
        if (msg.hdr.type > _IO_BASE && msg.hdr.type <= _IO_MAX) { MsgError(rcvid, ENOSYS); continue; }

        AckReply reply;
        memset(&reply, 0, sizeof(reply));
        reply.hdr.type = 0x01;
        handle_message(&msg, &reply);
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

/* SECTION: Initialise one intersection, start its QNX server thread, and run autonomous control.
 * ATTRIBUTION: MIXED - baseline startup DAM HOANG HUY; railway module initialisation TRAN VO VUONG. */
int main(int argc, char *argv[]) {
    if (argc > 1) intersection_id = atoi(argv[1]);
    if (intersection_id < 1 || intersection_id > 6) {
        fprintf(stderr, "Intersection ID must be 1..6\n");
        return EXIT_FAILURE;
    }

    railway_init();
    char name[NAME_MAXLEN];
    local_ctrl_name(name, intersection_id);
    attach = name_attach(NULL, name, 0);
    if (!attach) {
        fprintf(stderr, "Failed to name_attach %s\n", name);
        return EXIT_FAILURE;
    }

    printf("I%d local controller listening on %s\n", intersection_id, name);
    pthread_t server;
    if (pthread_create(&server, NULL, server_thread, NULL) != 0) {
        perror("pthread_create");
        return EXIT_FAILURE;
    }

    send_status();
    traffic_light_state();
    pthread_join(server, NULL);
    name_detach(attach, 0);
    return 0;
}
