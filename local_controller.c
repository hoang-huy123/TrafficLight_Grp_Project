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

/* [NEW] Control-room priority override state, protected by the same mutex.
 * override_dir is OVERRIDE_NONE / OVERRIDE_VERTICAL / OVERRIDE_HORIZONTAL.
 * override_seconds always counts down to zero, so an override can never hold a
 * green permanently even if the cancel command is lost or Central goes offline. */
static int override_dir = OVERRIDE_NONE;
static int override_seconds = 0;

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
    int ov_dir = override_dir;       /* [NEW] snapshot under the same lock. */
    int ov_secs = override_seconds;
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
    msg.override_dir = ov_dir;             /* [NEW] make override observable at Central/Display. */
    msg.override_remaining = ov_secs;

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

/* [NEW] Hold a green because the control room asked for a priority path.
 * Counts the override down once per second so it always expires on its own.
 * Railway safety still wins: a train event ends the override hold immediately. */
static bool override_hold(bool is_vertical) {
    const int mine = is_vertical ? OVERRIDE_VERTICAL : OVERRIDE_HORIZONTAL;

    while (1) {
        pthread_mutex_lock(&m);
        int dir = override_dir;
        int remaining = override_seconds;
        if (dir == mine && remaining > 0) override_seconds = remaining - 1;
        if (dir == mine && remaining <= 0) override_dir = OVERRIDE_NONE;
        pthread_mutex_unlock(&m);

        /* Override cancelled, expired, or re-pointed at the other road. */
        if (dir != mine) return true;
        if (remaining <= 0) {
            printf("[I%d] OVERRIDE finished; resuming normal operation.\n", intersection_id);
            fflush(stdout);
            return true;
        }

        sleep(1);
        /* [FIX] Push a status update each second so Central/Display can show the
         * override counting down. Without this the phase never changes during the
         * hold, so no status was sent and Display stayed frozen at the initial
         * value. Same 1 Hz pattern already used by handle_railway_event(). */
        send_status();
        if (railway_stop_required()) return false; /* railway safety has highest priority */
    }
}

/* [ORIGINAL BASELINE - DAM HOANG HUY] Sensor mode checks opposite-road demand periodically.
 * [EXTENDED] Now also serves control-room overrides. With no override active the
 * behaviour is unchanged: MODE_SENSOR still ends the phase at a CHECK_INTERVAL
 * checkpoint when opposing/pedestrian demand exists, and MODE_CONGESTION still
 * runs the full fixed green duration. */
/* SECTION: Run one green phase, honouring override, sensor demand, or fixed timing.
 * ATTRIBUTION: MIXED - sensor baseline DAM HOANG HUY; pedestrian integration TRAN VO VUONG; override handling NEW. */
static bool hold_green(bool is_vertical) {
    const int mine = is_vertical ? OVERRIDE_VERTICAL : OVERRIDE_HORIZONTAL;

    pthread_mutex_lock(&m);
    OpMode mode = current_mode;
    int dir = override_dir;
    pthread_mutex_unlock(&m);

    /* An override for THIS road takes over the whole green phase. */
    if (dir == mine) return override_hold(is_vertical);

    int elapsed = 0;
    int checkpoint = 0;
    const int maximum = green_duration(is_vertical);

    while (elapsed < maximum) {
        sleep(1);
        if (railway_stop_required()) return false;
        ++elapsed;
        ++checkpoint;

        /* [NEW] An override for the OTHER road ends this green early, but still
         * through the normal amber/all-red sequence - never a direct green-to-green. */
        pthread_mutex_lock(&m);
        int now_dir = override_dir;
        pthread_mutex_unlock(&m);
        if (now_dir != OVERRIDE_NONE && now_dir != mine) return true;

        if (mode == MODE_SENSOR && checkpoint >= CHECK_INTERVAL) {
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
    int ov = override_dir;
    /* [NEW] While a control-room priority path is active the pedestrian request is
     * DEFERRED, not discarded: the flag stays set and is served at the first
     * all-red once the override expires. Design assumption to justify in the
     * report: an emergency/dignitary path must not be interrupted mid-run. */
    if (pending && ov == OVERRIDE_NONE) pedestrian_pending = false;
    pthread_mutex_unlock(&m);

    if (ov != OVERRIDE_NONE) return true;   /* skip this cycle, keep the request queued */
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
                car_waiting_V = false; /* [PROPOSED CONTRIBUTION - TRAN VO VUONG] current green serves queued demand. */
                pthread_mutex_unlock(&m);
                /* [CHANGED] hold_green() now covers override, sensor and fixed-timing
                 * modes in one place; behaviour without an override is unchanged. */
                if (hold_green(true)) state = V_AMBER;
                break;
            }
            case V_AMBER:
                set_phase(V_AMBER);
                if (interruptible_sleep(AMBER_DURATION)) state = ALL_RED;
                break;

            case ALL_RED: {
                set_phase(ALL_RED);
                if (!interruptible_sleep(ALL_RED_DURATION)) break;
                if (!serve_pedestrian_if_needed()) break;

                /* [NEW] An active override decides which road goes green next.
                 * The normal alternation resumes automatically once it expires. */
                pthread_mutex_lock(&m);
                int ov = override_dir;
                pthread_mutex_unlock(&m);

                if (ov == OVERRIDE_VERTICAL)        { state = V_GREEN; horizontal_next = 1; }
                else if (ov == OVERRIDE_HORIZONTAL) { state = H_GREEN; horizontal_next = 0; }
                else if (horizontal_next)           { state = H_GREEN; horizontal_next = 0; }
                else                                { state = V_GREEN; horizontal_next = 1; }
                break;
            }

            case H_GREEN: {
                set_phase(H_GREEN);
                pthread_mutex_lock(&m);
                car_waiting_H = false;
                pthread_mutex_unlock(&m);
                /* [CHANGED] See V_GREEN above. */
                if (hold_green(false)) state = H_AMBER;
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
        /* [FIX - deadlock] Do NOT call send_status() here: this handler runs while the
         * sender (e.g. Central's broadcast_local) is still blocked inside MsgSend()
         * waiting for THIS reply. If we call send_status() (which itself blocks on
         * MsgSend to Central) before replying, Central can never receive it because
         * Central hasn't returned to MsgReceive() yet -> circular deadlock.
         * server_thread() now sends the status update AFTER MsgReply() completes. */
        return;
    }

    /* [NEW] Control-room priority override. */
    if (msg->hdr.type == MSG_COMMAND_OVERRIDE) {
        int dir = msg->command.hold_green;
        int secs = msg->command.hold_seconds;

        if (dir != OVERRIDE_NONE && dir != OVERRIDE_VERTICAL && dir != OVERRIDE_HORIZONTAL) {
            snprintf(reply->buf, REPLY_BUF_SIZE, "Invalid override direction %d", dir);
            return;
        }
        /* Clamp so a bad command can never hold a green forever. */
        if (secs <= 0) secs = OVERRIDE_DEFAULT_SECONDS;
        if (secs > OVERRIDE_MAX_SECONDS) secs = OVERRIDE_MAX_SECONDS;

        pthread_mutex_lock(&m);
        override_dir = dir;
        override_seconds = (dir == OVERRIDE_NONE) ? 0 : secs;
        pthread_mutex_unlock(&m);

        if (dir == OVERRIDE_NONE) {
            snprintf(reply->buf, REPLY_BUF_SIZE, "Override cancelled");
            printf("[I%d] OVERRIDE cancelled by control room.\n", intersection_id);
        } else {
            const char *road = (dir == OVERRIDE_VERTICAL) ? "VERTICAL" : "HORIZONTAL";
            snprintf(reply->buf, REPLY_BUF_SIZE, "Override %s green %ds", road, secs);
            printf("[I%d] OVERRIDE requested: hold %s green for %ds "
                   "(applies at the next safe phase change).\n", intersection_id, road, secs);
        }
        fflush(stdout);
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
        /* [FIX - deadlock] See note above: status push moved to after MsgReply(). */
        return;
    }

    snprintf(reply->buf, REPLY_BUF_SIZE, "Unknown message type %d", msg->hdr.type);
}

/* SECTION: QNX receive/reply loop that lets IPC events run concurrently with the local state machine.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static void *server_thread(void *arg) {
    (void)arg;
    /* [NEW] Highest priority in the system: railway and sensor events must be
     * received and acted on ahead of all other work at this intersection. */
    rt_set_self_priority(PRIO_LOCAL_EVENT, "local event intake");
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
        /* [FIX - deadlock] Push the status update only after the reply has been sent,
         * so we never hold a sender (Central or test) blocked while we try to reach
         * Central ourselves. See handle_message() for the full explanation. */
        if (msg.hdr.type == MSG_COMMAND_SET_MODE || msg.hdr.type == MSG_SENSOR_EVENT ||
            msg.hdr.type == MSG_COMMAND_OVERRIDE) {
            send_status();
        }
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

    /* [NEW] This thread runs the light state machine, which enforces amber and
     * clearance timing. It is time-critical, so it is raised well above the
     * default, but kept below the event-intake thread created next. */
    rt_set_self_priority(PRIO_LOCAL_CONTROL, "local state machine");

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
