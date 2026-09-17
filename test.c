/*
 * ============================================================================
 * FILE: test.c
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
#include <string.h>
#include "ipc.h"

/* SECTION: Connect the simulator to a selected local intersection controller.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static int connect_to(int intersection_id) {
    char name[NAME_MAXLEN]; local_ctrl_name(name, intersection_id);
    int coid = name_open(name, 0);
    if (coid == -1) printf("Could not connect to I%d\n", intersection_id);
    return coid;
}

/* SECTION: Inject a simulated sensor event into a local controller using QNX IPC.
 * ATTRIBUTION: MIXED - baseline DAM HOANG HUY; expanded event set TRAN VO VUONG. */
static void send_sensor(int id, SensorEvent event, const char *label) {
    int coid = connect_to(id); if (coid == -1) return;
    SensorMsg msg; memset(&msg, 0, sizeof(msg)); msg.hdr.type = MSG_SENSOR_EVENT; msg.event = event;
    AckReply reply;
    if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) printf("Send failed\n");
    else printf("%s -> I%d (%s)\n", label, id, reply.buf);
    name_close(coid);
}

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG | BUG FIX] Original function had an unused intersection_id parameter but calls supplied only mode. */
/* SECTION: Send an operator mode-change request to Central for network-wide broadcast.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG (bug fix to original DAM HOANG HUY interface). */
static void send_mode(OpMode mode) {
    int coid = name_open(CENTRAL_ATTACH_POINT, 0);
    if (coid == -1) { printf("Could not connect to Central\n"); return; }
    CommandMsg cmd; memset(&cmd, 0, sizeof(cmd)); cmd.hdr.type = MSG_COMMAND_SET_MODE; cmd.target_mode = mode;
    AckReply reply;
    if (MsgSend(coid, &cmd, sizeof(cmd), &reply, sizeof(reply)) == -1) printf("Send failed\n");
    else printf("Mode %d -> %s\n", mode, reply.buf);
    name_close(coid);
}

/* [NEW] Ask Central to issue a control-room priority override. target 0 = all
 * intersections (corridor-wide green path), 1..6 = a single intersection. */
static void send_override(int dir, int seconds, int target) {
    int coid = name_open(CENTRAL_ATTACH_POINT, 0);
    if (coid == -1) { printf("Could not connect to Central\n"); return; }
    CommandMsg cmd; memset(&cmd, 0, sizeof(cmd));
    cmd.hdr.type = MSG_COMMAND_OVERRIDE;
    cmd.hold_green = dir;
    cmd.hold_seconds = seconds;
    cmd.target_intersection = target;
    AckReply reply;
    if (MsgSend(coid, &cmd, sizeof(cmd), &reply, sizeof(reply)) == -1) printf("Send failed\n");
    else printf("Override -> %s\n", reply.buf);
    name_close(coid);
}

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Menu covers the scenarios required for demonstration/testing. */
/* SECTION: Present demonstration controls for traffic, pedestrian, railway, fault, and mode scenarios.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG, expanded from the original DAM HOANG HUY menu. */
static void print_menu(void) {
    printf("\n--- EVENT / TEST MENU ---\n"
           " 1) Car detected - Vertical\n"
           " 2) Car detected - Horizontal\n"
           " 3) Pedestrian button\n"
           " 4) Train approaching\n"
           " 5) Train gate down\n"
           " 6) Train / gate clear\n"
           " 7) Railway fault\n"
           " 8) Set mode: CONGESTION\n"
           " 9) Set mode: SENSOR\n"
           "10) Override: hold VERTICAL green\n"
           "11) Override: hold HORIZONTAL green\n"
           "12) Override: CANCEL\n"
           " 0) Quit\nChoice: ");
    fflush(stdout);
}

/* SECTION: Identify simulator commands that target a specific intersection.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
static int needs_id(int choice) { return choice >= 1 && choice <= 7; }

/* SECTION: Interactive event-simulator loop used for testing and project demonstration.
 * ATTRIBUTION: MIXED - baseline DAM HOANG HUY; expanded scenarios TRAN VO VUONG. */
int main(void) {
    char line[32];
    while (1) {
        print_menu(); if (!fgets(line, sizeof(line), stdin)) break;
        int choice = atoi(line); if (choice == 0) break;
        int id = 0;
        if (needs_id(choice)) {
            printf("Intersection ID (1-6): "); fflush(stdout);
            if (!fgets(line, sizeof(line), stdin)) break;
            id = atoi(line); if (id < 1 || id > 6) { printf("Invalid ID\n"); continue; }
        }
        switch (choice) {
            case 1: send_sensor(id, CAR_DETECTED_V, "Vertical car"); break;
            case 2: send_sensor(id, CAR_DETECTED_H, "Horizontal car"); break;
            case 3: send_sensor(id, PEDESTRIAN_PRESSED, "Pedestrian request"); break;
            case 4: send_sensor(id, TRAIN_APPROACHING, "Train approaching"); break;
            case 5: send_sensor(id, TRAIN_GATE_DOWN, "Gate down"); break;
            case 6: send_sensor(id, TRAIN_GATE_CLEAR, "Rail clear"); break;
            case 7: send_sensor(id, TRAIN_FAULT, "Railway fault"); break;
            case 8: send_mode(MODE_CONGESTION); break;
            case 9: send_mode(MODE_SENSOR); break;
            /* [NEW] Control-room override scenarios. */
            case 10:
            case 11: {
                int dir = (choice == 10) ? OVERRIDE_VERTICAL : OVERRIDE_HORIZONTAL;
                printf("Target intersection (0 = all, 1-6 = one): "); fflush(stdout);
                if (!fgets(line, sizeof(line), stdin)) return 0;
                int target = atoi(line);
                if (target < 0 || target > 6) { printf("Invalid target\n"); break; }
                printf("Hold seconds (blank = %d, max %d): ", OVERRIDE_DEFAULT_SECONDS, OVERRIDE_MAX_SECONDS);
                fflush(stdout);
                if (!fgets(line, sizeof(line), stdin)) return 0;
                int secs = atoi(line);            /* 0/blank -> local controller uses the default */
                send_override(dir, secs, target);
                break;
            }
            case 12: {
                printf("Target intersection (0 = all, 1-6 = one): "); fflush(stdout);
                if (!fgets(line, sizeof(line), stdin)) return 0;
                int target = atoi(line);
                if (target < 0 || target > 6) { printf("Invalid target\n"); break; }
                send_override(OVERRIDE_NONE, 0, target);
                break;
            }
            default: printf("Unknown choice\n"); break;
        }
    }
    return 0;
}
