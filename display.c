/*
 * ============================================================================
 * FILE: display.c
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
#include <errno.h>
#include "ipc.h"

#define INTERSECTIONS 6
static name_attach_t *attach;
static StatusMsg last[INTERSECTIONS + 1];
static int have_data[INTERSECTIONS + 1] = {0};

/* SECTION: Convert road-light states into readable terminal labels.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static const char *light_label(LightState s) {
    switch (s) {
        case V_GREEN: return "V-GREEN"; case V_AMBER: return "V-AMBER"; case V_RED: return "V-RED";
        case H_GREEN: return "H-GREEN"; case H_AMBER: return "H-AMBER"; case H_RED: return "H-RED";
        case ALL_RED: return "ALL-RED"; case PED_CROSS: return "PED"; case RAIL_SAFE: return "RAIL-SAFE";
        default: return "UNKNOWN";
    }
}
/* SECTION: Convert controller operating modes into readable terminal labels.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static const char *mode_label(OpMode m) { return m == MODE_CONGESTION ? "PEAK" : m == MODE_SENSOR ? "SENSOR" : "UNKNOWN"; }
/* SECTION: Convert explicit railway states into readable terminal labels.
 * ATTRIBUTION: PROPOSED CONTRIBUTION - TRAN VO VUONG. */
static const char *rail_label(RailState r) {
    switch (r) {
        case RAIL_CLEAR: return "CLEAR"; case RAIL_APPROACHING: return "APPROACH";
        case RAIL_GATE_DOWN_STATE: return "GATE-DOWN"; case RAIL_FAULT_STATE: return "FAULT";
        default: return "UNKNOWN";
    }
}

/* [NEW - VU LUONG MINH TRIET] Render the control-room override state, with the seconds remaining. */
static const char *override_label(const StatusMsg *s, char *buf, size_t n) {
    if (s->override_dir == OVERRIDE_VERTICAL)   snprintf(buf, n, "V %ds", s->override_remaining);
    else if (s->override_dir == OVERRIDE_HORIZONTAL) snprintf(buf, n, "H %ds", s->override_remaining);
    else snprintf(buf, n, "-");
    return buf;
}

/* [PROPOSED CONTRIBUTION - TRAN VO VUONG] Display now exposes railway fault state and pedestrian demand. */
/* SECTION: Render the latest status of all intersections in the terminal.
 * ATTRIBUTION: MIXED - baseline DAM HOANG HUY; railway/pedestrian columns TRAN VO VUONG. */
static void draw_table(void) {
    printf("\n============================= Traffic Network =============================\n");
    printf("%-4s %-10s %-10s %-8s %-11s %-5s %-9s\n",
           "No.", "Vertical", "Horizontal", "Mode", "Railway", "Ped", "Override");
    printf("---------------------------------------------------------------------------\n");
    for (int i = 1; i <= INTERSECTIONS; ++i) {
        if (!have_data[i]) {
            printf("%-4d %-10s %-10s %-8s %-11s %-5s %-9s\n", i,"-","-","-","-","-","-");
            continue;
        }
        StatusMsg *s = &last[i];
        char ovbuf[16];
        printf("%-4d %-10s %-10s %-8s %-11s %-5s %-9s\n", i, light_label(s->V_light), light_label(s->H_light),
               mode_label(s->mode), rail_label(s->rail_state), s->pedestrian_pending ? "WAIT" : "-",
               override_label(s, ovbuf, sizeof(ovbuf)));
    }
    printf("===========================================================================\n");
    fflush(stdout);
}

/* SECTION: Receive status updates from Central and refresh the display cache.
 * ATTRIBUTION: MIXED - baseline DAM HOANG HUY; extended status handling TRAN VO VUONG. */
static void *server_thread(void *arg) {
    (void)arg; Anymsg msg;
    while (1) {
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) { perror("MsgReceive"); break; }
        if (rcvid == 0) continue;
        if (msg.hdr.type == _IO_CONNECT) { MsgReply(rcvid, EOK, NULL, 0); continue; }
        if (msg.hdr.type > _IO_BASE && msg.hdr.type <= _IO_MAX) { MsgError(rcvid, ENOSYS); continue; }
        AckReply reply; memset(&reply, 0, sizeof(reply)); reply.hdr.type = 0x01;
        if (msg.hdr.type == MSG_STATUS_UPDATE) {
            int id = msg.status.intersection_id;
            if (id >= 1 && id <= INTERSECTIONS) { last[id] = msg.status; have_data[id] = 1; draw_table(); }
            snprintf(reply.buf, REPLY_BUF_SIZE, "Display processed I%d", id);
        } else snprintf(reply.buf, REPLY_BUF_SIZE, "Unhandled type %d", msg.hdr.type);
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

/* SECTION: Create the display QNX endpoint and run the display server.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
int main(void) {
    attach = name_attach(NULL, DISPLAY_ATTACH_POINT, 0);
    if (!attach) { fprintf(stderr, "Failed to name_attach %s\n", DISPLAY_ATTACH_POINT); return EXIT_FAILURE; }
    printf("Display listening on %s\n", DISPLAY_ATTACH_POINT);
    /* [NEW - VU LUONG MINH TRIET] Lowest priority. The display has no deadline and produces no
     * control action, so redrawing must never delay a controller. Set
     * explicitly rather than left implicit, to document the decision. */
    rt_set_self_priority(PRIO_DISPLAY, "display");
    server_thread(NULL);
    name_detach(attach, 0);
    return 0;
}
