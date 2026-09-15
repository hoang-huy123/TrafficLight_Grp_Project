/*
 * ============================================================================
 * FILE: central_controller.c
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
static int display_coid = -1;

/* [ORIGINAL BASELINE - DAM HOANG HUY] Central sends high-level mode commands; locals still own light sequencing. */
/* SECTION: Send a high-level operating-mode command to one local controller.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static void send_cmd(int intersection_id, const CommandMsg *cmd) {
    char name[NAME_MAXLEN];
    local_ctrl_name(name, intersection_id);
    int coid = name_open(name, 0);
    if (coid == -1) {
        printf("[Central] I%d offline; command skipped.\n", intersection_id);
        return;
    }
    AckReply reply;
    if (MsgSend(coid, cmd, sizeof(*cmd), &reply, sizeof(reply)) == -1)
        printf("[Central] command to I%d failed.\n", intersection_id);
    name_close(coid);
}

/* SECTION: Broadcast a selected operating mode to all six local controllers.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
static void broadcast_local(OpMode mode) {
    CommandMsg cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.hdr.type = MSG_COMMAND_SET_MODE;
    cmd.target_mode = mode;
    for (int i = 1; i <= INTERSECTIONS; ++i) send_cmd(i, &cmd);
}

/* [ORIGINAL + IMPROVEMENT] Cached display connection; failure is non-fatal. */
/* SECTION: Forward the latest local-controller status to the monitoring display.
 * ATTRIBUTION: MIXED - baseline DAM HOANG HUY; improved status payload integration TRAN VO VUONG. */
static void send_to_display(const StatusMsg *msg) {
    if (display_coid == -1) display_coid = name_open(DISPLAY_ATTACH_POINT, 0);
    if (display_coid == -1) return;
    AckReply reply;
    if (MsgSend(display_coid, msg, sizeof(*msg), &reply, sizeof(reply)) == -1) {
        name_close(display_coid);
        display_coid = -1;
    }
}

/* SECTION: Central QNX receive/reply loop for status updates and operator mode commands.
 * ATTRIBUTION: MIXED - baseline DAM HOANG HUY; railway/fault status integration TRAN VO VUONG. */
static void *server_thread(void *arg) {
    (void)arg;
    Anymsg msg;
    while (1) {
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) { perror("MsgReceive"); break; }
        if (rcvid == 0) continue;
        if (msg.hdr.type == _IO_CONNECT) { MsgReply(rcvid, EOK, NULL, 0); continue; }
        if (msg.hdr.type > _IO_BASE && msg.hdr.type <= _IO_MAX) { MsgError(rcvid, ENOSYS); continue; }

        AckReply reply;
        memset(&reply, 0, sizeof(reply));
        reply.hdr.type = 0x01;

        if (msg.hdr.type == MSG_STATUS_UPDATE) {
            printf("[Central] I%d mode=%d V=%d H=%d rail=%d ped=%d\n",
                   msg.status.intersection_id, msg.status.mode,
                   msg.status.V_light, msg.status.H_light,
                   msg.status.rail_state, msg.status.pedestrian_pending);
            if (msg.status.rail_state == RAIL_FAULT_STATE)
                printf("[Central] *** RAILWAY FAULT reported by I%d ***\n", msg.status.intersection_id);
            fflush(stdout);
            send_to_display(&msg.status);
            snprintf(reply.buf, REPLY_BUF_SIZE, "Status recorded for I%d", msg.status.intersection_id);
        } else if (msg.hdr.type == MSG_COMMAND_SET_MODE) {
            if (msg.command.target_mode != MODE_CONGESTION && msg.command.target_mode != MODE_SENSOR) {
                snprintf(reply.buf, REPLY_BUF_SIZE, "Invalid mode");
            } else {
                broadcast_local(msg.command.target_mode);
                snprintf(reply.buf, REPLY_BUF_SIZE, "Mode %d broadcast", msg.command.target_mode);
            }
        } else {
            snprintf(reply.buf, REPLY_BUF_SIZE, "Unknown message type %d", msg.hdr.type);
        }
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }
    return NULL;
}

/* SECTION: Create the central QNX name attachment and start the central server.
 * ATTRIBUTION: ORIGINAL BASELINE - DAM HOANG HUY. */
int main(void) {
    attach = name_attach(NULL, CENTRAL_ATTACH_POINT, 0);
    if (!attach) {
        fprintf(stderr, "Failed to name_attach %s\n", CENTRAL_ATTACH_POINT);
        return EXIT_FAILURE;
    }
    printf("Central controller listening on %s\n", CENTRAL_ATTACH_POINT);
    server_thread(NULL);
    if (display_coid != -1) name_close(display_coid);
    name_detach(attach, 0);
    return 0;
}
