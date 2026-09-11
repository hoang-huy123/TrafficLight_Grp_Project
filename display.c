#include <stdio.h>
#include <stdlib.h>
#include "ipc.h"
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define INTERSECTIONS 6
name_attach_t *attach;

//helper function for displaying light states
const char *light_labels(LightState states) {
	switch(states) {
		case V_GREEN: return "V-GREEN";
		case V_AMBER: return "V-AMBER";
		case V_RED: return "V-RED";
		case H_GREEN: return "H-GREEN";
		case H_AMBER: return "H-AMBER";
		case H_RED: return "H-RED";
		case ALL_RED: return "ALL-RED";
		case PED_CROSS: return "PEDESTRIAN";
		case RAIL_SAFE: return "RAIL-SAFE";
		default: return "UNKNOWN";
	}
}

//helper function for displaying modes
const char *mode_labels(OpMode modes) {
	switch(modes) {
		case MODE_CONGESTION: return "PEAK";
		case MODE_SENSOR: return "SENSOR";
		default: return "UNKNOWN";
	}
}


StatusMsg last[INTERSECTIONS + 1]; //hold recent StatusMsg received
int have_data[INTERSECTIONS + 1] = {0}; //track if that intersection included

//display information in a table
void draw_table(void) {

	printf("\n================Traffic Network================\n");
	printf("%-6s %-10s %-10s %-10s %-6s\n", "No.", "Vertical", "Horizontal", "Mode", "Train"); //in order print intersection number, vertical light, horizontal light, current running mode, and if train arriving
	printf("-----------------------------------------------\n");

	for(int i = 1; i <= INTERSECTIONS; i++) {
		if(!have_data[i]) {
			printf("%-6d %-10s %-10s %-10s %-6s\n", i, "-", "-", "-", "-"); //if that intersection not included then print '-' as placeholders
			continue;
		}

		StatusMsg *m = &last[i];
		printf("%-6d %-10s %-10s %-10s %-6s\n", i, light_labels(m->V_light), light_labels(m->H_light), mode_labels(m->mode), m->train_warning ? "YES" : "NO");
	}
	printf("===============================================\n");
	fflush(stdout);
}

//handle message from central
void handle_message(const Anymsg *msg, AckReply *reply) {
	if(msg->hdr.type == MSG_STATUS_UPDATE) {
		int id = msg->status.intersection_id;
		if (id >= 1 && id <= INTERSECTIONS) {
			last[id] = msg->status;
			have_data[id] = 1;
			draw_table();
			snprintf(reply->buf, REPLY_BUF_SIZE, "Display updated for I%d", id);
		} else {
			snprintf(reply->buf, REPLY_BUF_SIZE, "Ignored status for unknown I%d", id);
		}
	} else {
		snprintf(reply->buf, REPLY_BUF_SIZE, "Unhandled message type %d", msg->hdr.type);
	}
}


//listen to messages sent to display
void *server_thread(void *arg) {
	(void)arg;

	Anymsg msg;
	int rcvid = 0, msgnum = 0;
	int Stay_alive = 1, living = 0;

	living = 1;
	while (living) {

		rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);

		if (rcvid == -1) {
			printf("\nFailed to MsgReceive\n");
			break;
		}

		if (rcvid == 0) {
			switch (msg.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					if (Stay_alive == 0) {
						ConnectDetach(msg.hdr.scoid);
						printf("\n[Display] Server was told to Detach ...\n");
						living = 0;
						continue;
					} else {
						printf("\n[Display] Received Detach pulse but rejected it ...\n");
					}
					break;

				case _PULSE_CODE_UNBLOCK:
					printf("\n[Display] Got _PULSE_CODE_UNBLOCK after %d msgnum\n", msgnum);
					break;

				case _PULSE_CODE_COIDDEATH:
					printf("\n[Display] Got _PULSE_CODE_COIDDEATH after %d msgnum\n", msgnum);
					break;

				case _PULSE_CODE_THREADDEATH:
					printf("\n[Display] Got _PULSE_CODE_THREADDEATH after %d msgnum\n", msgnum);
					break;

				default:
					printf("\n[Display] Got some other pulse after %d msgnum\n", msgnum);
					break;
			}
			continue;
		}

		if (rcvid > 0) {
			msgnum++;

			if (msg.hdr.type == _IO_CONNECT) {
				MsgReply(rcvid, EOK, NULL, 0);
				msgnum--;
				continue;
			}

			if (msg.hdr.type > _IO_BASE && msg.hdr.type <= _IO_MAX) {
				MsgError(rcvid, ENOSYS);
				continue;
			}

			AckReply reply;
			memset(&reply, 0, sizeof(reply));
			reply.hdr.type = 0x01;
			reply.hdr.subtype = 0x00;

			handle_message(&msg, &reply);
			MsgReply(rcvid, EOK, &reply, sizeof(reply));
		} else {
			printf("\n[Display] ERROR: received something, but could not handle it correctly\n");
		}
	}

	name_detach(attach, 0);
	return NULL;
}

int main(void) {

	//attach using name
	if((attach = name_attach(NULL, DISPLAY_ATTACH_POINT, 0)) == NULL) {
		printf("\nFailed to name_attach: %s\n", DISPLAY_ATTACH_POINT);
		printf("\nAnother server with same name maybe running\n");
		return EXIT_FAILURE;
	}

	printf("Display listening on ATTACH_POINT: %s\n", DISPLAY_ATTACH_POINT);
	fflush(stdout);

	server_thread(NULL);

	return 0;
}









