#include <stdio.h>
#include <stdlib.h>
#include "ipc.h"
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>


#define INTERSECTIONS 6 //number of local controllers

name_attach_t *attach;
int display_coid = -1;

//send commands to local controller
void send_cmd(int intersection_id, CommandMsg cmd) {
	char name[NAME_MAXLEN];
	local_ctrl_name(name, intersection_id);

	int coid = name_open(name, 0);

	if(coid == -1) {
		printf("[Central] send_cmd: could not connect to %s\n", name);
		return;
	}

	AckReply reply;
	if(MsgSend(coid, &cmd, sizeof(cmd), &reply, sizeof(reply)) == -1) {
		printf("[Central] send_cmd: MsgSend to %s failed\n", name);
	}
	name_close(coid);
}

//function for sending command to all local controllers (used for changing sequence pattern)
void broadcast_local(OpMode mode) {
	CommandMsg cmd;
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.type = MSG_COMMAND_SET_MODE;
	cmd.target_mode = mode;
	cmd.hold_green = 0;
	for(int i = 1; i <= INTERSECTIONS; i++) {
		send_cmd(i, cmd);
	}
}

//send data to display thread
void send_to_display(const StatusMsg *msg) {
	if (display_coid == -1) {
		display_coid = name_open(DISPLAY_ATTACH_POINT, 0);
		if (display_coid == -1) {
			printf("[Central] forward_to_display: could not connect to %s\n", DISPLAY_ATTACH_POINT);
			return;
		}
	}

	AckReply reply;
	if(MsgSend(display_coid, msg, sizeof(*msg), &reply, sizeof(reply)) == -1) {
		printf("[Central] forward_to_display: MsgSend failed, dropping connection\n");
		name_close(display_coid);
		display_coid = -1;
	}
}

//listen to message sent to central controller
void *server_thread(void *arg) {
	Anymsg msg;
	int rcvid = 0;
	int msgnum = 0;
	int Stay_alive = 1, living = 0;

	living = 1;
	while(living) {
		rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);

		if(rcvid == -1) {
			printf("\nFailed to receive\n");
			break;
		}

		if (rcvid == 0) {
			switch (msg.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					if (Stay_alive == 0) {
						ConnectDetach(msg.hdr.scoid);
						printf("\n[Central] Server was told to Detach ...\n");
						living = 0;
						continue;
					} else {
						printf("\n[Central] Received Detach pulse but rejected it ...\n");
					}
					break;

				case _PULSE_CODE_UNBLOCK:
					printf("\n[Central] Got _PULSE_CODE_UNBLOCK after %d msgnum\n", msgnum);
					break;

				case _PULSE_CODE_COIDDEATH:
					printf("\n[Central] Got _PULSE_CODE_COIDDEATH after %d msgnum\n", msgnum);
					break;

				case _PULSE_CODE_THREADDEATH:
					printf("\n[Central] Got _PULSE_CODE_THREADDEATH after %d msgnum\n", msgnum);
					break;

				default:
					printf("\n[Central] Got some other pulse after %d msgnum\n", msgnum);
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

			//display status sent from local controller
			if(msg.hdr.type == MSG_STATUS_UPDATE) {
				printf("[Central] I%d mode=%d V=%d H=%d train=%d\n",
				       msg.status.intersection_id, msg.status.mode,		// mode 0 = congestion, mode 1 = sensor
				       msg.status.V_light, msg.status.H_light, msg.status.train_warning);	//v_light and h_light numbers are mapped directly to LightState enum in ipc.h
				fflush(stdout);

				send_to_display(&msg.status); //also send to display for clearer demonstration
				snprintf(reply.buf, REPLY_BUF_SIZE, "Status recorded for I%d", msg.status.intersection_id);

			} else if(msg.hdr.type == MSG_COMMAND_SET_MODE) { //for mode switching request sent from test.c
				printf("[Central] Received mode-switch request -> changing to mode %d\n", msg.command.target_mode);
				fflush(stdout);
				broadcast_local(msg.command.target_mode);
				snprintf(reply.buf, REPLY_BUF_SIZE, "Mode %d broadcast to all intersections", msg.command.target_mode);

			} else {
				snprintf(reply.buf, REPLY_BUF_SIZE, "Unknown message type %d", msg.hdr.type);
			}

			MsgReply(rcvid, EOK, &reply, sizeof(reply));
		} else {
			printf("\n[Central] ERROR: received something, but could not handle it correctly\n");
		}
	}

	name_detach(attach, 0);
	return NULL;

}

int main (void) {

	//attach using name
	if((attach = name_attach(NULL, CENTRAL_ATTACH_POINT, 0)) == NULL) {
		printf("\nFailed to name_attach: %s\n", CENTRAL_ATTACH_POINT);
		printf("\nAnother server with same name maybe running\n");
		return EXIT_FAILURE;
	}

	printf("----Central controller listening on ATTACH_POINT: %s---\n", CENTRAL_ATTACH_POINT);
	fflush(stdout);

	server_thread(NULL);



	return 0;
}
