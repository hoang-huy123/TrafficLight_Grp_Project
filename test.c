
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ipc.h"

//connect to a local controller
int connect_to(int intersection_id) {
	char name[NAME_MAXLEN];
	local_ctrl_name(name, intersection_id);
	int coid = name_open(name, 0);
	if (coid == -1) printf("Could not connect to I%d\n", intersection_id);
	return coid;
}

//send sensor event for car or pedestrian
void send_sensor(int intersection_id, SensorEvent event, const char *label) {
	int coid = connect_to(intersection_id);
	if (coid == -1) return;

	SensorMsg msg;
	memset(&msg, 0, sizeof(msg));
	msg.hdr.type = MSG_SENSOR_EVENT;
	msg.event = event;

	AckReply reply;
	if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
		printf("Send failed\n");

	} else {
		printf("%s -> I%d\n", label, intersection_id);

	}
	name_close(coid);
}

//send operating mode
void send_mode(int intersection_id, OpMode mode) {
	int coid = name_open(CENTRAL_ATTACH_POINT, 0);
	if (coid == -1) {
		printf("Could not connect to central\n");
		return;
	}

	CommandMsg cmd;
	memset(&cmd, 0, sizeof(cmd));

	cmd.hdr.type = MSG_COMMAND_SET_MODE;
	cmd.target_mode = mode;

	AckReply reply;
	if (MsgSend(coid, &cmd, sizeof(cmd), &reply, sizeof(reply)) == -1) {
		printf("Send failed\n");

	} else {
		printf("Mode %d -> %s\n", mode, reply.buf);

	}
	name_close(coid);
}

//helper function to print menu
void print_menu(void) {
	printf("\n--- TEST MENU ---\n");
	printf(" 1) Car detected - Vertical\n");
	printf(" 2) Car detected - Horizontal\n");
	printf(" 3) Set mode: CONGESTION\n");
	printf(" 4) Set mode: SENSOR\n");
	printf(" 0) Quit\n");
	printf("Choice: ");
	fflush(stdout);
}


int main(void) {
	int choice, id;
	char line[32];

	while (1) {
		print_menu();
		if (!fgets(line, sizeof(line), stdin)) break;
		choice = atoi(line);
		if (choice == 0) break;

		if (choice == 1 || choice == 2) { //simulate car sensor
			printf("Intersection ID: ");
			fflush(stdout);
			if (!fgets(line, sizeof(line), stdin)) break;
			id = atoi(line);
		} else if (choice != 3 && choice != 4) { //mode switching
			continue;
		}

		switch (choice) {
			case 1: send_sensor(id, CAR_DETECTED_V, "Vertical car"); break;
			case 2: send_sensor(id, CAR_DETECTED_H, "Horizontal car"); break;
			case 3: send_mode(MODE_CONGESTION); break;
			case 4: send_mode(MODE_SENSOR); break;
		}
	}
	return 0;
}
