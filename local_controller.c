#include <stdio.h>
#include <stdlib.h>
#include "ipc.h"
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>

//placeholder values will be changed later with appropriate calculations
#define AMBER_DURATION 4
#define ALL_RED_DURATION 2
#define CLEARANCE_DURATION 10
#define MAX_GREEN_SENSOR 30
#define CHECK_INTERVAL 10
const int GREEN_DURATION_MODES[2][2] = { //green duration differs based on mode
		[MODE_CONGESTION] = {30, 20}, //more value for vertical since bigger road and cut off by train track
		[MODE_SENSOR] = {30,30} //sensor driven mode used for off-peak hours
};

pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;

LightState current_state = V_GREEN;  //start with vertical green
OpMode current_mode = MODE_SENSOR; //start with default mode (off-peak hours)
bool train_arriving = false;
int intersection_id = 1;

bool car_waiting_V = false;
bool car_waiting_H = false;

name_attach_t *attach;
int central_coid = -1;

//helper function to determine which duration of green to use
int green_duration(int is_vertical) {
	pthread_mutex_lock(&m);
	OpMode mode = current_mode;
	pthread_mutex_unlock(&m);
	return GREEN_DURATION_MODES[mode][is_vertical ? 0 : 1];
}
//function to get the current light state for both horizontal and vertical directions
LightState get_state(LightState current_state, int is_vertical) {
	switch(current_state) {
		case V_GREEN:
			return is_vertical ? V_GREEN : H_RED;
		case V_AMBER:
			return is_vertical ? V_AMBER : H_RED;
		case H_GREEN:
			return is_vertical ? V_RED : H_GREEN;
		case H_AMBER:
			return is_vertical ? V_RED : H_AMBER;
		case ALL_RED:
		case RAIL_SAFE:
		default:
			return is_vertical ? V_RED : H_RED; //in ALL_RED and RAIL_SAFE cases both vertical and horizontal stays red
	}

}

//function for sending status to central controller
void send_status(void) {

	pthread_mutex_lock(&m);
	LightState state = current_state;
	OpMode mode = current_mode;
	bool train = train_arriving;
	pthread_mutex_unlock(&m);

	if(central_coid == -1) {
		central_coid = name_open(CENTRAL_ATTACH_POINT, 0);
	}
	if(central_coid == -1) return;

	//data to be sent to central
	StatusMsg msg;
	memset(&msg, 0, sizeof(msg));
	msg.hdr.type = MSG_STATUS_UPDATE;
	msg.intersection_id = intersection_id;
	msg.mode = mode;
	msg.V_light = get_state(state, 1);
	msg.H_light = get_state(state, 0);
	msg.train_warning = train ? 1 : 0;

	//send data to central, drop if fails
	AckReply reply;
	if(MsgSend(central_coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
		name_close(central_coid);
		central_coid = -1;
	}
}

//helper function to switch to next state and send status to central controller every time switching
void set_phase(LightState next_state) {
	pthread_mutex_lock(&m);
	current_state = next_state;
	pthread_mutex_unlock(&m);
	send_status();
}

//helper function for train arrival detection
bool train_arrive(void) {
	pthread_mutex_lock(&m);
	bool arrive = train_arriving;
	pthread_mutex_unlock(&m);
	return arrive;
}

//helper function to allow emergency state switch without waiting for sleep()
bool interrupt_sleep(int seconds) {
	for (int i = 0; i < seconds; i++) {
		sleep(1);
		if(train_arrive()) return false;
	}
	return true;
}

//hold the current light green until max allowed time reached, check the other direction in the process, if found then switch
bool hold_green(bool is_vertical) {
	int total_elapsed = 0; //total time ran
	int checkpoint = 0; //checkpoint to check if the other direction has car waiting
	while(1) {
		sleep(1);
		if(train_arrive()) return false;

		total_elapsed++;
		checkpoint++;

		if(total_elapsed >= GREEN_DURATION_MODES[MODE_SENSOR][is_vertical ? 0 : 1]) return true; //if the max allowed time reach then switch

		if(checkpoint >= CHECK_INTERVAL){ //if reach the check interval then see if another car is waiting, if yes then switch, if not continue
			pthread_mutex_lock(&m);
			bool other_side_waiting = is_vertical ? car_waiting_H : car_waiting_V;
			pthread_mutex_unlock( &m);

			checkpoint = 0; //reset for the next check
			if(other_side_waiting) return true; //if other side got car waiting then switch
		}
	}
}

//traffic state machine
void traffic_light_state(void) {
	pthread_mutex_lock(&m);
	LightState state = current_state;
	pthread_mutex_unlock(&m);

	int horizontal_next = 1;

	while(1) {

		//train arrival handling
		if(train_arrive()) {
			if(state == V_GREEN) { //vertical is green and train detected so switch to clearance duration instead
				interrupt_sleep(CLEARANCE_DURATION);
				set_phase(V_AMBER);
				interrupt_sleep(AMBER_DURATION);
			} else if(state == H_GREEN){ //horizontal is green so switch to yellow for clearance on vertical
				set_phase(H_AMBER);
				interrupt_sleep(AMBER_DURATION);
			} else if(state == ALL_RED && horizontal_next == 1) { //All lights currently at red so switch to vertical green for clearing
				set_phase(V_GREEN);
				interrupt_sleep(CLEARANCE_DURATION);
				set_phase(V_AMBER);
				interrupt_sleep(AMBER_DURATION);
			}
			//set to rail protocol then sleep until train is cleared, after that set v_green first
			set_phase(RAIL_SAFE);
			while(train_arrive()) sleep(1);
			state = V_GREEN;
			continue;
		}

		//usual state machine mechanism
		switch(state) {
			case V_GREEN: {
				set_phase(V_GREEN);
				bool advanced;
				pthread_mutex_lock(&m);
				OpMode mode = current_mode;
				pthread_mutex_unlock(&m);

				//additional step for the sensor driven mode
				if (mode == MODE_SENSOR) {
					advanced = hold_green(true);
					if (advanced) {
						pthread_mutex_lock(&m);
						car_waiting_V = false;
						pthread_mutex_unlock(&m);
					}
				} else {
					advanced = interrupt_sleep(green_duration(1));
				}
				state = advanced ? V_AMBER : state;
				break;
			}

			case V_AMBER:
				set_phase(V_AMBER);
				state = interrupt_sleep(AMBER_DURATION) ? ALL_RED : state;
				break;

			case ALL_RED:
				set_phase(ALL_RED);

				if(interrupt_sleep(ALL_RED_DURATION)) {
					if(horizontal_next) {
						state = H_GREEN;
						horizontal_next = 0;
					} else {
						state = V_GREEN;
						horizontal_next = 1;
					}
				}
				break;

			case H_GREEN: {
				set_phase(H_GREEN);
				bool advanced;
				pthread_mutex_lock(&m);
				OpMode mode = current_mode;
				pthread_mutex_unlock(&m);

				//additional step for the sensor driven mode
				if (mode == MODE_SENSOR) {
					advanced = hold_green(false);
					if (advanced) {
						pthread_mutex_lock(&m);
						car_waiting_H = false;
						pthread_mutex_unlock(&m);
					}
				} else {
					advanced = interrupt_sleep(green_duration(0));
				}
				state = advanced ? H_AMBER : state;
				break;
			}

			case H_AMBER:
				set_phase(H_AMBER);
				state = interrupt_sleep(AMBER_DURATION) ? ALL_RED : state;
				break;

			case RAIL_SAFE:
				break;
		}
	}
}

//handle messages send to local controller
void handle_message(const Anymsg *msg, AckReply *reply) {

	//change mode sensor or congestion sent from central
	if (msg->hdr.type == MSG_COMMAND_SET_MODE) {
		pthread_mutex_lock(&m);
		current_mode = msg->command.target_mode;
		pthread_mutex_unlock(&m);

		snprintf(reply->buf, REPLY_BUF_SIZE, "Mode changed to %d", msg->command.target_mode);

	} else if (msg->hdr.type == MSG_SENSOR_EVENT) { //for sensor-driven events
		pthread_mutex_lock(&m);
		if (msg->sensor.event == TRAIN_GATE_DOWN || msg->sensor.event == TRAIN_FAULT) {
			train_arriving = true;

		} else if (msg->sensor.event == TRAIN_GATE_CLEAR) {
			train_arriving = false;
		} else if (msg->sensor.event == CAR_DETECTED_V) {
			car_waiting_V = true;
		} else if (msg->sensor.event == CAR_DETECTED_H) {
			car_waiting_H = true;
		}
		/*


			SPACE FOR PEDESTRIAN CASE



		 */
		pthread_mutex_unlock(&m);
		snprintf(reply->buf, REPLY_BUF_SIZE, "Sensor event %d processed", msg->sensor.event);

	} else {
		snprintf(reply->buf, REPLY_BUF_SIZE, "Unknown message type %d", msg->hdr.type); //message not known
	}
}

//listen for message sent to local controller
void *server_thread(void *arg) {

	Anymsg msg;
	int rcvid = 0, msgnum = 0;
	int Stay_alive = 1, living = 0;

	living = 1;
	while (living) {

		rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);

		if (rcvid == -1) {
			printf("\nFailed to receive\n");
			break;
		}

		//handle pulses
		if (rcvid == 0) {
			switch (msg.hdr.code) {
				case _PULSE_CODE_DISCONNECT:
					if (Stay_alive == 0) {
						ConnectDetach(msg.hdr.scoid);
						printf("\n[I%d] Server was told to Detach ...\n", intersection_id);
						living = 0;
						continue;
					} else {
						printf("\n[I%d] Server received Detach pulse but rejected it ...\n", intersection_id);
					}
					break;

				case _PULSE_CODE_UNBLOCK:
					printf("\n[I%d] Server got _PULSE_CODE_UNBLOCK after %d msgnum\n", intersection_id, msgnum);
					break;

				case _PULSE_CODE_COIDDEATH:
					printf("\n[I%d] Server got _PULSE_CODE_COIDDEATH after %d msgnum\n", intersection_id, msgnum);
					break;

				case _PULSE_CODE_THREADDEATH:
					printf("\n[I%d] Server got _PULSE_CODE_THREADDEATH after %d msgnum\n", intersection_id, msgnum);
					break;

				default:
					printf("\n[I%d] Server got some other pulse after %d msgnum\n", intersection_id, msgnum);
					break;
			}
			continue;
		}

		//message detected
		if (rcvid > 0) {
			msgnum++;

			//handshake
			if (msg.hdr.type == _IO_CONNECT) {
				MsgReply(rcvid, EOK, NULL, 0);
				msgnum--;
				continue;
			}

			//I/O messages
			if (msg.hdr.type > _IO_BASE && msg.hdr.type <= _IO_MAX) {
				MsgError(rcvid, ENOSYS);
				continue;
			}

			//process message received
			AckReply reply;
			memset(&reply, 0, sizeof(reply));
			reply.hdr.type = 0x01;
			reply.hdr.subtype = 0x00;

			handle_message(&msg, &reply); //handle different messages
			MsgReply(rcvid, EOK, &reply, sizeof(reply)); //send back reply
		} else {
			printf("\n[I%d] ERROR: Server received something, but could not handle it correctly\n", intersection_id);
		}
	}

	name_detach(attach, 0);
	return NULL;
}


int main(int argc, char *argv[]) {
	if(argc > 1 ) {
		intersection_id = atoi(argv[1]); //register intersection id from terminal
	}

	char name[NAME_MAXLEN];
	local_ctrl_name(name, intersection_id);

	//attach using name
	if((attach = name_attach(NULL, name, 0)) == NULL) {
		printf("\nFailed to name_attach: %s\n", name);
		printf("\nAnother server with same name maybe running\n");
		return EXIT_FAILURE;
	}

	printf("I%d local controller listening on ATTACH_POINT: %s\n", intersection_id, name);
	fflush(stdout);

	pthread_t t1;
	pthread_create(&t1, NULL, server_thread, NULL);

	traffic_light_state();

	pthread_join(t1, NULL);

	return 0;
}
