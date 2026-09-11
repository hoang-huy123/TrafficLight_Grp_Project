
#ifndef IPC_H_
#define IPC_H_

#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <stdio.h>
#include <stdint.h>

typedef enum {
	V_GREEN,
	V_AMBER,
	V_RED,
	H_GREEN,
	H_AMBER,
	H_RED,
	ALL_RED,
	PED_CROSS,
	RAIL_SAFE
} LightState;

typedef enum {
	MODE_CONGESTION,
	MODE_SENSOR
} OpMode;

typedef union {
	union {
		_Uint32t sival_int;
		void *sival_ptr;
	};
	_Uint32t dummy[4];
} _mysigval;

typedef struct _Mypulse {
	_Uint16t type;
	_Uint16t subtype;
	_Int8t code;
	_Uint8t zero[3];
	_mysigval value;
	_Uint8t zero2[2];
	_Int32t scoid;
} msg_header_t;

typedef enum {
	MSG_STATUS_UPDATE = _IO_MAX + 1,
	 MSG_COMMAND_SET_MODE,
	 MSG_SENSOR_EVENT
} MsgType;

typedef struct {
	msg_header_t hdr;
	int intersection_id;
	OpMode mode;
	LightState V_light;
	LightState H_light;
	int train_warning;
} StatusMsg;

typedef struct {
	msg_header_t hdr;
	OpMode target_mode;
	int hold_green;
} CommandMsg;

typedef enum {
	TRAIN_GATE_DOWN,
	TRAIN_GATE_CLEAR,
	TRAIN_FAULT,
	PEDESTRIAN_PRESSED,
	CAR_DETECTED_V,
	CAR_DETECTED_H
} SensorEvent;

typedef struct {
	msg_header_t hdr;
	SensorEvent event;
} SensorMsg;

#define REPLY_BUF_SIZE 100
typedef struct {
	msg_header_t hdr;
	char buf[REPLY_BUF_SIZE];
} AckReply;

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
