#ifndef _U3V_CONTROL_H_
#define _U3V_CONTROL_H_

#include <errno.h>
#include <sys/usbdi.h>
#include <iostream>
#include <pthread.h>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"

/* Structs */

struct u3v_control {
	struct u3v_device *u3v_dev;
	pthread_mutex_t read_write_lock;
	uint8_t *ack_buffer;
	uint32_t max_ack_transfer_size;
	uint8_t *cmd_buffer;
	uint32_t max_cmd_transfer_size;
	uint16_t request_id;
	uint16_t max_request_id; /* Maximum id value we can have before we loop back around */
	uint32_t u3v_timeout; /* Maximum device response time in ms */
};



int u3v_create_control(struct u3v_device *u3v);
int u3v_read_mem(struct u3v_control *ctrl, uint32_t transfer_size, uint32_t *bytes_read, uint64_t address, void *kernel_buffer, void *user_buffer, uint32_t flags);
int u3v_write_mem(struct u3v_control *ctrl, uint32_t transfer_size, uint32_t *bytes_written, uint64_t address, const void *kernel_buffer, const void *user_buffer, uint32_t flags);

#endif
