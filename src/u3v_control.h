#ifndef _U3V_CONTROL_H_
#define _U3V_CONTROL_H_

#include <errno.h>
#include <sys/usbdi.h>
#include <sys/usb100.h>
#include <iostream>
#include <stdlib.h>
#include <gulliver.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/syspage.h>
#include <atomic.h>
#include <xmlparse.h>

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
	struct usbd_pipe* control_pipe;
	pthread_mutex_t read_write_lock;
	uint8_t *ack_buffer;
	uint32_t max_ack_transfer_size;
	uint8_t *cmd_buffer;
	uint32_t max_cmd_transfer_size;
	uint16_t request_id;
	uint32_t u3v_timeout; /* Maximum device response time in ms */
};

int u3v_create_control(struct u3v_device *u3v);
void u3v_destroy_control(struct u3v_device *u3v);

int u3v_read_memory(struct u3v_control *ctrl, _Uint32t transfer_size, _Uint32t *bytes_read, _Uint64t address, void *kernel_buffer, void *user_buffer);
int u3v_write_memory(struct u3v_control *ctrl, _Uint32t transfer_size, _Uint32t *bytes_written, _Uint64t address, const void *kernel_buffer, const void *user_buffer);

#endif
