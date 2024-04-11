#ifndef _U3V_STREAM_H_
#define _U3V_STREAM_H_

#include <errno.h>
#include <sys/usbdi.h>
#include <iostream>
#include <stdlib.h>
#include <gulliver.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"

#define MAX_URBS 10 // Nombre maximum d'URBs dans l'ancre USB

struct u3v_stream_buffer_config {
	uint64_t image_buffer_size;
	uint64_t chunk_data_buffer_size;
	uint32_t max_leader_size;
	uint32_t max_trailer_size;
	uint32_t payload_size;
	uint32_t payload_count;
	uint32_t transfer1_size;
	uint32_t transfer2_size;
	uint32_t transfer2_data_size;
};



struct u3v_stream {
	struct u3v_device *u3v_dev;
	struct buffer_entry *root;
	struct u3v_stream_buffer_config config;
	struct usbd_pipe* stream_pipe;
	bool wait_in_progress;
	_Uint64t next_buffer_id;
	pthread_mutex_t stream_lock;

};

int u3v_create_stream(struct u3v_device *u3v, struct usbd_interface *intf, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size);
int u3v_destroy_stream(struct u3v_device *u3v);
int u3v_configure_buffer(struct u3v_stream *stream, void *user_image_buffer, void *user_chunk_data_buffer, _Uint64t *buffer_id);
int u3v_unconfigure_buffer(struct u3v_stream *stream, _Uint64t buffer_id);
int u3v_queue_buffer(struct u3v_stream *stream, _Uint64t buffer_id);
int u3v_wait_for_buffer(struct u3v_stream *stream, _Uint64t buffer_id, void *user_leader_buffer, _Uint32t *leader_size, void *user_trailer_buffer, _Uint32t *trailer_size, void *buffer_complete_data);
int u3v_cancel_all_buffers(struct u3v_stream *stream);

#endif

