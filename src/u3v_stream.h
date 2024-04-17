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


/* Internal stream structs and enums */
enum buffer_state {
	u3v_idle, u3v_queued, u3v_complete, u3v_cancelled
};

struct urb_info {
	struct u3v_stream *stream;
	struct buffer_entry *entry;
	struct usbd_urb *purb;
	bool kernel_allocated;
	size_t buffer_size;
	size_t min_expected_size;
	_Uint8t *buffer;
	_Uint32t urb_index;
};



struct buffer_entry {
	struct urb_info *urb_info_array;
	pthread_mutex_t mutex_buffer_complete;
	pthread_cond_t cond_buffer_complete;
	int done_buffer_complete;
	enum buffer_state state;
	unsigned callbacks_received;
	int urb_info_count;
	void *transfer2_addr;
	_Uint32t incomplete_callbacks_received;
	_Uint32t leader_received_size;
	_Uint32t payload_received_size;
	_Uint32t trailer_received_size;
	_Uint32t transfer2_received_size;
	_Uint32t status;
	_Uint64t buffer_id;
};

struct u3v_stream_buffer_config {
	uint32_t max_leader_size;
	uint32_t max_trailer_size;
	uint32_t payload_size;
	uint32_t payload_count;
	uint32_t transfer1_size;
	uint32_t transfer2_size;
	uint32_t transfer2_data_size;
	uint64_t image_buffer_size;
	uint64_t chunk_data_buffer_size;
};

struct u3v_stream {
	struct u3v_device *u3v_dev;
	struct buffer_entry *root;
	struct u3v_stream_buffer_config config;
	struct usbd_pipe* streaming_pipe;
	bool wait_in_progress;
	pthread_mutex_t stream_lock;
	_Uint64t next_buffer_id;
};

int u3v_create_stream(struct u3v_device *u3v, struct usbd_interface *intf, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size);
int u3v_destroy_stream(struct u3v_device *u3v);
static int set_buffer_sizes(struct u3v_stream *stream, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size);
int u3v_configure_buffer(struct u3v_stream *stream, void *user_image_buffer, void *user_chunk_data_buffer, _Uint64t *buffer_id);
static int create_buffer_entry(struct u3v_stream *stream, void *user_image_buffer, void *user_chunk_data_buffer, _Uint64t *buffer_id);
int u3v_unconfigure_buffer(struct u3v_stream *stream, _Uint64t buffer_id);
static int destroy_buffer(struct u3v_stream *stream, _Uint64t buffer_id);
int u3v_queue_buffer(struct u3v_stream *stream, _Uint64t buffer_id);
static void reset_counters(struct buffer_entry *entry);
static int map_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, void *user_buffer, size_t buffer_size, size_t offset, size_t urb_buffer_size, size_t min_expected_size);
static int set_buffer_sizes(struct u3v_stream *stream, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size);
static int submit_stream_urb(struct u3v_stream *stream, struct urb_info *urb_info);
static int allocate_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, size_t size, size_t min_expected_size);
#endif

