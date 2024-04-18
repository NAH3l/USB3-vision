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
    buffer_entry* next; // Pointer to the next buffer entry
    buffer_entry* prev; // Pointer to the previous buffer entry
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
    buffer_entry* head; // Pointer to the head of the buffer queue
    buffer_entry* tail; // Pointer to the tail of the buffer queue
};

void initialize_queue(u3v_stream *stream);
void enqueue_buffer(u3v_stream *stream, buffer_entry *entry);
buffer_entry* dequeue_buffer(u3v_stream *stream);
bool is_queue_empty(u3v_stream *stream);
buffer_entry* create_buffer_entry(u3v_stream *stream);
void destroy_buffer_entry(buffer_entry *entry);
int u3v_create_stream(struct u3v_device *u3v, struct usbd_interface *intf, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size);
static int set_buffer_sizes(struct u3v_stream *stream,_Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size,_Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size);
int u3v_destroy_stream(struct u3v_device *u3v);
static int reset_stream(struct u3v_stream *stream);
#endif

