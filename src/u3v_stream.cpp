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
#include <unistd.h>
#include <sys/syspage.h>
#include <atomic.h>
#include <math.h>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"

// Initialize the buffer queue (linked list)
void initialize_queue(u3v_stream *stream) {
    stream->head = NULL;
    stream->tail = NULL;
}

// Enqueue a buffer entry at the tail of the queue
void enqueue_buffer(u3v_stream *stream, buffer_entry *entry) {
    if (stream->tail == NULL) {
        // Queue is empty
        stream->head = entry;
        stream->tail = entry;
        entry->next = NULL;
        entry->prev = NULL;
    }
    else {
    	// Add entry to the tail
        stream->tail->next = entry;
        entry->prev = stream->tail;
        entry->next = NULL;
        stream->tail = entry;
    }
}

// Dequeue a buffer entry from the head of the queue
buffer_entry* dequeue_buffer(u3v_stream *stream) {
    if (stream->head == NULL) {
        // Queue is empty
        return NULL;
    }
    else {
        buffer_entry* entry = stream->head;
        stream->head = entry->next;
        if (stream->head == NULL) {
            // Queue is now empty
            stream->tail = NULL;
        } else {
            stream->head->prev = NULL;
        }
        return entry;
    }
}

// Check if the queue is empty
bool is_queue_empty(u3v_stream *stream) {
    return stream->head == NULL;
}

// Allocate and initialize a buffer entry
buffer_entry* create_buffer_entry(u3v_stream *stream) {
	buffer_entry* entry = (buffer_entry*)malloc(sizeof(buffer_entry));
    if (entry == NULL) {
        return NULL; // Allocation failed
    }
    // Calculate number of URBs needed
    int num_urbs = stream->config.payload_count + 2; // leader, payloads, trailer

    // Allocate urb_info_array
    entry->urb_info_array = (urb_info*)malloc(sizeof(urb_info) * num_urbs);
    if (entry->urb_info_array == NULL) {
        free(entry);
        return NULL; // Allocation failed
    }

    // Initialize URBs and allocate buffers
    for (int i = 0; i < num_urbs; i++) {
        entry->urb_info_array[i].urb_index = i;
        entry->urb_info_array[i].stream = stream;
        entry->urb_info_array[i].entry = entry;
        entry->urb_info_array[i].purb = usbd_alloc_urb(NULL); // Allocate URB
        if (entry->urb_info_array[i].purb == NULL) {
            // Handle URB allocation failure
            // ... free previously allocated URBs and buffers ...
            free(entry->urb_info_array);
            free(entry);
            return NULL;
        }

        // Allocate buffer based on URB index (leader, payload, or trailer)
        // ...
    }

    entry->urb_info_count = num_urbs;
    entry->callbacks_received = 0;
    entry->state = u3v_idle;
    entry->buffer_id = stream->next_buffer_id++;
    entry->next = NULL;
    entry->prev = NULL;

    return entry;
}

// Free a buffer entry and its associated resources
void destroy_buffer_entry(buffer_entry *entry) {
    if (entry == NULL)
    	return;

    // Free URBs and buffers
    for (int i = 0; i < entry->urb_info_count; i++) {
    	usbd_free_urb(entry->urb_info_array[i].purb);
        // ... free buffer based on URB index ...
    }
    free(entry->urb_info_array);
    free(entry);
}

int u3v_create_stream(struct u3v_device *u3v, struct usbd_interface *intf,
                      _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size,
                      _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size,
                      _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size) {

    struct u3v_stream *stream = NULL;
    int ret = 0;

    if (u3v == NULL)
        return -EINVAL;

    if (u3v->stream_info.bulk_in == NULL) {
        //dev_err(u3v->device, "%s: Did not detect bulk in endpoint in the stream interface.\n", __func__);
        return U3V_ERR_NO_STREAM_INTERFACE;
    }

    if (u3v->u3v_info->sirm_addr == 0) {
        //dev_err(u3v->device, "%s: the SIRM address is 0 - streaming is not supported\n", __func__);
        return U3V_ERR_NO_STREAM_INTERFACE;
    }

    if (u3v->stream_info.interface_ptr != NULL)
        return U3V_ERR_STREAM_ALREADY_CONFIGURED;

    stream = (struct u3v_stream *)malloc(sizeof(struct u3v_stream));
    if (stream == NULL)
        return -ENOMEM;

    pthread_mutex_init(&stream->stream_lock, NULL);
    stream->u3v_dev = u3v;
    stream->wait_in_progress = false;
    stream->next_buffer_id = 0;

    // Initialize the buffer queue using a linked list
    initialize_queue(stream);

    // Set buffer configurations
    ret = set_buffer_sizes(stream, image_buffer_size, chunk_data_buffer_size,
                           max_leader_size, max_trailer_size, payload_size, payload_count,
                           transfer1_size, transfer2_size);

    if (ret != 0) {
        free(stream);
        return ret;
    }

    // Allocate and enqueue buffer entries
    for (int i = 0; i < stream->config.payload_count; i++) {
        buffer_entry* entry = create_buffer_entry(stream);
        if (entry == NULL) {
            // Handle allocation failure
            destroy_stream(stream); // Free previously allocated entries and stream
            return -ENOMEM;
        }
        enqueue_buffer(stream, entry);
    }

    u3v->stream_info.interface_ptr = stream;
    return 0;
}

static int set_buffer_sizes(struct u3v_stream *stream,
                            _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size,
                            _Uint32t max_leader_size, _Uint32t max_trailer_size,
                            _Uint32t payload_size, _Uint32t payload_count,
                            _Uint32t transfer1_size, _Uint32t transfer2_size) {

    struct device *dev = stream->u3v_dev->device;

    // ... (code for checking buffer sizes and logging is the same as before) ...

    // Calculate the number of payload URBs needed
    _Uint32t num_payload_urbs = (_Uint32t)ceil((double)image_buffer_size / payload_size);

    // Adjust payload count if needed (ensure enough URBs for the image)
    if (num_payload_urbs > payload_count) {
        payload_count = num_payload_urbs;
    }

    // Store buffer configuration parameters
    stream->config.image_buffer_size = image_buffer_size;
    stream->config.max_leader_size = max_leader_size;
    stream->config.max_trailer_size = max_trailer_size;
    stream->config.payload_size = payload_size;
    stream->config.payload_count = payload_count;

    return 0;
}
