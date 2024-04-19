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

buffer_entry* create_buffer_entry(u3v_stream *stream) {
	buffer_entry* entry = (buffer_entry*)malloc(sizeof(buffer_entry));
	if (entry == NULL) {
		return NULL; // Allocation failed
	}

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
			for (int j = 0; j < i; ++j) {
				destroy_urb(&entry->urb_info_array[j]);
			}
			free(entry->urb_info_array);
			free(entry);
			return NULL;
		}

		// Allocate buffer based on URB index (leader, payload, or trailer)
		size_t buffer_size;
		size_t min_expected_size;
		if (i == 0) {
			// Leader
			buffer_size = stream->config.max_leader_size;
			min_expected_size = sizeof(struct leader_header);
		}
		else if (i == num_urbs - 1) {
			// Trailer
			buffer_size = stream->config.max_trailer_size;
			min_expected_size = sizeof(struct trailer_header);
		}
		else {
			// Payload
			buffer_size = stream->config.payload_size;
			min_expected_size = stream->config.payload_size;
		}

		if (allocate_urb_buffer(stream, &entry->urb_info_array[i], buffer_size, min_expected_size) != 0) {
			// Handle buffer allocation failure
			destroy_urb(&entry->urb_info_array[i]); // Free the URB
			for (int j = 0; j < i; ++j) {
				destroy_urb(&entry->urb_info_array[j]);
			}
			free(entry->urb_info_array);
			free(entry);
			return NULL;
		}
	}

	// Initialize other members of the buffer_entry structure
	entry->urb_info_count = num_urbs;
	entry->callbacks_received = 0;
	entry->state = u3v_idle;
	entry->buffer_id = stream->next_buffer_id++;
	entry->next = NULL;
	entry->prev = NULL;
	pthread_mutex_init(&entry->mutex_buffer_complete, NULL);
	pthread_cond_init(&entry->cond_buffer_complete, NULL);
	entry->done_buffer_complete = 0;

	return entry;
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
    for (int i = 0; i < static_cast<int>(stream->config.payload_count); i++) {
        buffer_entry* entry = create_buffer_entry(stream);
        if (entry == NULL) {
            // Handle allocation failure
        	u3v_destroy_stream(u3v); // Free previously allocated entries and stream
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
	if (stream == NULL)
		return -EINVAL;

	std::cout << __func__ << ": image buffer size = " << image_buffer_size << "\n";
	std::cout << __func__ << ": chunk data buffer size = " << chunk_data_buffer_size << "\n";
	std::cout << __func__ << ": payload size = " << payload_size << "\n";
	std::cout << __func__ << ": payload count = " << payload_count << "\n";
	std::cout << __func__ << ": transfer1 = " << transfer1_size << "\n";
	std::cout << __func__ << ": transfer2 = " << transfer2_size << "\n";

	/*
    // Calculate the number of payload URBs needed
    _Uint32t num_payload_urbs = (_Uint32t)ceil((double)image_buffer_size / payload_size);

    // Adjust payload count if needed (ensure enough URBs for the image)
    if (num_payload_urbs > payload_count) {
        payload_count = num_payload_urbs;
    }
	*/
    // Store buffer configuration parameters
    stream->config.image_buffer_size = image_buffer_size;
    stream->config.max_leader_size = max_leader_size;
    stream->config.max_trailer_size = max_trailer_size;
    stream->config.payload_size = payload_size;
    stream->config.payload_count = payload_count;

    return 0;
}

int u3v_destroy_stream(struct u3v_device *u3v) {
	struct u3v_stream *stream;
	int ret;

	if (u3v == NULL) {
		return -EINVAL;
	}

	stream = (struct u3v_stream *)(u3v->stream_info.interface_ptr);

	if (stream == NULL) {
		return U3V_ERR_STREAM_NOT_CONFIGURED;
	}

	pthread_mutex_lock(&stream->stream_lock);

	ret = reset_stream(stream);
	if (ret != 0) {
		goto error;
	}

	// Wait for ongoing ioctls to complete
	wait_for_completion(u3v->stream_info.mutex_ioctl_complete, u3v->stream_info.cond_ioctl_complete, u3v->stream_info.done_ioctl_complete);

	// Free all buffer entries in the linked list
	while (!is_queue_empty(stream)) {
		buffer_entry *entry = dequeue_buffer(stream);
		destroy_buffer_entry(entry);
	}

	pthread_mutex_unlock(&stream->stream_lock);

	// Free the stream structure
	free(stream);
	u3v->stream_info.interface_ptr = NULL;
	return 0;

error:
	pthread_mutex_unlock(&stream->stream_lock);
    return ret;
}

static int reset_stream(struct u3v_stream *stream) {
	struct u3v_device *u3v;
	struct usbd_device *udev;
	uint8_t ep_addr;
	int dummy_size = 32;
	uint8_t dummy_buffer[dummy_size];
	int actual = 0;
	int ret = 0;

	if (stream == NULL) {
		return -EINVAL;
	}

	u3v = stream->u3v_dev;
	if (!u3v->device_connected) {
		return 0;
	}

	udev = u3v->udev;
	ep_addr = u3v->stream_info.bulk_in->bEndpointAddress;

	// QNX specific: Aborting URBs
	while (!is_queue_empty(stream)) {
		buffer_entry *entry = dequeue_buffer(stream);
		for (int i = 0; i < entry->urb_info_count; i++) {
			usbd_urb *urb = entry->urb_info_array[i].purb;
			if (urb) {
				usbd_abort_pipe(stream->streaming_pipe); // Abort URB on the streaming pipe
				usbd_urb_status(urb, NULL, NULL); // Clear the URB status
			}
		}
		enqueue_buffer(stream, entry); // Re-enqueue the buffer entry
	}

	// QNX specific: Resetting the streaming pipe
	if (!u3v->stalling_disabled) {
		ret = usbd_reset_pipe(stream->streaming_pipe);
		if (ret != EOK) {
    	 //dev_err(u3v->device, "%s: Error %d resetting pipe for ep %02X\n", __func__, ret, ep_addr);
		}
	}

 	return ret;
}

static int allocate_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, size_t size, size_t min_expected_size) {
  if (urb_info == NULL) {
    return -EINVAL;
  }

  // Allocate buffer using usbd_alloc
  urb_info->buffer = reinterpret_cast<_Uint8t*>(usbd_alloc(size));
  if (urb_info->buffer == NULL) {
    return -ENOMEM;
  }

  // Initialize urb_info structure
  urb_info->kernel_allocated = true;
  urb_info->buffer_size = size;
  urb_info->min_expected_size = min_expected_size;

  return 0; // Success
}

static int submit_stream_urb(struct u3v_stream *stream, struct urb_info *urb_info) {
	struct device *dev;
	struct usbd_device *udev;
	int ret;
	struct urb *purb;

	if (stream == NULL || urb_info == NULL || urb_info->purb == NULL) {
		return -EINVAL;
	}

	dev = stream->u3v_dev->device;
	udev = stream->u3v_dev->udev;

	if (urb_info->kernel_allocated && (urb_info->buffer == NULL)) {
		//dev_err(dev, "%s: NULL buffer for kernel URB\n", __func__);
		return -EINVAL;
	}

	// QNX specific: Setting up the URB for bulk transfer
	if (static_cast<int>(urb_info->urb_index) == 0 || static_cast<int>(urb_info->urb_index) == static_cast<int>(urb_info->entry->urb_info_count) - 1) {
		// Leader or trailer transfer (direction is always IN)
		ret = usbd_setup_bulk(urb_info->purb, URB_DIR_IN, urb_info->buffer, urb_info->buffer_size);
	} else {
		// Payload transfer (direction depends on the URB index)
		if (urb_info->urb_index % 2 == 0) {
			ret = usbd_setup_bulk(urb_info->purb, URB_DIR_IN, urb_info->buffer, urb_info->buffer_size);
		}
		else {
			ret = usbd_setup_bulk(urb_info->purb, URB_DIR_OUT, urb_info->buffer, urb_info->buffer_size);
		}
	}

	if (ret != EOK) {
		//dev_err(dev, "%s: Failed to set up URB\n", __func__);
		return ret;
	}

	// QNX specific: Submitting the URB
	ret = usbd_io(urb_info->purb, stream->streaming_pipe, NULL, urb_info, U3V_TIMEOUT);
	if (ret != EOK) {
		//dev_err(dev, "%s: Error %d submitting urb\n", __func__, ret);
	}

	return ret;
}

int u3v_wait_for_buffer(struct u3v_stream *stream, _Uint64t buffer_id, void *user_leader_buffer, _Uint32t *u_leader_size, void *user_trailer_buffer, _Uint32t *trailer_size, void *buffer_complete_data) {
	struct buffer_entry *entry;
	struct device *dev;
	struct buffer_complete_data bcd;
	int ret = 0;
	int leader_index;
	int transfer2_index;
	int trailer_index;
	struct trailer *trailer;

	if (stream == NULL) {
		return ret;
	}

	dev = stream->u3v_dev->device;

	//dev_vdbg(dev, "%s: buffer id = %llu\n", __func__, buffer_id);

	pthread_mutex_lock(&stream->stream_lock);

	// Find the buffer entry with the specified ID
	// Assuming linked list implementation, you might need to adjust this search
	entry = stream->head;
	while (entry != NULL && entry->buffer_id != buffer_id) {
		entry = entry->next;
	}

	if (entry == NULL) {
		//dev_err(dev, "%s: Failed to find buffer with id %llu\n", __func__, buffer_id);
		ret = -EINVAL;
		goto exit;
	}

	// Handle different buffer states
	if (entry->state == u3v_idle) {
	  //dev_err(dev, "%s: Error, cannot wait for idle buffer\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (entry->state == u3v_cancelled) {
		//dev_err(dev, "%s: Error, cannot wait for cancelled buffer\n", __func__);
		ret = -EINVAL;
		goto exit;
  }

	if (stream->wait_in_progress) {
		//dev_err(dev, "%s: Error, wait is already in progress\n", __func__);
		ret = U3V_ERR_ALREADY_WAITING;
		goto exit;
	}

 	// Wait for the buffer to complete
	if (entry->callbacks_received != static_cast<int>(entry->urb_info_count)) {
		stream->wait_in_progress = true;
		pthread_mutex_lock(&entry->mutex_buffer_complete);
		while (!entry->done_buffer_complete) {
			pthread_cond_wait(&entry->cond_buffer_complete, &entry->mutex_buffer_complete);
		}
		pthread_mutex_unlock(&entry->mutex_buffer_complete);
		stream->wait_in_progress = false;
	}

	// Extract information from the buffer
	trailer = (struct trailer *) entry->urb_info_array[entry->urb_info_count - 1].buffer;
 	//dev_vdbg(dev, "%s: device reports valid payload size is %llu\n", __func__, trailer->header.valid_payload_size);
	//dev_vdbg(dev, "%s: driver reports payload received size is %u\n", __func__, entry->payload_received_size);
	//dev_vdbg(dev, "%s: device reports status %04X\n", __func__, trailer->header.status);

	// Check if wait was successful
	if (ret != 0) {
		//dev_err(dev, "%s: Error, interrupted by signal\n", __func__);
		ret = U3V_ERR_BUFFER_WAIT_CANCELED;
		goto exit;
	}

	if (entry->state != u3v_complete) {
		//dev_err(dev, "%s: Cannot copy from the requested buffer because it has been requeued or cancelled\n", __func__);
		ret = U3V_ERR_BUFFER_CANNOT_BE_COPIED;
		goto exit;
	}

	// Fill in the requested information
	leader_index = 0;
	transfer2_index = entry->urb_info_count - 2;
	trailer_index = entry->urb_info_count - 1;

	if (user_leader_buffer != NULL && u_leader_size != NULL) {
		uint32_t leader_size = min(entry->leader_received_size, *u_leader_size);
		memcpy(user_leader_buffer, entry->urb_info_array[leader_index].buffer, leader_size);
		*u_leader_size = leader_size;
	}

	if (entry->transfer2_addr != NULL) {
		uint32_t transfer2_size = min((size_t)(entry->transfer2_received_size), entry->urb_info_array[transfer2_index].min_expected_size);
		memcpy(entry->transfer2_addr, entry->urb_info_array[transfer2_index].buffer, transfer2_size);
	}

	if (user_trailer_buffer != NULL && trailer_size != NULL) {
		uint32_t trailer_size_to_copy = min(entry->trailer_received_size, *trailer_size);
		memcpy(user_trailer_buffer, entry->urb_info_array[trailer_index].buffer, trailer_size_to_copy);
		*trailer_size = trailer_size_to_copy;
	}

	if (buffer_complete_data != NULL) {
		bcd.structure_size = sizeof(struct buffer_complete_data);
		bcd.status = entry->status;
		bcd.expected_urb_count = entry->urb_info_count;
		bcd.incomplete_urb_count = entry->incomplete_callbacks_received;
		bcd.payload_bytes_received = entry->payload_received_size;
		memcpy(buffer_complete_data, &bcd, bcd.structure_size);
	}

exit:
  pthread_mutex_unlock(&stream->stream_lock);
  return ret;
}

int u3v_cancel_all_buffers(struct u3v_stream *stream) {
	int ret;

	if (stream == NULL) {
		return -EINVAL;
	}

	pthread_mutex_lock(&stream->stream_lock);

	// Cancel all URBs and reset the stream endpoint
	ret = reset_stream(stream);

	// Mark all buffer entries as cancelled
	buffer_entry *entry = stream->head;
	while (entry != NULL) {
		entry->state = u3v_cancelled;
		pthread_mutex_lock(&entry->mutex_buffer_complete);
		entry->done_buffer_complete = 1; // Signal completion for waiting threads
		pthread_cond_broadcast(&entry->cond_buffer_complete);
		pthread_mutex_unlock(&entry->mutex_buffer_complete);
		entry = entry->next;
	}

	pthread_mutex_unlock(&stream->stream_lock);

	return ret;
}

static void destroy_urb(struct urb_info *urb_info) {
	if (urb_info == NULL) {
		return;
	}

	// Free the URB
	usbd_free_urb(urb_info->purb);
	urb_info->purb = NULL;

	// Free the buffer if it was kernel-allocated
	if (urb_info->kernel_allocated) {
		usbd_free(urb_info->buffer);
		urb_info->buffer = NULL;
	}
}

void destroy_buffer_entry(buffer_entry *entry) {
	if (entry == NULL) {
		return;
	}

	// Free URBs and buffers
	for (int i = 0; i < entry->urb_info_count; i++) {
		destroy_urb(&entry->urb_info_array[i]);
	}

	// Free urb_info_array
	free(entry->urb_info_array);

	// Destroy mutex and condition variable
	pthread_mutex_destroy(&entry->mutex_buffer_complete);
	pthread_cond_destroy(&entry->cond_buffer_complete);

    // Free the buffer_entry itself
  	 free(entry);
}

int u3v_configure_buffer(struct u3v_stream *stream, void *user_image_buffer, void *user_chunk_data_buffer, _Uint64t *buffer_id) {
	struct device *dev;
	int ret;
	buffer_entry *entry;
	size_t image_offset;
	size_t chunk_data_offset;

	if (stream == NULL || user_image_buffer == NULL || buffer_id == NULL) {
		return -EINVAL;
	}
	dev = stream->u3v_dev->device;

	if ((user_chunk_data_buffer == NULL && stream->config.chunk_data_buffer_size != 0) || (user_chunk_data_buffer != NULL && stream->config.chunk_data_buffer_size == 0)) {
		//dev_err(dev, "%s: Invalid chunk data buffer address and size combination\n", __func__);
    return -EINVAL;
	}

	// Allocate and initialize a buffer entry
	entry = create_buffer_entry(stream);
	if (entry == NULL) {
		return -ENOMEM;
	}

	// Initialize buffer offsets for copying data
	image_offset = 0;
	chunk_data_offset = 0;

		// Copy user data to the kernel buffers within the entry
	for (int i = 0; i < entry->urb_info_count; ++i) {
		size_t size_to_copy;
		void *source_buffer;
		if (i == 0 || i == entry->urb_info_count - 1) {
			// Leader or trailer - no need to copy user data
			continue;
		}
		else if (image_offset < stream->config.image_buffer_size) {
			// Copy from user image buffer
			size_to_copy = min(stream->config.payload_size, stream->config.image_buffer_size - image_offset);
			source_buffer = (uint8_t *)user_image_buffer + image_offset;
			image_offset += size_to_copy;
		}
		else {
			// Copy from user chunk data buffer
			size_to_copy = min(stream->config.payload_size, stream->config.chunk_data_buffer_size - chunk_data_offset);
			source_buffer = (uint8_t *)user_chunk_data_buffer + chunk_data_offset;
			chunk_data_offset += size_to_copy;
		}

		memcpy(entry->urb_info_array[i].buffer, source_buffer, size_to_copy);
	}

	// Assign buffer ID and add entry to the linked list
	entry->buffer_id = stream->next_buffer_id++;
	*buffer_id = entry->buffer_id;
	enqueue_buffer(stream, entry);

	return 0;
}
