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

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"


/*
 * u3v_create_stream - Initializes the stream interface.
 * @u3v: pointer to the u3v_device struct
 * @intf: pointer to the stream interface struct
 * @image_buffer_size: the size of the buffers that will hold the image data.
 *	Could include size of both the image data and chunk data if they
 *	are to be acquired into the same buffer. Otherwise, it is just
 *	the size of the image data.
 * @chunk_data_buffer_size: the size of the buffers that will hold only
 *	chunk data. If the chunk data is acquired into the same buffer
 *	as the image data, this size should be 0.
 * @max_leader_size: the max required size + alignment for the leader buffer
 * @max_trailer_size: the max required size + alignment for the trailer buffer
 * @payload_size: size of each payload buffer
 * @payload_count: number of payload buffers
 * @transfer1_size: size of transfer1 payload buffer
 * @transfer2_size: size of transfer2 payload buffer
 */
int u3v_create_stream(struct u3v_device *u3v, struct usbd_interface *intf, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size) {
	return 0;
}

/*
 * u3v_destroy_stream - destroys the stream interface
 * @u3v: pointer to the u3v_device struct
 */
int u3v_destroy_stream(struct u3v_device *u3v) {
	return 0;
}


/*
 * set_buffer_sizes - helper function for configure_stream that initializes
 *	buffer size information and sets the device up for streaming
 * precondition: must be called from u3v_create_stream
 * @stream: pointer to the u3v_stream interface struct
 * @image_buffer_size: the size of the buffers that will hold the image data.
 *	Could include size of both the image data and chunk data if they
 *	are to be acquired into the same buffer. Otherwise, it is just
 *	the size of the image data.
 * @chunk_data_buffer_size: the size of the buffers that will hold only
 *	chunk data. If the chunk data is acquired into the same buffer
 *	as the image data, this size should be 0.
 * @max_leader_size: the max required size + alignment for the leader buffer
 * @max_trailer_size: the max required size + alignment for the trailer buffer
 * @payload_size: size of each payload buffer
 * @payload_count: number of payload buffers
 * @transfer1_size: size of transfer1 payload buffer
 * @transfer2_size: size of transfer2 payload buffer
 */
static int set_buffer_sizes(struct u3v_stream *stream, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size) {
	struct u3v_device *u3v;
	struct device *dev;
	_Uint32t transfer2_data_size;

	if (stream == NULL)
		return -EINVAL;

	u3v = stream->u3v_dev;
	dev = u3v->device;

	std::cout << __func__ << ": image buffer size = " << image_buffer_size << "\n";
	std::cout << __func__ << ": chunk data buffer size = " << chunk_data_buffer_size << "\n";
	std::cout << __func__ << ": payload size = " << static_cast<unsigned int>(payload_size) << "\n";
	std::cout << __func__ << ": payload count = " << static_cast<unsigned int>(payload_count) << "\n";
	std::cout << __func__ << ": transfer1 = " << static_cast<unsigned int>(transfer1_size) << "\n";
	std::cout << __func__ << ": transfer2 = " << static_cast<unsigned int>(transfer2_size) << "\n";

	if (image_buffer_size == 0 || max_leader_size == 0 || max_trailer_size == 0 || payload_size == 0) {
		std::cout << __func__ << ": leader, trailer, and image buffer sizes cannot be 0\n";
		std::cout << "\timage buffer size is " << image_buffer_size << "\n";
		std::cout << "\tmax leader buffer size is " << max_leader_size << "\n";
		std::cout << "\tmax trailer buffer size is " << max_trailer_size << "\n";
		std::cout << "\tpayload transfer buffer size is " << payload_size << "\n";
		return -EINVAL;
	}
	if ((image_buffer_size + chunk_data_buffer_size) < ((payload_size * payload_count) + transfer1_size)) {
		std::cout << __func__ << ": buffer sizes are too small to hold all of the requested DMA payload data. Total buffer size = "
		          << image_buffer_size + chunk_data_buffer_size << ", calculated DMA size = "
		          << ((payload_size * payload_count) + transfer1_size) << "\n";
		return -EINVAL;
	}
	/*
	 * Calculate how much data we actually expect to be valid in the
	 * final transfer 2 buffer
	 */
	transfer2_data_size = (image_buffer_size + chunk_data_buffer_size -
		(payload_size * payload_count) - transfer1_size);

	std::cout << __func__ << ": transfer2_data = " << transfer2_data_size << "\n";

	if (transfer2_data_size > transfer2_size) {
		std::cout << __func__ << ": final transfer 2 data size (" << transfer2_data_size << ") exceeds the size of the buffer (" << transfer2_size << ")\n";
		return -EINVAL;
	}

	/* The buffer sizes are valid, so now we store them */
	stream->config.image_buffer_size = image_buffer_size;
	stream->config.chunk_data_buffer_size = chunk_data_buffer_size;
	stream->config.max_leader_size = max_leader_size;
	stream->config.max_trailer_size = max_trailer_size;
	stream->config.payload_size = payload_size;
	stream->config.payload_count = payload_count;
	stream->config.transfer1_size = transfer1_size;
	stream->config.transfer2_size = transfer2_size;
	stream->config.transfer2_data_size = transfer2_data_size;

	return 0;
}

/*
 * u3v_configure_buffer - creates the buffer context data, page locks the
 *	user mode image and chunk data buffers, and adds the buffer to the
 *	rbtree of available buffers.
 * @stream: pointer to the stream interface struct
 * @user_image_buffer: user mode memory address of the image buffer to lock
 * @user_chunk_data_buffer: user mode memory address of the chunk data buffer
 *	to lock
 */
int u3v_configure_buffer(struct u3v_stream *stream, void *user_image_buffer, void *user_chunk_data_buffer, _Uint64t *buffer_id) {
	return 0;
}

/*
 * create_buffer_entry - this function allocates and initializes the buffer
 *	context data, page locks the image buffer and chunk data buffer,
 *	allocates buffers for the leader and trailer URBs, and allocates
 *	a scratch buffer for the final transfer 2 URB if necessary
 *
 * @stream: pointer to the stream interface struct
 * @user_image_buffer: memory address of the user image buffer to be locked
 * @user_chunk_data_buffer: memory address of the user chunk data buffer to
 *	be locked
 * @buffer_id: on return this user pointer points to valid buffer id
 */
static int create_buffer_entry(struct u3v_stream *stream, void *user_image_buffer, void *user_chunk_data_buffer, _Uint64t *buffer_id) {
	return 0;
}
/*
 * allocate_urb_buffer - this function populates a urb_info struct by
 *	allocating a kernel buffer. If a user buffer exists and needs to
 *	be mapped to the kernel, map_urb_buffer is used instead of this
 *	function
 * @stream: pointer to the stream interface struct
 * @urb_info: on return this struct will be initialized
 * @size: size of the buffer to allocate
 * @min_expected_size: minimum amount of data we expect to receive
 */
static int allocate_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, size_t size, size_t min_expected_size) {
    if (urb_info == NULL)
        return -EINVAL;

    urb_info->buffer = (_Uint8t *)malloc(size);
    if (urb_info->buffer == NULL)
        return -ENOMEM;

    urb_info->buffer_size = size;
    urb_info->min_expected_size = min_expected_size;
    return 0;
}

/*
 * map_urb_buffer - this function locks a segment of user memory, creates
 *	a corresponding scatter-gather list, and populates the urb_info
 *	struct accordingly. This function is an alternative to
 *	allocate_urb_buffer and is used to DMA directly into the user
 *	buffer without making a copy.
 * @stream: pointer to the stream interface struct
 * @urb_info: on return this struct will be initialized
 * @user_buffer: user mode memory address of the buffer to lock
 * @buffer_size: size of the user buffer
 * @offset: offset into the user buffer
 * @urb_buffer_size: size of the URB buffer
 * @min_expected_size: minimum size for valid data
 */
static int map_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, void *user_buffer, size_t buffer_size, size_t offset, size_t urb_buffer_size, size_t min_expected_size) {
	return 0;
}

/*
 * u3v_unconfigure_buffer - finds the buffer with buffer_id,
 * unlocks corresponding pagelists, and frees all resources associated
 * with the buffer entry.
 * @stream: pointer to the u3v_stream interface struct
 * @buffer_id: id of the buffer to be destroyed
 */
int u3v_unconfigure_buffer(struct u3v_stream *stream, _Uint64t buffer_id) {
	return 0;
}

/*
 * destroy_buffer - helper function that finds a buffer entry by id,
 *	removes it from the list of buffers, and frees its memory
 * precondition: caller must hold stream_lock
 * @stream: pointer to the u3v_stream interface struct
 * @buffer_id: id of the buffer to be destroyed
 */
static int destroy_buffer(struct u3v_stream *stream, _Uint64t buffer_id) {
	return 0;
}

/*
 * destroy_urb - helper function that handles the destruction of a urb.
 *	If the urb was mapped from user memory, we destroy the page list
 *	and unlock the memory. If it was allocated from kernel memory,
 *	we just free that buffer
 * precondition: caller must hold stream_lock
 * @urb_info: pointer to struct containing a urb pointer and its metadata
 */

static void destroy_urb(struct urb_info *urb_info) {
	if (urb_info == NULL)
		return;

	free(urb_info->purb);

	if (urb_info->kernel_allocated)
		free(urb_info->buffer);

}

/*
 * u3v_queue_buffer - queues the buffer to the camera for it to be filled
 *	with image data
 * @stream: pointer to the stream interface struct
 * @buffer_id: on return, buffer with this id is queued
 */
int u3v_queue_buffer(struct u3v_stream *stream, _Uint64t buffer_id) {
	return 0;
}
/*
* reset_counters - helper function that resets values in a buffer
*	entry so that it can be resubmitted
* precondition: caller must hold stream_lock
* @entry: buffer to be reset
*/
static void reset_counters(struct buffer_entry *entry) {
}

/*
 * submit_stream_urb - queues a buffer to the stream endpoint and
 *	increments the number of outstanding URBS
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @urb_info: pointer to the urb_info struct that contains the
 *	urb to be submitted
 */
static int submit_stream_urb(struct u3v_stream *stream, struct urb_info *urb_info) {
	return 0;
}
