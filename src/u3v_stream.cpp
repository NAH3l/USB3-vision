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

/* Internal stream structs and enums */

struct urb_info {
	_Uint32t urb_index;
	struct u3v_stream *stream;
	struct buffer_entry *entry;
	struct usbd_urb *purb;
	bool kernel_allocated;
	struct pg_list *pglist;
	_Uint8t *buffer;
	size_t buffer_size;
	size_t min_expected_size;
};

enum buffer_state {
	u3v_idle, u3v_queued, u3v_complete, u3v_cancelled
};

struct buffer_entry {
	//atomic_t callbacks_received;
	unsigned callbacks_received;
	enum buffer_state state;
	struct urb_info *urb_info_array;
	int urb_info_count;
	_Uint32t incomplete_callbacks_received;
	_Uint32t leader_received_size;
	_Uint32t payload_received_size;
	_Uint32t trailer_received_size;
	void *transfer2_addr;
	_Uint32t transfer2_received_size;
	//struct completion buffer_complete;
    //pthread_mutex_t buffer_entry_lock;
    //pthread_cond_t buffer_complete;
	struct completion *buffer_complete;
    bool complete;
	_Uint64t buffer_id;
	struct usbd_desc_node* node;
	_Uint32t status;
    struct buffer_entry *next;
};

struct pg_list {
	size_t num_bytes;
	size_t num_pages;
	size_t offset;
	//struct page **pages;
	struct syspage_entry  **pages;
};

/*
 * Inline helper function that returns the number of pages in a num_bytes
 * sized chunk of memory starting at addr.
 */
/* static inline int get_num_pages(uintptr_t addr, size_t num_bytes) {
 * // Obtenir la taille d'une page en octets
 * int page_size = getpagesize();
 *
 * // Get page size in bytes
 * unsigned long end = (addr + num_bytes + page_size - 1) / page_size;
 * unsigned long start = addr / page_size;
 * return end - start;
 * }
 */

/* Functions */
static int set_buffer_sizes(struct u3v_stream *stream, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size);
static int create_buffer_entry(struct u3v_stream *stream, void *user_image_buffer, void *user_chunk_data_buffer, _Uint64t *buffer_id);
static int allocate_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, size_t size, size_t min_expected_size);
static int map_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, void *user_buffer, size_t buffer_size, size_t offset, size_t urb_buffer_size, size_t min_expected_size);
static int destroy_buffer(struct u3v_stream *stream, _Uint64t buffer_id);
static void destroy_urb(struct urb_info *urb_info);
static void reset_counters(struct buffer_entry *entry);
static int submit_stream_urb(struct u3v_stream *stream, struct urb_info *urb_info);
//static void stream_urb_completion(struct usbd_urb *purb);
static int reset_stream(struct u3v_stream *stream);
static struct buffer_entry *search_buffer_entries(struct u3v_stream *stream, _Uint64t value);
static void buffer_entry_insert(struct u3v_stream *stream, struct buffer_entry *new_entry);
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
int u3v_create_stream(struct u3v_device *u3v, struct usbd_interface *intf, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size){
	struct u3v_stream *stream = NULL;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	if (u3v->stream_info.bulk_in == NULL) {
		//dev_err(u3v->device, "%s: Did not detect bulk in endpoint in the stream interface.\n",__func__);
		return U3V_ERR_NO_STREAM_INTERFACE;
	}

	if (u3v->u3v_info->sirm_addr == 0) {
		//dev_err(u3v->device, "%s: the SIRM address is 0 - streaming is not supported\n",__func__);
		return U3V_ERR_NO_STREAM_INTERFACE;
	}

	if (u3v->stream_info.interface_ptr != NULL)
		return U3V_ERR_STREAM_ALREADY_CONFIGURED;

	stream = (struct u3v_stream*)malloc(sizeof(struct u3v_stream));
	if (stream == NULL)
		return -ENOMEM;

	pthread_mutex_init(&stream->stream_lock, NULL);
	stream->u3v_dev = u3v;
	stream->wait_in_progress = false;
	stream->next_buffer_id = 0;
 	reset_stream(stream);
	/* set buffer configurations */
	ret = set_buffer_sizes(stream, image_buffer_size, chunk_data_buffer_size, max_leader_size, max_trailer_size, payload_size, payload_count, transfer1_size, transfer2_size);

	if (ret != 0)
			free(stream);
		else
			u3v->stream_info.interface_ptr = stream;

		return ret;
}

/*
 * u3v_destroy_stream - destroys the stream interface
 * @u3v: pointer to the u3v_device struct
 */
int u3v_destroy_stream(struct u3v_device *u3v) {
	struct u3v_stream *stream;
	struct buffer_entry *current, *next;
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
        pthread_mutex_unlock(&stream->stream_lock);
        goto error;
    }

    pthread_mutex_unlock(&stream->stream_lock);
    wait_for_completion(u3v->stream_info.ioctl_complete);
    pthread_mutex_lock(&stream->stream_lock);

    current = stream->root;

    while (current != NULL) {
            next = current->next;
            //pthread_mutex_destroy(&current->buffer_entry_lock);
            //pthread_cond_destroy(&current->buffer_complete);
            destroy_buffer(stream, current->buffer_id); // Appel de destroy_buffer
			free(current); // Libérez la mémoire du buffer
            current = next;
        }

    pthread_mutex_unlock(&stream->stream_lock);

    free(stream);
    u3v->stream_info.interface_ptr = NULL;
    return 0;

error:
	pthread_mutex_unlock(&stream->stream_lock);
    return ret;
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
	struct usbd_device *dev;
	_Uint32t transfer2_data_size;

	if (stream == NULL)
		return -EINVAL;

	u3v = stream->u3v_dev;
	dev = u3v->device;

	//dev_dbg(dev, "%s: image buffer size = %llu\n", __func__, image_buffer_size);
	//dev_dbg(dev, "%s: chunk data buffer size = %llu\n", __func__, chunk_data_buffer_size);
	//dev_dbg(dev, "%s: payload size = %u\n", __func__, payload_size);
	//dev_dbg(dev, "%s: payload count = %u\n", __func__, payload_count); 
	//dev_dbg(dev, "%s: transfer1 = %u\n", __func__, transfer1_size);
	//dev_dbg(dev, "%s: transfer2 = %u\n", __func__, transfer2_size);

	if (image_buffer_size == 0 || max_leader_size == 0 ||
	    max_trailer_size == 0 || payload_size == 0) {
		//dev_err(dev,"%s: leader, trailer, and image buffer sizes cannot be 0\n", __func__);
		//dev_err(dev, "\timage buffer size is %llu\n",	image_buffer_size);
		//dev_err(dev, "\tmax leader buffer size is %u\n",	max_leader_size);
		//dev_err(dev, "\tmax trailer buffer size is %u\n",max_trailer_size);
		//dev_err(dev, "\tpayload transfer buffer size is %u\n",	payload_size);
		return -EINVAL;
	}
	if ((image_buffer_size + chunk_data_buffer_size) < ((payload_size * payload_count) + transfer1_size)) {
		//dev_err(dev, "%s: buffer sizes are too small to hold all of the requested DMA payload data. Total buffer size = %llu, calculated DMA size = %u\n",__func__, image_buffer_size + chunk_data_buffer_size,((payload_size * payload_count) + transfer1_size));
		return -EINVAL;
	}
	/*
	 * Calculate how much data we actually expect to be valid in the
	 * final transfer 2 buffer
	 */
	transfer2_data_size = (image_buffer_size + chunk_data_buffer_size - (payload_size * payload_count) - transfer1_size);

	//dev_dbg(dev, "%s: transfer2_data = %u\n",__func__, transfer2_data_size);

	if (transfer2_data_size > transfer2_size) {
		//dev_err(dev, "%s: final transfer 2 data size (%u) exceeds the size of the buffer (%u)\n", __func__, transfer2_data_size, transfer2_size);
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
	struct usbd_device *dev;
	int ret;

	if (stream == NULL || user_image_buffer == NULL ||
		buffer_id == NULL)
		return -EINVAL;
	dev = stream->u3v_dev->device;

	if ((user_chunk_data_buffer == NULL && stream->config.chunk_data_buffer_size != 0) || (user_chunk_data_buffer != NULL && stream->config.chunk_data_buffer_size == 0)) {
		//dev_err(dev,"%s: Invalid chunk data buffer address and size combination\n",__func__);
		return -EINVAL;
	}
	ret = create_buffer_entry(stream, user_image_buffer, user_chunk_data_buffer, buffer_id);
	if (ret != 0)
		return ret;

	return 0;
}

/*
 * create_buffer_entry  - this function allocates and initializes the buffer
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
	int i;
	int ret;
	size_t image_offset;
	size_t chunk_data_offset;
	struct buffer_entry *entry;
	_Uint8t *user_transfer2_addr;

	entry = (struct buffer_entry*)malloc(sizeof(*entry));

	if (entry == NULL)
		return -ENOMEM;

	/*
	 * We will need URBs for each of the payloads as well as the
	 * leader, trailer, final transfer 1 (if applicable) and final
	 * transfer 2 (if applicable)
	 */
	entry->urb_info_count = stream->config.payload_count + 2;
	if (stream->config.transfer1_size != 0)
		entry->urb_info_count++;
	if (stream->config.transfer2_size != 0)
		entry->urb_info_count++;
	entry->urb_info_array = (struct urb_info*)malloc(sizeof(struct urb_info) * entry->urb_info_count);

	if (entry->urb_info_array == NULL) {
        free(entry);
        return -ENOMEM;
    }

	/* Initialize the common fields of all of the urb_info structs */
	for (i = 0; i < entry->urb_info_count; i++) {
		entry->urb_info_array[i].urb_index = i;
		entry->urb_info_array[i].stream = stream;
		entry->urb_info_array[i].entry = entry;
		entry->urb_info_array[i].purb = usbd_alloc_urb(NULL);
		if (entry->urb_info_array[i].purb == NULL) {
			ret = -ENOMEM;
			goto error;
		}
	}	

	/* Initialize the leader */
	i = 0;
	ret = allocate_urb_buffer(stream, &entry->urb_info_array[i++], stream->config.max_leader_size, sizeof(struct leader_header));
	if (ret != 0)
		goto error;

	/* Initialize the buffer offsets for configuring the URBs */
	image_offset = 0;
	chunk_data_offset = 0;

	while ((static_cast<size_t>(i) < static_cast<size_t>(stream->config.payload_count + 1)) && (image_offset < stream->config.image_buffer_size)) {
		ret = map_urb_buffer(stream, 
							 &entry->urb_info_array[i++], user_image_buffer, stream->config.image_buffer_size,
			                 image_offset, stream->config.payload_size,
			                 stream->config.payload_size);
		if (ret != 0)
			goto error;

		image_offset += stream->config.payload_size;
	}

	/*
	 * If we account for the total image size before getting to
	 * the last payload transfer then switch to the chunk data
	 * buffer for the remaining payload transfer requests
	 */
	while (static_cast<size_t>(i) < stream->config.payload_count + 1) {
		ret = map_urb_buffer(stream, &entry->urb_info_array[i++], user_chunk_data_buffer, stream->config.chunk_data_buffer_size, chunk_data_offset, stream->config.payload_size, stream->config.payload_size);
		if (ret != 0)
			goto error;
		chunk_data_offset += stream->config.payload_size;
	}	

	/*
	 * Configure the final transfer 1 buffer if applicable. If we
	 * haven't already finished requesting all of the image data
	 * then use the image buffer. Otherwise, use the chunk data buffer.
	 */
	if (stream->config.transfer1_size != 0) {
		if (image_offset < stream->config.image_buffer_size) {
			ret = map_urb_buffer(stream, 
								 &entry->urb_info_array[i++], user_image_buffer, stream->config.image_buffer_size,
				                 image_offset, stream->config.transfer1_size,
				                 stream->config.transfer1_size);
			if (ret != 0)
				goto error;
			image_offset += stream->config.transfer1_size;
		} 
		else {
			ret = map_urb_buffer(stream,
				                 &entry->urb_info_array[i++],
				                 user_chunk_data_buffer,
				                 stream->config.chunk_data_buffer_size,
				                 chunk_data_offset,
				                 stream->config.transfer1_size,
				                 stream->config.transfer1_size);
			if (ret != 0)
				goto error;
			chunk_data_offset += stream->config.transfer1_size;
		}
	}

	entry->transfer2_addr = NULL;
	user_transfer2_addr = NULL;	

    /*
	 * Configure the final transfer 2 buffer if applicable. If the
	 * transfer 2 size is not equal to the transfer 2 data size, then
	 * use a scratch buffer. We will copy from the scratch buffer
	 * to the specified address or just transfer the data straight
	 * to the image or chunk data buffer
	 */

	if (stream->config.transfer2_size != 0) {
		if (stream->config.transfer2_size != stream->config.transfer2_data_size) {
			ret = allocate_urb_buffer(stream,
				                      &entry->urb_info_array[i++],
									  stream->config.transfer2_size,
				                      stream->config.transfer2_data_size);
			if (ret != 0)
				goto error;
			
			if (image_offset < stream->config.image_buffer_size) {
				user_transfer2_addr = (_Uint8t *)(user_image_buffer) +image_offset;
				image_offset += stream->config.transfer2_data_size;				
			}	
			else {
				user_transfer2_addr = (_Uint8t *)(user_chunk_data_buffer) + chunk_data_offset;
				chunk_data_offset += stream->config.transfer2_data_size;	
			}			

		/* lock user memory */
			//entry->urb_info_array[i - 1].pglist = create_pg_list(stream, user_transfer2_addr, stream->config.transfer2_size);
		}
		else {
			if (image_offset < stream->config.image_buffer_size) {
				ret = map_urb_buffer(stream, 
									 &entry->urb_info_array[i++], 
									 user_image_buffer, 
									 stream->config.image_buffer_size,
				                     image_offset, 
									 stream->config.transfer2_size,
				                     stream->config.transfer2_size);
				if (ret != 0)
					goto error;
				image_offset += stream->config.transfer2_data_size;					 
			}
			else {
				ret = map_urb_buffer(stream, 
									 &entry->urb_info_array[i++], 
									 user_chunk_data_buffer, 
									 stream->config.chunk_data_buffer_size,
				                     chunk_data_offset, 
									 stream->config.transfer2_size,
				                     stream->config.transfer2_size);
				if (ret != 0)
					goto error;
				chunk_data_offset += stream->config.transfer2_data_size;	
			}	

		}	
	}

	/* Initialize the trailer */
	ret = allocate_urb_buffer(stream, 
	                          &entry->urb_info_array[i], 
							  stream->config.max_trailer_size, 
							  sizeof(struct trailer_header));
	if (ret != 0)
		goto error;		

	if ((i != entry->urb_info_count) || (image_offset != stream->config.image_buffer_size) || (chunk_data_offset != stream->config.chunk_data_buffer_size)) {					 
		//dev_err(dev, "%s: Error configuring buffer entry\n", __func__);
		ret = U3V_ERR_INTERNAL;
		goto error;
	} 

	init_completion(entry->buffer_complete);
	complete_all(entry->buffer_complete);
	atomic_set(&entry->callbacks_received, 0);
	entry->state = u3v_idle;
	entry->buffer_id = stream->next_buffer_id++;
	entry->leader_received_size = 0;
	entry->payload_received_size = 0;
	entry->trailer_received_size = 0;
	entry->status = 0;

	pthread_mutex_lock(&stream->stream_lock);
	buffer_entry_insert(stream, entry);
	pthread_mutex_unlock(&stream->stream_lock);
	return 0;

error:
	for (i = 0; i < entry->urb_info_count; i++)
		usbd_free_urb(entry->urb_info_array[i].purb);
	free(entry->urb_info_array);
	free(entry);
	return ret;	
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

static int allocate_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, size_t size, size_t min_expected_size){
	if (urb_info == NULL)
		return -EINVAL;

	urb_info->buffer = (_Uint8t*)malloc(size);
	if (urb_info->buffer == NULL)
		return -ENOMEM;
	urb_info->kernel_allocated = true;
	//urb_info->pglist = NULL;
	urb_info->buffer_size = size;
	urb_info->min_expected_size = min_expected_size;
	return 0;
}

/*
 * map_urb_buffer - this function locks a segment of user memory.
 * This function is an alternative to allocate_urb_buffer and is used to DMA directly into the user buffer without making a copy.
 * @stream: pointer to the stream interface struct
 * @urb_info: on return this struct will be initialized
 * @user_buffer: user mode memory address of the buffer to lock
 * @buffer_size: size of the user buffer
 * @offset: offset into the user buffer
 * @urb_buffer_size: size of the URB buffer
 * @min_expected_size: minimum size for valid data
 */
static int map_urb_buffer(struct u3v_stream *stream, struct urb_info *urb_info, void *user_buffer, size_t buffer_size, size_t offset, size_t urb_buffer_size, size_t min_expected_size) {
    if (urb_info == NULL || user_buffer == NULL || buffer_size == 0)
        return -EINVAL;

    urb_info->kernel_allocated = false;
    urb_info->buffer = (_Uint8t*)malloc(buffer_size);
    if (urb_info->buffer == NULL)
        return -ENOMEM;
    urb_info->buffer_size = buffer_size;
    urb_info->min_expected_size = min_expected_size;
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
	int ret;

	if (stream == NULL)
		return -EINVAL;

	pthread_mutex_lock(&stream->stream_lock);
	ret = destroy_buffer(stream, buffer_id);
	pthread_mutex_unlock(&stream->stream_lock);

	return ret;
}

/*
 * destroy_buffer - helper function that finds a buffer entry by id,
 * removes it from the list of buffers, and frees its memory
 * precondition: caller must hold stream_lock
 * @stream: pointer to the u3v_stream interface struct
 * @buffer_id: id of the buffer to be destroyed
 */
static int destroy_buffer(struct u3v_stream *stream, _Uint64t buffer_id) {
	struct buffer_entry *entry;
	 struct buffer_entry **current;
	struct usbd_device *dev;
	int i;

	if (stream == NULL)
		return -EINVAL;

	dev = stream->u3v_dev->device;

	entry = search_buffer_entries(stream, buffer_id);

	if (entry == NULL) {
		//dev_err(dev, "%s: Failed to find entry with buffer id %llu\n", __func__, buffer_id);
		return -EINVAL;
	}

	if (entry->state == u3v_queued) {
		//dev_err(dev, "%s: Buffer is in use, cannot be unconfigured\n",__func__);
		return U3V_ERR_BUFFER_STILL_IN_USE;
	}

    // Recherche de l'adresse de l'entrée de buffer dans la liste chaînée
    for (current = &stream->root; *current != NULL; current = &(*current)->next) {
        if (*current == entry) {
            // Suppression de l'entrée de buffer de la liste chaînée
            *current = entry->next;
            break;
        }
    }

	for (i = 0; i < entry->urb_info_count; i++)
		destroy_urb(&entry->urb_info_array[i]);

	free(entry->urb_info_array);
	free(entry);

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
    struct buffer_entry *entry;
    struct usbd_device *dev = stream->u3v_dev->device;
    int ret = 0;
    int i = 0;
    if (stream == NULL)
        return -EINVAL;

    //dev_vdbg(dev, "%s: buffer id = %llu\n", __func__, buffer_id);

    pthread_mutex_lock(&stream->stream_lock);
    entry = search_buffer_entries(stream, buffer_id);

    if (entry == NULL) {
        //dev_err(dev, "%s: Failed to find entry with id %llu\n", __func__, buffer_id);
        ret = U3V_ERR_INTERNAL;
        goto exit;
    }

    if (entry->state == u3v_queued) {
        //dev_err(dev, "%s: Error, buffer %llu is already queued\n", __func__, buffer_id);
        ret = U3V_ERR_INTERNAL;
        goto exit;
    }

    reset_counters(entry);
    entry->state = u3v_queued;
    // Initialiser la complétion du buffer
    init_completion(entry->buffer_complete);

    for (i = 0; i < entry->urb_info_count; i++) {
        ret = submit_stream_urb(stream, &entry->urb_info_array[i]);
        if (ret != 0)
            goto reset_stream_exit;
    }

exit:
    pthread_mutex_unlock(&stream->stream_lock);
    return ret;

reset_stream_exit:
    entry->state = u3v_cancelled;
    // Signaler l'achèvement à tous les threads en attente
    complete_all(entry->buffer_complete);
    reset_stream(stream);
    pthread_mutex_unlock(&stream->stream_lock);
    return ret;
}	


/*
 * reset_counters - helper function that resets values in a buffer
 *	entry so that it can be resubmitted
 * precondition: caller must hold stream_lock
 * @entry: buffer to be reset
 */
static void reset_counters(struct buffer_entry *entry) {
	entry->state = u3v_idle;
	entry->incomplete_callbacks_received = 0;
	entry->leader_received_size = 0;
	entry->payload_received_size = 0;
	entry->trailer_received_size = 0;
	entry->status = 0;
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

	struct usbd_device *udev;
	int ret;

	if (stream == NULL || urb_info == NULL || urb_info->purb == NULL)
		return -EINVAL;

	udev = stream->u3v_dev->udev;



	if (urb_info->kernel_allocated && (urb_info->buffer == NULL)) {
		//dev_err(dev, "%s: NULL buffer for kernel URB\n", __func__);
		return -EINVAL;
	}

    /* Initialiser l'URB pour le transfert en vrac */
    ret = usbd_setup_bulk(urb_info->purb, URB_DIR_IN, urb_info->buffer, urb_info->buffer_size);
    if (ret != EOK) {
        //dev_err(stream->u3v_dev->device, "%s: Error setting up bulk URB\n", __func__);
        return ret;
    }

    /* Soumettre l'URB */
    ret = usbd_io(urb_info->purb, (usbd_pipe*)usb_endpoint_num(stream->u3v_dev->stream_info.bulk_in), NULL, udev, USBD_TIME_INFINITY);
    if (ret != EOK) {
        //dev_err(stream->u3v_dev->device, "%s: Error submitting urb\n", __func__);
        return ret;
    }
	return ret;
}

/*
 * stream_urb_completion - This function executes when the transfer
 *	finishes, fails, or is cancelled. It runs in interrupt context,
 *	and keeps track of when a buffer entry has received all of its
 *	expected callbacks. Once the callbacks have all been received, it
 *	signals that it is complete so that wait_for_buffer can finish.
 * @purb: pointer to the urb that has been completed
 */
//static void stream_urb_completion(struct usbd_urb *purb) {}

/*
 * u3v_wait_for_buffer - This function waits for the buffer entry with
 *	the specified id to receive all of its callbacks, then copies
 *	the leader, trailer and buffer complete data into the
 *	provided buffers.
 * @stream: pointer to the stream interface
 * @buffer_id: the buffer to be waited on
 * @user_leader_buffer: if not NULL, the leader information will be copied
 *	to this buffer on return
 * @leader_size: size of user_leader_buffer on input, and number of bytes
 *	copied on return. Can be NULL if user_leader_buffer is NULL
 * @user_trailer_buffer: if not NULL, the trailer information will be
 *	copied to this buffer on return
 * @trailer_size: size of the user_trailer_buffer on input, and number
 *	of bytes copied on return. Can be NULL if user_trailer_buffer is NULL
 * @buffer_complete_data: additional data describing the completed buffer,
 *	can be NULL.
 */

int u3v_wait_for_buffer(struct u3v_stream *stream, _Uint64t buffer_id, void *user_leader_buffer, _Uint32t *leader_size, void *user_trailer_buffer, _Uint32t *trailer_size, void *buffer_complete_data){
	struct buffer_entry *entry;
	struct usbd_device *dev;
	struct buffer_complete_data bcd;
	int ret = 0;
	int leader_index;
	int transfer2_index;
	int trailer_index;
	struct trailer *trailer;

	if (stream == NULL)
		return ret;

	dev = stream->u3v_dev->device;

	//dev_vdbg(dev, "%s: buffer id = %llu\n", __func__, buffer_id);

	pthread_mutex_lock(&stream->stream_lock);

	entry = search_buffer_entries(stream, buffer_id);

	if (entry == NULL) {
		//dev_err(dev, "%s: Failed to find buffer with id %llu\n", __func__, buffer_id);
		ret = -EINVAL;
		goto exit;
	}

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

	if (entry->callbacks_received != (_Uint32t)entry->urb_info_count) {
		stream->wait_in_progress = true;
		pthread_mutex_unlock(&stream->stream_lock);
		ret = wait_for_completion(entry->buffer_complete);
		pthread_mutex_lock(&stream->stream_lock);
		stream->wait_in_progress = false;
	}

	trailer = (struct trailer *)entry->urb_info_array[entry->urb_info_count - 1].buffer;
	//dev_vdbg(dev, "%s: device reports valid payload size is %llu\n",__func__, trailer->header.valid_payload_size);
	//dev_vdbg(dev, "%s: driver reports payload received size is %u\n",__func__, entry->payload_received_size);
	//dev_vdbg(dev, "%s: device reports status %04X\n",__func__, trailer->header.status);

	if (ret != 0) {
		//dev_err(dev, "%s: Error, interrupted by signal\n", __func__);
		ret = U3V_ERR_BUFFER_WAIT_CANCELED;
		goto exit;
	}
	if (entry->state != u3v_complete) {
		//dev_err(dev,"%s: Cannot copy from the requested buffer because it has been requeued or cancelled\n", __func__);
		ret = U3V_ERR_BUFFER_CANNOT_BE_COPIED;
		goto exit;
	}

	/* Fill in the requested information */
	leader_index = 0;
	transfer2_index = entry->urb_info_count - 2;
	trailer_index = entry->urb_info_count - 1;

	if (user_leader_buffer != NULL && leader_size != NULL) {
	    if (entry->leader_received_size <= *leader_size) {
	        memcpy(user_leader_buffer, entry->urb_info_array[leader_index].buffer, entry->leader_received_size);
	    }
	    else {
	        //dev_err(dev, "%s: User buffer size is too small for leader data\n", __func__);
	        return -EINVAL;
	    }
	}

	if (entry->transfer2_addr != NULL) {
	    if (entry->transfer2_received_size <= entry->urb_info_array[transfer2_index].min_expected_size) {
	        memcpy(entry->transfer2_addr, entry->urb_info_array[transfer2_index].buffer, entry->transfer2_received_size);
	    }
	    else {
	        //dev_err(dev, "%s: Transfer2 buffer size is too small for data\n", __func__);
	        return -EINVAL;
	    }
	}

	if (user_trailer_buffer != NULL && trailer_size != NULL) {
	    if (entry->trailer_received_size <= *trailer_size) {
	        memcpy(user_trailer_buffer, entry->urb_info_array[trailer_index].buffer, entry->trailer_received_size);
	    }
	    else {
	        //dev_err(dev, "%s: User buffer size is too small for trailer data\n", __func__);
	        return -EINVAL;
	    }
	}

	if (trailer_size != NULL) {
	    // Copy trailer size directly to user buffer
	    memcpy(trailer_size, &(entry->trailer_received_size), sizeof(entry->trailer_received_size));
	}

	if (buffer_complete_data != NULL) {
	    bcd.structure_size = sizeof(struct buffer_complete_data);
	    bcd.status = entry->status;
	    bcd.expected_urb_count = entry->urb_info_count;
	    bcd.incomplete_urb_count = entry->incomplete_callbacks_received;
	    bcd.payload_bytes_received = entry->payload_received_size;

	    if (bcd.structure_size >= sizeof(struct buffer_complete_data)) {
	        memcpy(buffer_complete_data, &bcd, bcd.structure_size);
	    }
	    else {
	        //dev_err(dev, "%s: Buffer for metadata is too small\n", __func__);
	        return -EINVAL;
	    }
	}

exit:
	pthread_mutex_unlock(&stream->stream_lock);
	return ret;
}

/*
 * u3v_cancel_all_buffers - cancels any URB transactions that are in
 *	progress, which causes queued buffers to be completed.
 * @stream: pointer to the stream interface struct
 */
int u3v_cancel_all_buffers(struct u3v_stream *stream) {
	int ret;

	if (stream == NULL)
		return -EINVAL;

	pthread_mutex_lock(&stream->stream_lock);
	/*
	 * Cancel all urbs that are currently in progress.
	 * on return, all callbacks have been completed
	 */
	ret = reset_stream(stream);
	pthread_mutex_unlock(&stream->stream_lock);

	return ret;
}

/*
 * reset_stream - cancels any outstanding urbs and resets the stream
 *	bulk in endpoint
 * precondition: caller holds stream_lock or has not yet set the stream
 *	interface pointer
 * @stream - pointer to the stream interface struct
 */
static int reset_stream(struct u3v_stream *stream) {
	struct u3v_device *u3v;
	struct usbd_device *udev;
	uint8_t ep_addr;
	int dummy_size = 32;
	uint8_t dummy_buffer[dummy_size];
	uint32_t  actual = 0;
	int ret = 0;

	if (stream == NULL)
		return -EINVAL;

	u3v = stream->u3v_dev;
	if (!u3v->device_connected)
		return 0;

	udev = u3v->udev;
	ep_addr = u3v->stream_info.bulk_in->bEndpointAddress;


	/*
	 * We have had issues with xhci when trying to stall/halt
	 * endpoints, so this is a workaround.
	 */
	if (u3v->stalling_disabled) {
		ret = usb_control_msg(udev, (usbd_pipe*)usb_sndctrlpipe(udev, U3V_CTRL_ENDPOINT), NULL, USB_RECIPIENT_ENDPOINT, NULL, ep_addr, NULL, 0, U3V_TIMEOUT);
		if (ret != 0) {
			//dev_err(u3v->device,"%s: Error %d stalling ep %02X\n",__func__, ret, ep_addr);
			return ret;
		}
		/* submit dummy read */
		usb_bulk_msg(udev, (usbd_pipe*)usb_sndbulkpipe(udev,(usbd_pipe*)usb_endpoint_num(u3v->stream_info.bulk_in)), dummy_buffer , dummy_size, &actual, U3V_TIMEOUT);
		/* clear stall */
		ret = usb_clear_halt(udev, (usbd_pipe*)usb_rcvbulkpipe(udev, ep_addr));
		if (ret != 0) {
			//dev_err(u3v->device, "%s: Error %d clearing halt on ep %02X\n", __func__, ret, ep_addr);
		}
		/* submit another dummy read */
		usb_bulk_msg(udev, (usbd_pipe*)usb_sndbulkpipe(udev,(usbd_pipe*)usb_endpoint_num(u3v->stream_info.bulk_in)), dummy_buffer , dummy_size, &actual, U3V_TIMEOUT);
	}

	/* aborts outstanding urbs, waits for callbacks to complete */
//  usb_kill_anchored_urbs(&stream->stream_anchor);

	if (!u3v->stalling_disabled){
//			ret = reset_pipe(stream->u3v_dev, &u3v->stream_info);
	}

	return ret;
}

/*
 * search_buffer_entries - helper function that traverses the rb tree of
 * buffer entries to find the one whose id matches the provided value.
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @value: id of the buffer to be found
 * @return: pointer to the entry if found, NULL otherwise
 */
static struct buffer_entry *search_buffer_entries(struct u3v_stream *stream, _Uint64t value){
	struct buffer_entry *entry;

	// Remplacez le code de recherche basé sur l'arbre rouge-noir par une boucle
	entry = stream->root;
	while (entry != NULL) {
		if (entry->buffer_id == value)
			return entry;
		entry = entry->next;
	}
	return NULL;
}

/*
 * buffer_entry_insert - helper function that adds a new buffer entry
 *	in the correct spot to balance the tree
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @new: buffer entry to be added to the rb tree
 */
static void buffer_entry_insert(struct u3v_stream *stream, struct buffer_entry *new_buffer_entry) {
    struct buffer_entry *entry;

    // Verifier si la liste est vide
    if (stream->root == NULL) {
        stream->root = new_buffer_entry;
        new_buffer_entry->next = NULL;
        return;
    }

    // Trouver l'emplacement d'insertion dans la liste
    entry = stream->root;
    while (entry->next != NULL && entry->next->buffer_id < new_buffer_entry->buffer_id) {
        entry = entry->next;
    }

    // Insérer le nouvel élément après 'entry'
    new_buffer_entry->next = entry->next;
    entry->next = new_buffer_entry;
}
