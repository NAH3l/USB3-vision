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
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>
#include <math.h>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"
#include "u3v_sysfile.h"

/* device level ioctls and helper functions*/
static int populate_u3v_properties(struct u3v_device *u3v);
/* device initialization */
static int initialize(struct u3v_device *u3v);
static int get_stream_capabilities(struct u3v_device *u3v);

/* device level ioctls and helper functions*/
static int u3v_get_os_max_transfer_size(struct u3v_device *u3v, _Uint32t *os_max);
static int u3v_get_stream_alignment(struct u3v_device *u3v, _Uint32t *alignment);
static int u3v_configure_stream(struct u3v_device *u3v,
								_Uint64t image_buffer_size,
								_Uint64t chunk_data_buffer_size,
								_Uint32t max_urb_size,
								_Uint32t *max_leader_size,
								_Uint32t *max_trailer_size
								);
static int read_stream_registers(struct u3v_device *u3v,
								 _Uint32t *req_leader_size,
								 _Uint32t *req_trailer_size
									);
static int write_stream_registers(struct u3v_device *u3v,
								  _Uint32t max_leader_size,
								  _Uint32t max_trailer_size,
								  _Uint32t payload_size,
								  _Uint32t payload_count,
								  _Uint32t transfer1_size,
								  _Uint32t transfer2_size
								  );
static _Uint64t compatibility_div(_Uint64t dividend, _Uint32t divisor);
static _Uint32t compatibility_mod(_Uint64t dividend, _Uint32t divisor);


/* Macro */
#define GET_INTERFACE(ptr_type, ptr, interface_info)              \
do {                                                                \
    pthread_mutex_lock(&interface_info.interface_lock);             \
    pthread_mutex_lock(&interface_info.ioctl_count_lock);            \
    ptr = (ptr_type)(interface_info.interface_ptr);                  \
    if (interface_info.ioctl_count++ == 0) {                        \
        init_completion(interface_info.mutex_ioctl_complete,         \
                       interface_info.cond_ioctl_complete,          \
                       interface_info.done_ioctl_complete);        \
    }                                                                \
    pthread_mutex_unlock(&interface_info.ioctl_count_lock);           \
    pthread_mutex_unlock(&interface_info.interface_lock);            \
} while (0)

#define PUT_INTERFACE(interface_info)                              \
do {                                                                \
    pthread_mutex_lock(&interface_info.ioctl_count_lock);            \
    if (--interface_info.ioctl_count == 0) {                        \
        completion_complete_all(interface_info.mutex_ioctl_complete, \
                               interface_info.cond_ioctl_complete,  \
                               interface_info.done_ioctl_complete); \
    }                                                                \
    pthread_mutex_unlock(&interface_info.ioctl_count_lock);           \
} while (0)


/*
 * initialize - enumerates and initializes interfaces for this device
 */
static int initialize(struct u3v_device *u3v) {
	int ret = 0;

	pthread_mutex_lock(&u3v->control_info.interface_lock);

	if (u3v->control_info.interface_ptr != NULL) {
		pthread_mutex_unlock(&u3v->control_info.interface_lock);
		return 0;
	}

	ret = u3v_create_control(u3v);

	pthread_mutex_unlock(&u3v->control_info.interface_lock);
	if (ret != 0)
		return ret;

	ret = get_stream_capabilities(u3v);
	/*
	 * get_stream_capabilities only fails if the reads fail.
	 * If streaming is actually not supported, it will still
	 * return 0 and we will not destroy the control interface.
	 * Initialization succeeds, but any calls to create or use
	 * the stream interface will fail.
	 */
	if (ret != 0) {
		pthread_mutex_lock(&u3v->control_info.interface_lock);
		u3v_destroy_control(u3v);
		pthread_mutex_unlock(&u3v->control_info.interface_lock);
	} else {
		u3v->u3v_info->previously_initialized = 1;
	}
	return ret;
}

/*
 * get_stream_capabilities - helper function for initialize that reads
 *	relevant stream registers to store configuration information
 * @u3v: pointer to the u3v_device struct. If stream is supported,
 *	on return sirm_addr and transfer_alignment are set in
 *	u3v_device_info
 */
static int get_stream_capabilities(struct u3v_device *u3v) {
	struct u3v_control *control;
	struct device *dev;
	_Uint64t sbrm_address;
	_Uint64t u3v_capability;
	_Uint32t si_info;
	_Uint32t device_byte_alignment;
	_Uint32t bytes_read;
	int ret;

	/*
	 * Control interface must already be set up so we can read
	 * the stream interface registers
	 */
	if (u3v == NULL || u3v->u3v_info == NULL)
		return -EINVAL;

	GET_INTERFACE(struct u3v_control *, control, u3v->control_info);
	dev = u3v->device;

	/* First get stream interface information */
	ret = u3v_read_memory(control, sizeof(sbrm_address), &bytes_read, ABRM_SBRM_ADDRESS, &sbrm_address);

	if (ret != 0) {
		//dev_err(dev, "%s: Error reading SBRM address\n", __func__);
		goto exit;
	}
	/* Check capabilities to see if SIRM is available */
	ret = u3v_read_memory(control, sizeof(u3v_capability), &bytes_read, sbrm_address + SBRM_U3VCP_CAPABILITY, &u3v_capability);

	if (ret != 0) {
		//dev_err(dev, "%s: Error reading U3VCP capability\n", __func__);
		goto exit;
	}

	if (u3v_capability & SIRM_AVAILABLE_MASK) {
		ret = u3v_read_memory(control, sizeof(u3v->u3v_info->sirm_addr), &bytes_read, sbrm_address + SBRM_SIRM_ADDRESS, &u3v->u3v_info->sirm_addr);

		if (ret != 0) {
			//dev_err(dev, "%s: Error reading SIRM address\n", __func__);
			goto exit;
		}

		ret = u3v_read_memory(control, sizeof(si_info), &bytes_read, u3v->u3v_info->sirm_addr + SI_INFO, &si_info);

		if (ret != 0) {
			//dev_err(dev, "%s: Error reading SI info\n", __func__);
			goto exit;
		}

		device_byte_alignment = 1 << ((si_info & SI_INFO_ALIGNMENT_MASK) >> SI_INFO_ALIGNMENT_SHIFT);

		//u3v->u3v_info->transfer_alignment = LCM(device_byte_alignment, u3v->u3v_info->host_byte_alignment);
		int max_num  = max(device_byte_alignment, u3v->u3v_info->host_byte_alignment);
	    int transfer_alignment = max_num;

	    // Trouver le PPMC en incrémentant transfer_alignment jusqu'à ce qu'il soit divisible par les deux nombres
	    while (true) {
	        if (transfer_alignment % device_byte_alignment == 0 && transfer_alignment % u3v->u3v_info->host_byte_alignment == 0) {
	            break;
	        }
	        ++transfer_alignment;
	    }

	    // Attribuer la valeur du PPMC à u3v->u3v_info->transfer_alignment
	    u3v->u3v_info->transfer_alignment = transfer_alignment;
	    std::cout << "Le PPMC de " << device_byte_alignment << " et " << u3v->u3v_info->host_byte_alignment << " est " << transfer_alignment << std::endl;
	}
exit:
	PUT_INTERFACE(u3v->control_info);
	return ret;
}


usbd_interface_descriptor_t *qnx_ifnum_to_if(struct usbd_device *device, uint8_t config_value, uint8_t ifnum) {
    usbd_configuration_descriptor_t *cfg_desc;
    usbd_interface_descriptor_t *ifc_desc;
    struct usbd_desc_node *node = NULL;
    int i;

    // Obtenir le descripteur de configuration
    cfg_desc = usbd_configuration_descriptor(device, config_value, &node);
    if (cfg_desc == NULL) {
        return NULL; // Gestion d'erreur
    }

    // Itérer sur les interfaces
    for (i = 0; i < cfg_desc->bNumInterfaces; ++i) {
        ifc_desc = usbd_interface_descriptor(device, config_value, i, 0, &node);
        if (ifc_desc == NULL) {
            return NULL; // Gestion d'erreur
        }

        if (ifc_desc->bInterfaceNumber == ifnum) {
            return ifc_desc; // Interface trouvée
        }
    }

    // Interface non trouvée
    return NULL;
}

/*
 * u3v_configure_stream - sets up the stream interface for the device
 * @u3v: pointer to the u3v_device struct
 * @image_buffer_size: the size of the buffers that will hold the image data.
 *	Could include size of both the image data and chunk data if they
 *	are to be acquired into the same buffer. Otherwise, it is just
 *	the size of the image data.
 * @chunk_data_buffer_size: the size of the buffers that will hold only
 *	chunk data. If the chunk data is acquired into the same buffer
 *	as the image data, this size should be 0.
 * @max_urb_size: this parameter can be used to limit the size of the
 *	URBs if necessary. If you want to use the max size supported
 *	by the OS, set this value to uint max
 * @max_leader_size: on return this will contain the size of the
 *	buffer that the caller must allocate to guarantee enough
 *	space to hold all of the leader data. This size will account
 *	for any alignment restrictions
 * @max_trailer_size: on return this will contain the size of the
 *	buffer that the caller must allocate to guarantee enough space
 *	to hold all of the trailer data. This size will account for
 *	any alignment restrictions.
 */
static int u3v_configure_stream(struct u3v_device *u3v, _Uint64t image_buffer_size, _Uint64t chunk_data_buffer_size, _Uint32t max_urb_size, _Uint32t *max_leader_size, _Uint32t *max_trailer_size) {
	struct device *dev;
    _Uint32t req_leader_size = 0;
    _Uint32t req_trailer_size = 0;
    _Uint32t byte_alignment = 0;
    _Uint32t payload_size = 0;
    _Uint32t payload_count = 0;
    _Uint32t transfer1_size = 0;
    _Uint32t transfer2_size = 0;
    _Uint32t aligned_max_transfer_size = 0;
    _Uint32t aligned_cd_buffer_size = 0;
    _Uint32t k_max_leader_size = 0;
    _Uint32t k_max_trailer_size = 0;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	dev = u3v->device;

	ret = read_stream_registers(u3v, &req_leader_size, &req_trailer_size);
	if (ret != 0)
		return ret;

	byte_alignment = u3v->u3v_info->transfer_alignment;

	aligned_max_transfer_size = (max_urb_size / byte_alignment) * byte_alignment;

    // Calculate maximum aligned leader size
	k_max_leader_size = ((req_leader_size + byte_alignment - 1) / byte_alignment) * byte_alignment;
	k_max_leader_size = min(k_max_leader_size, aligned_max_transfer_size);

    if (max_leader_size != NULL) {
        *max_leader_size = k_max_leader_size;
    }

    // Calculate payload size and count
    payload_size = aligned_max_transfer_size;
    payload_count = compatibility_div(image_buffer_size, payload_size);

    // Calculate transfer1 size for remaining image data
	transfer1_size = (compatibility_mod(image_buffer_size, payload_size) / byte_alignment) * byte_alignment;
    // Calculate transfer2 size for final alignment
    transfer2_size = compatibility_mod(image_buffer_size, byte_alignment) == 0 ? 0 : byte_alignment;

    // Calculate maximum aligned trailer size
	k_max_trailer_size = ((req_trailer_size + byte_alignment - 1) / byte_alignment) * byte_alignment;
    k_max_trailer_size = min(k_max_trailer_size, aligned_max_transfer_size);

    if (max_trailer_size != NULL) {
        *max_trailer_size = k_max_trailer_size;
    }

	if (max_trailer_size != NULL) {
		ret = k_max_trailer_size;
		if (ret != 0) {
			//dev_err(dev,"%s: Error copying max trailer size to user\n", __func__);
			return ret;
		}
	}


	/* now that the buffer sizes are configured, create the stream */
	ret = u3v_create_stream(u3v,
							qnx_ifnum_to_if(u3v->udev, 0, u3v->stream_info.idx),
							image_buffer_size,
							chunk_data_buffer_size,
							k_max_leader_size,
							k_max_trailer_size,
							payload_size,
							payload_count,
							transfer1_size,
							transfer2_size
							);

	if (ret != 0)
		return ret;

	ret = write_stream_registers(u3v,
								 k_max_leader_size,
								 k_max_trailer_size, payload_size, payload_count,
								 transfer1_size, transfer2_size);

	if (ret != 0) {
		/* caller already holds stream_info->interface_lock */
		u3v_destroy_stream(u3v);
	}
	return ret;
}

















/*
 * read_stream_registers - helper function for configure_stream that
 *	reads the required leader and trailer sizes from the device
 * @u3v: pointer to the struct u3v_device
 * @req_leader_size: on successful return, this will point to the
 *	value for the device's required leader size
 * @req_trailer_size: on successful return, this will point to the
 *	value for the device's required trailer size
 */
static int read_stream_registers(struct u3v_device *u3v, _Uint32t *req_leader_size, _Uint32t *req_trailer_size) {
	struct u3v_control *control;
	struct device *dev;
	_Uint32t leader_size = 0;
	_Uint32t trailer_size = 0;
	_Uint32t bytes_read; bytes_read;
	int ret;

	if (u3v == NULL)
		return -EINVAL;

	dev = u3v->device;
	GET_INTERFACE(struct u3v_control *, control, u3v->control_info);

	/*
	 * Read the required size information. If device reports 0
	 * for the required leader and trailer size, we set it to
	 * at least 1024 bytes
	 */

	ret = u3v_read_memory(control, sizeof(leader_size), &bytes_read, u3v->u3v_info->sirm_addr + SI_REQ_LEADER_SIZE, req_leader_size);
	if (ret != 0) {
		//dev_err(dev, "%s: Error reading required leader size\n", __func__);
		goto exit;
	}

	leader_size = max(leader_size, (_Uint32t)(1024));

	ret = u3v_read_memory(control, sizeof(trailer_size), &bytes_read, u3v->u3v_info->sirm_addr + SI_REQ_TRAILER_SIZE, &trailer_size);
	if (ret != 0) {
		//dev_err(dev, "%s: Error reading required trailer size\n", __func__);
	}
	trailer_size = max(trailer_size, (_Uint32t)(1024));

	*req_leader_size = leader_size;
	*req_trailer_size = trailer_size;

exit:
	PUT_INTERFACE(u3v->control_info);
	return ret;
}

/*
 * write_stream_registers - helper function for configure_stream that
 *	writes the buffer configuration information out to the device's
 *	stream registers. These sizes were validated previously when
 *	we created the stream interface.
 * @u3v: pointer to the u3v_device struct
 * @max_leader_size: the max required size + alignment for the leader buffer
 * @max_trailer_size: the max required size + alignment for the trailer buffer
 * @payload_size: size of each payload buffer
 * @payload_count: number of payload buffers
 * @transfer1_size: size of transfer1 payload buffer
 * @transfer2_size: size of transfer2 payload buffer
 */
static int write_stream_registers(struct u3v_device *u3v, _Uint32t max_leader_size, _Uint32t max_trailer_size, _Uint32t payload_size, _Uint32t payload_count, _Uint32t transfer1_size, _Uint32t transfer2_size) {
	struct u3v_control *control;
	struct device *dev;
	_Uint32t bytes_written;
	int ret;

	if (u3v == NULL)
		return -EINVAL;

	dev = u3v->device;
	GET_INTERFACE(struct u3v_control *, control, u3v->control_info);

	ret = u3v_write_memory(control, sizeof(max_leader_size), &bytes_written, u3v->u3v_info->sirm_addr + SI_MAX_LEADER_SIZE, &max_leader_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(payload_size), &bytes_written, u3v->u3v_info->sirm_addr + SI_PAYLOAD_SIZE, &payload_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(payload_count), &bytes_written, u3v->u3v_info->sirm_addr + SI_PAYLOAD_COUNT,
		&payload_count, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(transfer1_size), &bytes_written, u3v->u3v_info->sirm_addr + SI_TRANSFER1_SIZE, &transfer1_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(transfer2_size), &bytes_written, u3v->u3v_info->sirm_addr + SI_TRANSFER2_SIZE, &transfer2_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(max_trailer_size), &bytes_written, u3v->u3v_info->sirm_addr + SI_MAX_TRAILER_SIZE, &max_trailer_size, NULL);
	if (ret != 0)
		goto error;

	PUT_INTERFACE(u3v->control_info);
	return 0;
error:
	//dev_err(dev, "%s: Error setting stream interface registers\n", __func__);
	PUT_INTERFACE(u3v->control_info);
	return ret;
}

/*
 * u3v_control_msg - ioctl using control endpoint 0.
 *	Does not require the control interface.
 * @udev: pointer to the usb device
 * @request: request code value
 * @requesttype: request type code value
 * @value: message value
 * @index: message index
 * @data: user buffer
 * @size: size of user buffer
 */
int u3v_control_msg(struct usbd_device *udev, _Uint8t request, _Uint8t requesttype, _Uint16t value, void *data, _Uint16t size) {
	int ret;
	void *kdata = malloc(size);
	if (!kdata)
		return -ENOMEM;

	ret = qnx_control_msg(udev, request, requesttype, value, 0, kdata, size, U3V_TIMEOUT);
    if (ret < 0) {
        free(kdata); // Libérer la mémoire en cas d'erreur
        return ret; // Retourner le code d'erreur
    }

    // Copie des données du tampon noyau vers le tampon utilisateur
    memcpy(data, kdata, size);
    free(kdata); // Libérer la mémoire
    return 0; // Succès
}

static int u3v_get_os_max_transfer_size(struct u3v_device *u3v, _Uint32t *os_max) {
	if (u3v == NULL || os_max == NULL)
		return -EINVAL;

	if (u3v->u3v_info->sirm_addr == 0)
		return U3V_ERR_NO_STREAM_INTERFACE;

	*os_max = u3v->u3v_info->os_max_transfer_size;
	return EOK;
}

static int u3v_get_stream_alignment(struct u3v_device *u3v, _Uint32t *alignment) {
    if (u3v == NULL || u3v->u3v_info == NULL || alignment == NULL)
        return -EINVAL;

    if (u3v->u3v_info->sirm_addr == 0)
        return U3V_ERR_NO_STREAM_INTERFACE;

    // Assuming alignment is already calculated and stored in u3v_info
    *alignment = u3v->u3v_info->transfer_alignment;
    return EOK; // Success
}

/*
 * populate_u3v_properties - this helper function copies necessary
 *	USB3 data to our u3v_device_info struct
 * @u3v: pointer to the u3v struct
 */
static int populate_u3v_properties(struct u3v_device *u3v) {
    struct usbd_desc_node *node = NULL;
    unsigned char *buffer;
    int buflen;
    struct usbd_device *udev = u3v->udev;
    struct u3v_device_info *u3v_info = u3v->u3v_info;
    usbd_device_descriptor_t* device_descriptor;  // Use device descriptor

    // Retrieve the device descriptor
    device_descriptor = usbd_device_descriptor(udev, &node);
    if (device_descriptor == NULL) {
        //dev_err(u3v->device, "%s: Failed to get device descriptor.\n", __func__);
        return U3V_ERR_INVALID_USB_DESCRIPTOR;
    }

    // Assuming Camera Info is within the device descriptor
    buffer = (unsigned char*)device_descriptor;
    buflen = device_descriptor->bLength;

    // Validate descriptor type, subtype, and length
    if ((buflen < MIN_U3V_INFO_LENGTH) ||
        (buffer[1] != U3V_INTERFACE) ||
        (buffer[2] != U3V_DEVICEINFO)) {

		//dev_err(u3v->device, "%s: Failed to get a proper Camera Info descriptor.\n", __func__);
		//dev_err(u3v->device, "Descriptor Type = 0x%02X (Expected 0x%02X)\n", buffer[1], U3V_INTERFACE);
		//dev_err(u3v->device, "Descriptor SubType = 0x%02X (Expected 0x%02X)\n",buffer[2], U3V_DEVICEINFO);
		//dev_err(u3v->device, "Descriptor Length = 0x%02X (Expected 0x%02X)\n", buflen, MIN_U3V_INFO_LENGTH);
        return U3V_ERR_INVALID_USB_DESCRIPTOR;
    }

    // Extract and store information from the descriptor
    u3v_info->gen_cp_version = ENDIAN_LE32(*(uint32_t *)(buffer + 3));
    u3v_info->u3v_version = ENDIAN_LE32(*(uint32_t *)(buffer + 7));

    // Retrieve string descriptors using usbd_string
    if (buffer[11] != 0) {
        strncpy(u3v_info->device_guid, usbd_string(udev, buffer[11], 0), sizeof(u3v_info->device_guid));
    }
    if (buffer[12] != 0) {
        strncpy(u3v_info->vendor_name, usbd_string(udev, buffer[12], 0), sizeof(u3v_info->vendor_name));
    }
    if (buffer[13] != 0) {
        strncpy(u3v_info->model_name, usbd_string(udev, buffer[13], 0), sizeof(u3v_info->model_name));
    }
    if (buffer[14] != 0) {
        strncpy(u3v_info->family_name, usbd_string(udev, buffer[14], 0), sizeof(u3v_info->family_name));
    }
    if (buffer[15] != 0) {
        strncpy(u3v_info->device_version, usbd_string(udev, buffer[15], 0), sizeof(u3v_info->device_version));
    }
    if (buffer[16] != 0) {
        strncpy(u3v_info->manufacturer_info, usbd_string(udev, buffer[16], 0), sizeof(u3v_info->manufacturer_info));
    }
    if (buffer[17] != 0) {
        strncpy(u3v_info->serial_number_u3v, usbd_string(udev, buffer[17], 0), sizeof(u3v_info->serial_number_u3v));
    }
    if (buffer[18] != 0) {
        strncpy(u3v_info->user_defined_name, usbd_string(udev, buffer[18], 0), sizeof(u3v_info->user_defined_name));
    }

    u3v_info->port_speed = buffer[19];
    u3v_info->previously_initialized = 0;

    // Set host byte alignment (assuming 4096 for QNX)
    u3v_info->host_byte_alignment = 4096;

    // Set OS max transfer size (assuming UINT_MAX for QNX)
    u3v_info->os_max_transfer_size = UINT_MAX;

    // sirm_addr and transfer_alignment will be initialized later
    u3v_info->sirm_addr = 0;
    u3v_info->transfer_alignment = 0;

    return 0;
}
static int enumerate_u3v_interfaces(struct u3v_device *u3v) {
	struct usbd_device *udev = u3v->udev;
	usbd_interface_descriptor_t *iface_desc;
	int num_interfaces, i;
	int ret = 0;

    if (u3v == NULL)
        return -EINVAL;


}












static _Uint64t compatibility_div(_Uint64t dividend, _Uint32t divisor) {
    div_t result = div(dividend, divisor);
    return result.quot;
}

static _Uint32t compatibility_mod(_Uint64t dividend, _Uint32t divisor) {
    div_t result = div(dividend, divisor);
    return result.rem;
}
