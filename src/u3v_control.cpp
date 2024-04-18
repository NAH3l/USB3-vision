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

/*
 * u3v_create_control - Initializes the control interface.
 * @u3v: pointer to the u3v_device struct
 */
int u3v_create_control(struct u3v_device *u3v) {
	struct u3v_control *ctrl = NULL;
	struct device *dev = u3v->device;
	_Uint32t bytes_read = 0;
	_Uint64t sbrm_address = 0;
	_Uint32t max_response;
	_Uint32t cmd_buffer_size;
	_Uint32t ack_buffer_size;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	if (!u3v->device_connected)
		return -ENODEV;

	if (u3v->control_info.interface_ptr != NULL) {
		//dev_err(u3v->device, "%s: Control interface already created\n", __func__);
		return U3V_ERR_INTERNAL;
	}

	if (u3v->control_info.bulk_in == NULL || u3v->control_info.bulk_out == NULL) {
		//dev_err(u3v->device, "%s: Did not detect bulk in and bulk out endpoints in the control interface.\n", __func__);
		return U3V_ERR_NO_CONTROL_INTERFACE;
	}

	ctrl = (struct u3v_control *)malloc(sizeof(struct u3v_control));
	if (ctrl == NULL)
		return -ENOMEM;

	ctrl->u3v_dev = u3v;

	pthread_mutex_init(&ctrl->read_write_lock, NULL);

	ctrl->u3v_timeout = U3V_TIMEOUT;

	ctrl->max_ack_transfer_size = min(((1 << 16) + sizeof(struct ack_header)), (size_t)0xFFFFFFFF);

	ctrl->max_cmd_transfer_size = min(((1 << 16) + sizeof(struct command_header)), (size_t)0xFFFFFFFF);

	/* id is preincremented so we want it to start at 0 */
	ctrl->request_id = -1;

    /* Set the maximum id value so that we can skip 0 once we overflow */
    ctrl->max_request_id = -1;

	ctrl->ack_buffer = (uint8_t *)malloc(ctrl->max_ack_transfer_size);
	ctrl->cmd_buffer = (uint8_t *)malloc(ctrl->max_cmd_transfer_size);

	if (ctrl->ack_buffer == NULL || ctrl->cmd_buffer == NULL) {
		//dev_err(dev, "%s: Error allocating control buffers\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	if (!u3v->stalling_disabled){
//		reset_pipe(u3v, &u3v->control_info);
	}

	/*
	 * Query the device for max response time, and max buffer sizes,
	 * updating control attributes if needed
	 */
	ret = u3v_read_memory(ctrl, sizeof(_Uint32t), &bytes_read, ABRM_MAX_DEVICE_RESPONSE_TIME, &max_response);

	if ((ret < 0) || (bytes_read != sizeof(_Uint32t))) {
		//dev_err(dev, "%s: Error reading max device response time\n", __func__);
		goto error;
	}

	ctrl->u3v_timeout = max((_Uint32t)(U3V_TIMEOUT), ENDIAN_LE32(max_response));

	ret = u3v_read_memory(ctrl, sizeof(_Uint64t), &bytes_read, ABRM_SBRM_ADDRESS, &sbrm_address);

	if ((ret < 0) || (bytes_read != sizeof(_Uint64t))) {
		//dev_err(dev, "%s: Error reading SBRM address\n", __func__);
		goto error;
	}

	ret = u3v_read_memory(ctrl, sizeof(_Uint32t), &bytes_read, sbrm_address + SBRM_MAX_CMD_TRANSFER, &cmd_buffer_size);

	if ((ret < 0) || (bytes_read != sizeof(_Uint32t))) {
		//dev_err(dev, "%s: Error reading maximum command transfer size\n", __func__);
		goto error;
	}

	ctrl->max_cmd_transfer_size = min(ctrl->max_cmd_transfer_size, ENDIAN_LE32(cmd_buffer_size));

	ret = u3v_read_memory(ctrl, sizeof(_Uint32t), &bytes_read, sbrm_address + SBRM_MAX_ACK_TRANSFER, &ack_buffer_size);

	if ((ret < 0) || (bytes_read != sizeof(_Uint32t))) {
		//dev_err(dev, "%s: Error reading maximum ack transfer size\n", __func__);
		goto error;
	}

	ctrl->max_ack_transfer_size = min(ctrl->max_ack_transfer_size, ENDIAN_LE32(ack_buffer_size));

	u3v->control_info.interface_ptr = ctrl;
	return ret;

error:
	free(ctrl->ack_buffer);
	free(ctrl->cmd_buffer);
	free(ctrl);
	return ret;
}

/*
 * u3v_destroy_control - destroys the control interface
 * @u3v: pointer to the u3v_device struct
 */
void u3v_destroy_control(struct u3v_device *u3v) {
    struct u3v_control *ctrl;

    if (u3v == NULL)
        return;

    ctrl = (struct u3v_control *)(u3v->control_info.interface_ptr);
    if (ctrl == NULL)
        return;

    // Wait for ongoing ioctls to complete
    wait_for_completion(u3v->control_info.mutex_ioctl_complete, u3v->control_info.cond_ioctl_complete, u3v->control_info.done_ioctl_complete);

    // Reset pipe if stalling is enabled
    if (!u3v->stalling_disabled) {
        // Keep this line as it is
//        reset_pipe(u3v, &u3v->control_info);
    }

    // Free allocated memory
    free(ctrl->ack_buffer);
    free(ctrl->cmd_buffer);
    free(ctrl);
    u3v->control_info.interface_ptr = NULL;
}

/*
 * u3v_read_memory - ioctl to read device registers.
 * @ctrl: pointer to control interface
 * @transfer_size: number of bytes to read
 * @bytes_read: number of bytes read
 * @address: camera's memory address to be read
 * @buffer: kernel buffer to be read into
 */
int u3v_read_memory(struct u3v_control *ctrl, _Uint32t transfer_size, _Uint32t *bytes_read, _Uint64t address, void *buffer) {

	const int max_bytes_per_read = ctrl->max_ack_transfer_size - sizeof(struct ack_header);
	size_t cmd_buffer_size = sizeof(struct command_header) + sizeof(struct read_mem_cmd_payload);
	size_t ack_buffer_size = sizeof(struct ack_header) + transfer_size;
	struct ack *ack = NULL;
	struct pending_ack_payload *pending_ack = NULL;
	int actual = 0;
	int ret = 0;
	int total_bytes_read = 0;
	struct u3v_device *u3v = ctrl->u3v_dev;
	struct device *dev = u3v->device;
	bool request_acknowledged = false;
	usbd_endpoint_descriptor_t *in_endpoint, *out_endpoint;

    if (buffer == NULL)
            return -EINVAL;


    if (ctrl == NULL)
            return U3V_ERR_NO_CONTROL_INTERFACE;

    if (bytes_read != NULL)
    	*bytes_read = 0;

    if (transfer_size == 0)
            return 0;


	//dev_dbg(dev, "u3v_read_memory: address = %llX, transfer_size = %d", address, transfer_size);

	pthread_mutex_lock(&ctrl->read_write_lock);

    /* Getting endpoint descriptors for IN and OUT endpoints */
    in_endpoint = usbd_endpoint_descriptor(u3v->udev, 0, 0, 0, u3v->control_info.bulk_in->bEndpointAddress, NULL);
    out_endpoint = usbd_endpoint_descriptor(u3v->udev, 0, 0, 0, u3v->control_info.bulk_out->bEndpointAddress, NULL);

    while (total_bytes_read < static_cast<int>(transfer_size)) {

		_Uint32t bytes_this_iteration = min((int)(transfer_size - total_bytes_read), max_bytes_per_read);
        struct command *command = (struct command *)(ctrl->cmd_buffer);
        struct read_mem_cmd_payload *payload = (struct read_mem_cmd_payload *)(command->payload);
        command->header.prefix = ENDIAN_LE32(U3V_CONTROL_PREFIX);
        command->header.flags = ENDIAN_LE16(U3V_REQUEST_ACK);
        command->header.cmd = ENDIAN_LE16(READMEM_CMD);
        command->header.length = ENDIAN_LE16(sizeof(struct read_mem_cmd_payload));
        if (ctrl->request_id + 1 == ctrl->max_request_id) {
                ctrl->request_id = 0;
        }
        command->header.request_id = ENDIAN_LE16(++(ctrl->request_id));
        payload->address = address + total_bytes_read;
        payload->reserved = 0;
        payload->byte_count = ENDIAN_LE16(bytes_this_iteration);

        /* Sending the command using the QNX usb_bulk_msg and qnx_usb_sndbulkpipe */
        ret = usb_bulk_msg(u3v->udev, qnx_usb_sndbulkpipe(u3v->udev, qnx_usb_endpoint_num(out_endpoint)), ctrl->cmd_buffer, cmd_buffer_size,  &actual, ctrl->u3v_timeout);
		if (ret < 0) {
			//dev_err(dev, "%s: Error %d from usb_bulk_msg out\n", __func__, ret);
			if (!u3v->stalling_disabled)
//				reset_pipe(u3v, &u3v->control_info);
			goto exit;
		}

		request_acknowledged = false;
		while (!request_acknowledged) {
			/* reset */
			actual = 0;
            ack_buffer_size = sizeof(struct ack_header) + bytes_this_iteration; max((size_t)(bytes_this_iteration), sizeof(struct pending_ack_payload));
			memset(ctrl->ack_buffer, 0, ack_buffer_size);

			/* read the ack */
	        ret = usb_bulk_msg(u3v->udev, qnx_usb_rcvbulkpipe(u3v->udev, qnx_usb_endpoint_num(in_endpoint)), ctrl->ack_buffer, ack_buffer_size, &actual, ctrl->u3v_timeout);

	        ack = (struct ack *)(ctrl->ack_buffer);

			/*
			 * Validate that we read enough bytes to process the
			 * header and that this seems like a valid GenCP
			 * response
			 */

			if (ret < 0) {
				//dev_err(dev, "%s: Error %d from usb_bulk_msg in\n", __func__, ret);
				if (!u3v->stalling_disabled)
//					reset_pipe(u3v, &u3v->control_info);
				goto exit;
			}

			if ((actual < static_cast<int>(sizeof(struct ack_header))) || (ack->header.prefix != U3V_CONTROL_PREFIX) || (actual != ack->header.length + static_cast<int>(sizeof(struct ack_header)))) {
				ret = U3V_ERR_INVALID_DEVICE_RESPONSE;
				goto exit;
			}

			/*
			 * Fix for broken Basler cameras where they can get in a
			 * state where there is an extra bogus response on the
			 * pipe that needs to be thrown away. We just submit
			 * another read in that case.
			 */

			if (ack->header.ack_id == (ctrl->request_id - 1)){
			}

			/* Inspect the acknowledge buffer */
			if (((ack->header.cmd != READMEM_ACK) & (ack->header.cmd != PENDING_ACK)) || (ack->header.status != U3V_ERR_NO_ERROR) || (ack->header.ack_id != ctrl->request_id) || ((ack->header.cmd == READMEM_ACK) && (ack->header.length != bytes_this_iteration)) || ((ack->header.cmd == PENDING_ACK) && (ack->header.length != sizeof(struct pending_ack_payload)))) {
				std::cout << __func__ << ": received an invalid READMEM_ACK buffer\n";
				std::cout << "\tReceived bytes = " << actual << ", expected " << ack_buffer_size << "\n";
				std::cout << "\tPrefix = 0x" << std::hex << ack->header.prefix << ", expected 0x" << U3V_CONTROL_PREFIX << "\n";
				std::cout << "\tCmd = 0x" << ack->header.cmd << ", expected 0x" << READMEM_ACK << " or 0x" << PENDING_ACK << "\n";
				std::cout << "\tStatus = 0x" << ack->header.status << ", expected 0x" << U3V_ERR_NO_ERROR << "\n";
				std::cout << "\tAck Id = 0x" << ack->header.ack_id << ", expected 0x" << ctrl->request_id << "\n";
				std::cout << "\tPayload length = " << ack->header.length << ", expected " << (ack->header.cmd == PENDING_ACK ? sizeof(struct pending_ack_payload) : bytes_this_iteration) << "\n";
				if (ack->header.status != U3V_ERR_NO_ERROR)
					ret = U3V_ERR_BASE + ack->header.status;
				else
					ret = U3V_ERR_INVALID_DEVICE_RESPONSE;

				goto exit;
			}

			/*
			 * For a pending ack, update the timeout and resubmit
			 * the read request
			 */
			if (ack->header.cmd == PENDING_ACK) {
				//dev_dbg(dev, "%s: received pending ack, resubmitting\n", __func__);
				pending_ack = (struct pending_ack_payload *) (ack->payload);
				ctrl->u3v_timeout = max(ctrl->u3v_timeout, (_Uint32t)(ENDIAN_LE16(pending_ack->timeout)));
				continue;
			}

			/* Acknowledge received successfully */
			request_acknowledged = true;
		}

		total_bytes_read += bytes_this_iteration;
	}

    if (total_bytes_read != static_cast<int>(transfer_size))
		//dev_err(dev, "%s: total_bytes != xfer: total is %d and xfer is %d", __func__, total_bytes_read, transfer_size);

	/* Extract the data */
    memcpy(buffer, ack->payload, total_bytes_read);

exit:
	if (bytes_read != NULL)
			*bytes_read = total_bytes_read;
	pthread_mutex_unlock(&ctrl->read_write_lock);
	return ret;
}

/*
 * u3v_write_memory - ioctl to write device registers
 * @ctrl: pointer to control interface
 * @transfer_size: number of bytes to write
 * @bytes_written: number of bytes written
 * @address: camera's memory address to be written to
 * @buffer: kernel buffer to be written from
 */
int u3v_write_memory(struct u3v_control *ctrl, _Uint32t transfer_size, _Uint32t *bytes_written, _Uint64t address, const void *buffer) {
    int ret = 0;
    int total_bytes_written = 0;
	const int max_bytes_per_write = ctrl->max_cmd_transfer_size - (sizeof(struct command_header) + sizeof(struct write_mem_cmd_payload));
	size_t cmd_buffer_size = sizeof(struct command_header) + sizeof(struct read_mem_cmd_payload);
	size_t ack_buffer_size = sizeof(struct ack_header) + sizeof(struct write_mem_ack_payload);
	struct u3v_device *u3v = ctrl->u3v_dev;
	struct ack *ack = NULL;
	struct pending_ack_payload *pending_ack = NULL;
	struct write_mem_ack_payload *write_mem_ack = NULL;
	int actual;
	struct device *dev = u3v->device;
    usbd_endpoint_descriptor_t *in_endpoint, *out_endpoint;
    bool request_acknowledged = false;

	if (bytes_written != NULL)
		*bytes_written = 0;

	if (transfer_size == 0)
		return 0;

	if (ctrl == NULL)
		return U3V_ERR_NO_CONTROL_INTERFACE;

	if (ack_buffer_size > ctrl->max_ack_transfer_size) {
		//dev_err(dev, "%s: Requested ack buffer of size %zu, but maximum size is %d\n", __func__, ack_buffer_size, ctrl->max_ack_transfer_size);
		ret = -EINVAL;
		goto exit;
	}

	if (max_bytes_per_write <= 0) {
		//dev_err(dev, "%s: Requested cmd buffer of size <= 0\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	//dev_dbg(dev, "%s: write mem: address = %llX, transfer_size = %d", __func__, address, transfer_size);

	pthread_mutex_lock(&ctrl->read_write_lock);

    /* Getting endpoint descriptors for IN and OUT endpoints */
    in_endpoint = usbd_endpoint_descriptor(u3v->udev, 0, 0, 0, u3v->control_info.bulk_in->bEndpointAddress, NULL);
    out_endpoint = usbd_endpoint_descriptor(u3v->udev, 0, 0, 0, u3v->control_info.bulk_out->bEndpointAddress, NULL);

	while (total_bytes_written < static_cast<int>(transfer_size)) {
		_Uint32t bytes_this_iteration = min((int)(transfer_size - total_bytes_written), max_bytes_per_write);
		struct command *command = (struct command *)(ctrl->cmd_buffer);
		struct write_mem_cmd_payload *payload = (struct write_mem_cmd_payload *)(command->payload);
		command->header.prefix = ENDIAN_LE32(U3V_CONTROL_PREFIX);
		command->header.flags = ENDIAN_LE16(U3V_REQUEST_ACK);
		command->header.cmd = ENDIAN_LE16(WRITEMEM_CMD);
		command->header.length = ENDIAN_LE16(sizeof(struct write_mem_cmd_payload) + bytes_this_iteration);
		if (ctrl->request_id + 1 == ctrl->max_request_id) {
			ctrl->request_id = 0;
		}
		command->header.request_id = ENDIAN_LE16(++(ctrl->request_id));
		payload->address = ENDIAN_LE16(address + total_bytes_written);

        memcpy(payload->data, (_Uint8t *)(buffer + total_bytes_written), bytes_this_iteration);

		cmd_buffer_size = sizeof(struct command_header) + sizeof(struct write_mem_cmd_payload) + bytes_this_iteration;

        /* Sending the command using QNX usb_bulk_msg and qnx_usb_sndbulkpipe */
        ret = usb_bulk_msg(u3v->udev, qnx_usb_sndbulkpipe(u3v->udev, qnx_usb_endpoint_num(out_endpoint)), ctrl->cmd_buffer, cmd_buffer_size, &actual, ctrl->u3v_timeout);
		if (ret < 0) {
			//dev_err(dev, "%s: Error %d from usb_bulk_msg out\n",__func__, ret);
			if (!u3v->stalling_disabled)
//				reset_pipe(u3v, &u3v->control_info);
			goto exit;
		}

		request_acknowledged = false;
		while (!request_acknowledged) {
			/* reset */
			actual = 0;
			memset(ctrl->ack_buffer, 0, ack_buffer_size);
			/* read the ack */
	        ret = usb_bulk_msg(u3v->udev, qnx_usb_sndbulkpipe(u3v->udev, qnx_usb_endpoint_num(in_endpoint)), ctrl->ack_buffer, ack_buffer_size, &actual, ctrl->u3v_timeout);

	        ack = (struct ack *)(ctrl->ack_buffer);


			/*
			 * Validate that we read enough bytes to process the
			 * header and that this seems like a valid GenCP
			 * response
			 */
			if (ret < 0) {
				//dev_err(dev, "%s: Error %d from usb_bulk_msg in\n", __func__, ret);
				if (!u3v->stalling_disabled)
//					reset_pipe(u3v, &u3v->control_info);
				goto exit;
			}

			if ((actual < static_cast<int>(sizeof(struct ack_header))) || (ack->header.prefix != U3V_CONTROL_PREFIX) || (actual != static_cast<int>(ack->header.length) + static_cast<int>(sizeof(struct ack_header)))) {
				ret = U3V_ERR_INVALID_DEVICE_RESPONSE;
				goto exit;
			}

			/*
			 * Fix for broken Basler cameras where they can get in a
			 * state where there is an extra bogus response on the
			 * pipe that needs to be thrown away. We just submit
			 * another read in that case.
			 */

			if (ack->header.ack_id == (ctrl->request_id - 1)) {
			}

			write_mem_ack = (struct write_mem_ack_payload *)(ack->payload);

			/* Inspect the acknowledge buffer */
			if (((ack->header.cmd != WRITEMEM_ACK) &&
				(ack->header.cmd != PENDING_ACK)) ||
				(ack->header.status != U3V_ERR_NO_ERROR) ||
				(ack->header.ack_id != ctrl->request_id) ||
				((ack->header.cmd == WRITEMEM_ACK) &&
				(ack->header.length !=
				sizeof(struct write_mem_ack_payload)) &&
				(ack->header.length != 0)) ||
				((ack->header.cmd == PENDING_ACK) &&
				(ack->header.length !=
				sizeof(struct pending_ack_payload))) ||
				((ack->header.cmd == WRITEMEM_ACK) &&
				(ack->header.length ==
				sizeof(struct write_mem_ack_payload)) &&
				(write_mem_ack->bytes_written !=
				bytes_this_iteration))) {

				std::cout << __func__ << ": received an invalid WRITEMEM_ACK buffer\n";
				std::cout << "\tReceived bytes = " << actual << ", expected " << ack_buffer_size << "\n";
				std::cout << "\tPrefix = 0x" << std::hex << ack->header.prefix << ", expected 0x" << U3V_CONTROL_PREFIX << "\n";
				std::cout << "\tCmd = 0x" << ack->header.cmd << ", expected 0x" << WRITEMEM_ACK << " or 0x" << PENDING_ACK << "\n";
				std::cout << "\tStatus = 0x" << ack->header.status << ", expected 0x" << U3V_ERR_NO_ERROR << "\n";
				std::cout << "\tAck Id = 0x" << ack->header.ack_id << ", expected 0x" << ctrl->request_id << "\n";

				if (ack->header.cmd == WRITEMEM_ACK) {
					std::cout << "\tPayload Length = " << ack->header.length << ", expected " << sizeof(struct write_mem_ack_payload) << "\n";
					std::cout << "\tBytes Written = " << write_mem_ack->bytes_written << ", expected " << bytes_this_iteration << "\n";
				}
				if (ack->header.cmd == PENDING_ACK) {
					std::cout << "\tPayload Length = " << ack->header.length << ", expected " << sizeof(struct pending_ack_payload) << "\n";
				}
				if (ack->header.status != U3V_ERR_NO_ERROR)
					ret = U3V_ERR_BASE + ack->header.status;
				else
					ret = U3V_ERR_INVALID_DEVICE_RESPONSE;

				goto exit;
			}

			/*
			 * For a pending ack, update the timeout and resubmit
			 * the read request
			 */
			if (ack->header.cmd == PENDING_ACK) {
				//dev_dbg(dev, "%s: received pending ack, resubmitting\n",__func__);
				pending_ack = (struct pending_ack_payload *)
					(ack->payload);
				ctrl->u3v_timeout = max(ctrl->u3v_timeout, (_Uint32t)(ENDIAN_LE32(pending_ack->timeout)));
				continue;
			}
			/* Acknowledge received successfully */
			request_acknowledged = true;
		}
		total_bytes_written += bytes_this_iteration;
	}
	if (total_bytes_written != static_cast<int>(transfer_size))
		std::cout << __func__ << ": wrote " << total_bytes_written << " bytes, but expected " << transfer_size;

exit:
	if (bytes_written != NULL)
		*bytes_written = total_bytes_written;

	pthread_mutex_unlock(&ctrl->read_write_lock);
	return ret;
}
