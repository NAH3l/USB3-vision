#include <errno.h>
#include <sys/usbdi.h>
#include <iostream>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"


/*
 * u3v_create_control - Initializes the control interface.
 *
 */
/*
 * u3v_create_control - Initializes the control interface.
 * @u3v: pointer to the u3v_device struct
 */
int u3v_create_control(struct u3v_device *u3v){
	struct u3v_control *ctrl = NULL;
	struct usbd_device *dev = u3v->device;
	uint32_t  bytes_read = 0;
	uint64_t sbrm_address = 0;
	uint32_t max_response;
	uint32_t cmd_buffer_size;
	uint32_t ack_buffer_size;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	if (!u3v->device_connected)
		return -ENODEV;

	if (u3v->control_info.interface_ptr != NULL) {
		//dev_err(u3v->device,"%s: Control interface already created\n", __func__);
		return U3V_ERR_INTERNAL;
	}		

	if (u3v->control_info.bulk_in == NULL || u3v->control_info.bulk_out == NULL) {
		//dev_err(u3v->device, "%s: Did not detect bulk in and bulk out endpoints in the control interface.\n", __func__);
		return U3V_ERR_NO_CONTROL_INTERFACE;
	}	

	ctrl = (struct u3v_control *)malloc(sizeof(struct u3v_control));
	if (ctrl == NULL) {
		//dev_err(u3v->device, "%s: Failed to allocate memory for control interface\n", __func__);
		return -ENOMEM;
	}

	ctrl->u3v_dev = u3v;

	pthread_mutex_init(&ctrl->read_write_lock, NULL);

	ctrl->u3v_timeout = U3V_TIMEOUT;

	ctrl->max_ack_transfer_size = min(((1 << 16) + sizeof(struct ack_header)),(size_t)0xFFFFFFFF);	

	ctrl->max_cmd_transfer_size = min(((1 << 16) + sizeof(struct command_header)), (size_t)0xFFFFFFFF);

	/* id is preincremented so we want it to start at 0 */ 
	ctrl->request_id = -1;

	/* Set the maximum id value so that we can skip 0 once we overflow */
	ctrl->max_request_id = -1;

	/* Allocate memory for the command and ack buffers */
	ctrl->ack_buffer = (uint8_t *)malloc(ctrl->max_ack_transfer_size);
	ctrl->cmd_buffer = (uint8_t *)malloc(ctrl->max_cmd_transfer_size);

	if (ctrl->ack_buffer == NULL || ctrl->cmd_buffer == NULL) {
		//dev_err(u3v->device, "%s: Failed to allocate memory for control interface buffers\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

//	if (!u3v->stalling_disabled && u3v->u3v_info->legacy_ctrl_ep_stall_enabled)
//		reset_pipe(u3v, &u3v->control_info);

    /*
	* Query the device for max response time, and max buffer sizes,
	* updating control attributes if needed
	*/

	ret = u3v_read_mem(ctrl, sizeof(uint32_t), &bytes_read, ABRM_MAX_DEVICE_RESPONSE_TIME, &max_response, NULL, URB_DIR_IN);

	if (ret < 0) {
		//dev_err(u3v->device, "%s: Failed to read max response time from device\n", __func__);
		goto error;
	}

	ctrl->u3v_timeout = max((uint32_t)(U3V_TIMEOUT), ENDIAN_LE32(max_response));

	ret = u3v_read_mem(ctrl, sizeof(uint64_t), &bytes_read, ABRM_SBRM_ADDRESS, &sbrm_address,NULL, URB_DIR_IN);

	if (ret < 0) {
		//dev_err(u3v->device, "%s: Failed to read SBRM address from device\n", __func__);
		goto error;
	}
	
	ret = u3v_read_mem(ctrl, sizeof(uint32_t), &bytes_read, sbrm_address + SBRM_MAX_CMD_TRANSFER, &cmd_buffer_size, NULL, URB_DIR_IN);

	if (ret < 0) {
		//dev_err(u3v->device, "%s: Failed to read max command buffer size from device\n", __func__);
		goto error;
	}
	
	ctrl->max_cmd_transfer_size = min(ctrl->max_cmd_transfer_size, ENDIAN_LE32(cmd_buffer_size));

	ret = u3v_read_mem(ctrl, sizeof(uint32_t), &bytes_read, sbrm_address + SBRM_MAX_ACK_TRANSFER, &ack_buffer_size, NULL, URB_DIR_IN);

	if (ret < 0) {
		//dev_err(u3v->device, "%s: Failed to read max ack buffer size from device\n", __func__);
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
void u3v_destroy_control(struct u3v_device *u3v){
	struct u3v_control *ctrl;

	if (u3v == NULL)
		return;

	ctrl = (struct u3v_control *)(u3v->control_info.interface_ptr);
	if (ctrl == NULL)
		return;

	wait_for_completion(u3v->control_info.ioctl_complete);

	if (!u3v->stalling_disabled)
//		reset_pipe(u3v, &u3v->control_info);

	free(ctrl->ack_buffer);
	free(ctrl->cmd_buffer);
	free(ctrl);
	u3v->control_info.interface_ptr = NULL;
}


/*
 * u3v_read_mem - ioctl to read device registers.
 * @ctrl: pointer to control interface
 * @transfer_size: number of bytes to read
 * @bytes_read: number of bytes read
 * @address: camera's memory address to be read
 * @buffer: kernel buffer to be read into
 */
int u3v_read_mem(struct u3v_control *ctrl, uint32_t transfer_size, uint32_t *bytes_read, uint64_t address, void *kernel_buffer, void *user_buffer, uint32_t flags){
	struct u3v_device *u3v =  ctrl->u3v_dev;
	struct usbd_device *dev = u3v->device;
	struct ack *ack = NULL;
	struct pending_ack_payload *pending_ack = NULL;
	uint32_t  actual = 0;
	const int max_bytes_per_read = ctrl->max_ack_transfer_size - sizeof(struct ack_header);
	int ret = 0;
	int total_bytes_read = 0;
	bool request_acknowledged = false;
	size_t cmd_buffer_size = sizeof(struct command_header) + sizeof(struct read_mem_cmd_payload);
	size_t ack_buffer_size = sizeof(struct ack_header) + transfer_size;

	if (bytes_read != NULL)
		*bytes_read = 0;

	if (transfer_size == 0)
		return 0;

	if (ctrl == NULL)
		return U3V_ERR_NO_CONTROL_INTERFACE;

	if (kernel_buffer == NULL && user_buffer == NULL) {
		//dev_err(dev, "%s: No valid buffer provided to read_memory\n", __func__);
		return -EINVAL;
	}

	if (kernel_buffer != NULL && user_buffer != NULL) {
		//dev_err(dev, "%s: Must provide either a kernel_buffer or a user buffer, not both\n", __func__);
		return -EINVAL;
	}

	if (cmd_buffer_size > ctrl->max_cmd_transfer_size) {
		//dev_err(dev, "%s: Requested command buffer of size %zu, but maximum size is %d\n",__func__, cmd_buffer_size, ctrl->max_cmd_transfer_size);
		return -EINVAL;
	}

	if (cmd_buffer_size > ctrl->max_cmd_transfer_size) {
		//dev_err(dev, "%s: Requested command buffer of size %zu, but maximum size is %d\n", __func__, cmd_buffer_size, ctrl->max_cmd_transfer_size);
		return -EINVAL;
	}	

	if (max_bytes_per_read <= 0) { 
		//dev_err(dev, "%s: Requested ack buffer of size <= 0\n", __func__);
		return -EINVAL;
	}

	pthread_mutex_lock(&ctrl->read_write_lock);

	while (total_bytes_read < static_cast<int>(transfer_size)){
		uint32_t bytes_this_iteration = min(transfer_size - total_bytes_read, (uint32_t)max_bytes_per_read);

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

		ret = usb_bulk_msg(u3v->udev, (usbd_pipe*)usb_sndbulkpipe(u3v->udev,(usbd_pipe*)usb_endpoint_num(u3v->control_info.bulk_out)), ctrl->cmd_buffer , cmd_buffer_size, &actual, ctrl->u3v_timeout);

		if (ret < 0) {
			//dev_err(dev, "%s: Error %d from usb_bulk_msg out\n", __func__, ret);
			if (!u3v->stalling_disabled)
//				reset_pipe(u3v, &u3v->control_info);
			goto exit;
		}
		while (!request_acknowledged) {
			/* reset */
			actual = 0;
			ack_buffer_size = sizeof(struct ack_header) + bytes_this_iteration;
			memset(ctrl->ack_buffer, 0, ack_buffer_size);

			/* read the ack */
			ret = usb_bulk_msg(u3v->udev, (usbd_pipe*)usb_sndbulkpipe(u3v->udev,(usbd_pipe*)usb_endpoint_num(u3v->control_info.bulk_in)), ctrl->ack_buffer , ack_buffer_size, &actual, ctrl->u3v_timeout);

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
			if ((actual < static_cast<int>(sizeof(struct ack_header))) || (ack->header.prefix != U3V_CONTROL_PREFIX) || (static_cast<int>(actual) != static_cast<int>(ack->header.length) + static_cast<int>(sizeof(struct ack_header)))) {
				ret = U3V_ERR_INVALID_DEVICE_RESPONSE;
				goto exit;
			}
			/*
			 * Fix for broken Basler cameras where they can get in a
			 * state where there is an extra bogus response on the
			 * pipe that needs to be thrown away. We just submit
			 * another read in that case.
			 */
			if (ack->header.ack_id == (ctrl->request_id - 1))
				continue;

			/* Inspect the acknowledge buffer */
			if (((ack->header.cmd != READMEM_ACK) && (ack->header.cmd != PENDING_ACK)) || (ack->header.status != U3V_ERR_NO_ERROR) || (ack->header.ack_id != ctrl->request_id) || ((ack->header.cmd == READMEM_ACK) && (ack->header.length != bytes_this_iteration)) || ((ack->header.cmd == PENDING_ACK) && (ack->header.length != sizeof(struct pending_ack_payload)))) {
				std::cout << __func__ << ": received an invalid READMEM_ACK buffer" << std::endl;
				std::cout << "\tReceived bytes = " << actual << ", expected " << ack_buffer_size << std::endl;
				std::cout << "\tPrefix = 0x" << std::hex << ack->header.prefix << ", expected 0x" << U3V_CONTROL_PREFIX << std::endl;
				std::cout << "\tCmd = 0x" << std::hex << ack->header.cmd << ", expected 0x" << READMEM_ACK << " or 0x" << PENDING_ACK << std::endl;
				std::cout << "\tStatus = 0x" << std::hex << ack->header.status << ", expected 0x" << U3V_ERR_NO_ERROR << std::endl;
				std::cout << "\tAck Id = 0x" << std::hex << ack->header.ack_id << ", expected 0x" << ctrl->request_id << std::endl;
				std::cout << "\tPayload length = " << ack->header.length << ", expected " << (ack->header.cmd == PENDING_ACK ? sizeof(struct pending_ack_payload) : bytes_this_iteration) << std::endl;
			}
			/*
			 * For a pending ack, update the timeout and resubmit
			 * the read request
			 */
			if (ack->header.cmd == PENDING_ACK) {
				//dev_dbg(dev, "%s: received pending ack, resubmitting\n",__func__);
				pending_ack = (struct pending_ack_payload *)(ack->payload);
				ctrl->u3v_timeout = max(ctrl->u3v_timeout,(_Uint32t)(ENDIAN_LE16(pending_ack->timeout)));
				continue;
			}

			/* Acknowledge received successfully */
			request_acknowledged = true;
		}

		total_bytes_read += bytes_this_iteration;
	}

	if (total_bytes_read != static_cast<int>(transfer_size))
		std::cout << __func__ << ": total_bytes != xfer: total is " << total_bytes_read << " and xfer is " << transfer_size << std::endl;

	/* Extract the data */
	if (kernel_buffer != NULL)
		memcpy(kernel_buffer, ack->payload, total_bytes_read);
	if (user_buffer != NULL)
		memcpy(user_buffer, ack->payload, total_bytes_read);

exit:
	if (bytes_read != NULL)
		*bytes_read = total_bytes_read;

	pthread_mutex_unlock(&ctrl->read_write_lock);
	return ret;
}

/*
 * u3v_write_mem - ioctl to write device registers
 * @ctrl: pointer to control interface
 * @transfer_size: number of bytes to write
 * @bytes_written: number of bytes written
 * @address: camera's memory address to be written to
 * @kernel_buffer: kernel buffer to be written from, can be NULL if
 * user_buffer is not NULL
 * @user_buffer: user buffer to be written from, can be NULL if
 * kernel_buffer is not NULL
 */
int u3v_write_mem(struct u3v_control *ctrl, uint32_t transfer_size, uint32_t *bytes_written, uint64_t address, const void *kernel_buffer, const void *user_buffer, uint32_t flags){
	int ret = 0;
	int total_bytes_written = 0;
	const int max_bytes_per_write = ctrl->max_cmd_transfer_size - (sizeof(struct command_header) + sizeof(struct write_mem_cmd_payload));
	size_t cmd_buffer_size = sizeof(struct command_header) + sizeof(struct read_mem_cmd_payload);
	size_t ack_buffer_size = sizeof(struct ack_header) + sizeof(struct write_mem_ack_payload);
	struct ack *ack = NULL;
	struct pending_ack_payload *pending_ack = NULL;
	struct write_mem_ack_payload *write_mem_ack = NULL;
	uint32_t actual;
	struct u3v_device *u3v =  ctrl->u3v_dev;
	struct usbd_device *dev = u3v->device;
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

	while (total_bytes_written < static_cast<int>(transfer_size)) {
		_Uint32t bytes_this_iteration = min((int)(transfer_size - total_bytes_written), max_bytes_per_write);
		struct command *command = (struct command *)(ctrl->cmd_buffer);
		struct write_mem_cmd_payload *payload = (struct write_mem_cmd_payload *)(command->payload);
		command->header.prefix = ENDIAN_LE32(U3V_CONTROL_PREFIX);
		command->header.flags = ENDIAN_LE16(U3V_REQUEST_ACK);
		command->header.cmd = ENDIAN_LE16(WRITEMEM_CMD);
		command->header.length = ENDIAN_LE16(sizeof(struct write_mem_cmd_payload) + bytes_this_iteration);
		command->header.request_id = ENDIAN_LE16(++(ctrl->request_id));
		payload->address = ENDIAN_LE64(address + total_bytes_written);

		if (kernel_buffer != NULL) {
			memcpy(payload->data, (_Uint8t *)(kernel_buffer) + total_bytes_written, bytes_this_iteration);
		}
		if (user_buffer != NULL) {
			ret = (int)memcpy(payload->data, (_Uint8t *)(user_buffer) + total_bytes_written, bytes_this_iteration);
			if (ret > 0) {
				//dev_err(dev, "copy to user failed\n");
				ret = U3V_ERR_INTERNAL;
				goto exit;
			}
		}
		cmd_buffer_size = sizeof(struct command_header) + sizeof(struct write_mem_cmd_payload) + bytes_this_iteration;
		ret = usb_bulk_msg(u3v->udev, (usbd_pipe*)usb_sndbulkpipe(u3v->udev,(usbd_pipe*)usb_endpoint_num(u3v->control_info.bulk_out)), ctrl->cmd_buffer , cmd_buffer_size, &actual, ctrl->u3v_timeout);

		if (ret < 0) {
			//dev_err(dev, "%s: Error %d from usb_bulk_msg in\n", __func__, ret);
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
			ret = usb_bulk_msg(u3v->udev, (usbd_pipe*)usb_sndbulkpipe(u3v->udev,(usbd_pipe*)usb_endpoint_num(u3v->control_info.bulk_in)), ctrl->ack_buffer , ack_buffer_size, &actual, ctrl->u3v_timeout);

			ack = (struct ack *)(ctrl->ack_buffer);

			/*
			 * Validate that we read enough bytes to process the
			 * header and that this seems like a valid GenCP
			 * response
			 */
			if (ret < 0) {
				//dev_err(dev, "%s: Error %d from usb_bulk_msg in\n",__func__, ret);
				if (!u3v->stalling_disabled)
//					reset_pipe(u3v, &u3v->control_info);
				goto exit;
			}
			if ((actual < static_cast<int>(sizeof(struct ack_header))) || (ack->header.prefix != U3V_CONTROL_PREFIX) || (static_cast<size_t>(actual) != ack->header.length + sizeof(struct ack_header))) {
				ret = U3V_ERR_INVALID_DEVICE_RESPONSE;
				goto exit;
			}

			if (ack->header.ack_id == (ctrl->request_id - 1))
				continue;

			write_mem_ack = (struct write_mem_ack_payload *)(ack->payload);

			/* Inspect the acknowledge buffer */
			if (((ack->header.cmd != WRITEMEM_ACK) && (ack->header.cmd != PENDING_ACK)) || (ack->header.status != U3V_ERR_NO_ERROR) || (ack->header.ack_id != ctrl->request_id) || ((ack->header.cmd == WRITEMEM_ACK) && (ack->header.length != sizeof(struct write_mem_ack_payload)) && (ack->header.length != 0)) || ((ack->header.cmd == PENDING_ACK) && (ack->header.length != sizeof(struct pending_ack_payload))) || ((ack->header.cmd == WRITEMEM_ACK) && (ack->header.length == sizeof(struct write_mem_ack_payload)) && (write_mem_ack->bytes_written != bytes_this_iteration))) {
				std::cout << __func__ << ": received an invalid WRITEMEM_ACK buffer" << std::endl;
				std::cout << "\tReceived bytes = " << actual << ", expected " << ack_buffer_size << std::endl;
				std::cout << "\tPrefix = 0x" << std::hex << ack->header.prefix << ", expected 0x" << U3V_CONTROL_PREFIX << std::endl;
				std::cout << "\tCmd = 0x" << std::hex << ack->header.cmd << ", expected 0x" << WRITEMEM_ACK << " or 0x" << PENDING_ACK << std::endl;
				std::cout << "\tStatus = 0x" << std::hex << ack->header.status << ", expected 0x" << U3V_ERR_NO_ERROR << std::endl;
				std::cout << "\tAck Id = 0x" << std::hex << ack->header.ack_id << ", expected 0x" << ctrl->request_id << std::endl;

				if (ack->header.cmd == WRITEMEM_ACK) {
					//dev_err(dev, "\tPayload Length = %d, expected %zu\n",ack->header.length, sizeof(struct write_mem_ack_payload));
					//dev_err(dev, "\tBytes Written = %d, expected %d\n",write_mem_ack->bytes_written, bytes_this_iteration);
				}
				if (ack->header.cmd == PENDING_ACK) {
					//dev_err(dev, "\tPayload Length = %d, expected %zu\n", ack->header.length, sizeof(struct pending_ack_payload));
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
				//dev_dbg(dev, "%s: received pending ack, resubmitting\n", __func__);
				pending_ack = (struct pending_ack_payload *)(ack->payload);
				ctrl->u3v_timeout = max(ctrl->u3v_timeout,(_Uint32t)(ENDIAN_LE32(pending_ack->timeout)));
				continue;
			}

			/* Acknowledge received successfully */
			request_acknowledged = true;
		}

		total_bytes_written += bytes_this_iteration;
	}

	if (static_cast<size_t>(total_bytes_written) != transfer_size)
		//dev_err(dev, "%s: wrote %d bytes, but expected %d", __func__, total_bytes_written, transfer_size);

exit:
	if (bytes_written != NULL)
		*bytes_written = total_bytes_written;

	pthread_mutex_unlock(&ctrl->read_write_lock);
	return ret;
}



