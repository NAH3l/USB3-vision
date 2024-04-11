#ifndef _U3V_H_
#define _U3V_H_

#include <errno.h>
#include <sys/usbdi.h>
#include <iostream>
#include <sys/iofunc.h>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"

/* General device info */
#define DRIVER_DESC 					"USB3 Vision Driver"
#define U3V_DEVICE_CLASS 				0xEF
#define U3V_DEVICE_SUBCLASS 			0x02
#define U3V_DEVICE_PROTOCOL 			0x01
#define U3V_INTERFACE_CLASS 			0xEF
#define U3V_INTERFACE_SUBCLASS 			0x05
#define U3V_INTERFACE_PROTOCOL_CONTROL 	0x00
#define U3V_INTERFACE_PROTOCOL_EVENT 	0x01
#define U3V_INTERFACE_PROTOCOL_STREAM 	0x02
#define U3V_INTERFACE 					0x24
#define U3V_DEVICEINFO 					0x01
#define MIN_U3V_INFO_LENGTH 			20
#define U3V_MINOR_BASE 					208
#define U3V_MAX_STR 					64
#define U3V_REQUEST_ACK 				0x4000
#define U3V_TIMEOUT 					5000
#define	U3V_DEV_DISCONNECTED 			1


struct completion {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    bool complete;
};

struct u3v_interface_info {
	usbd_endpoint_descriptor_t *bulk_in;
	usbd_endpoint_descriptor_t *bulk_out;
	struct usbd_urb *urb;
	struct completion *ioctl_complete;
	void *interface_ptr;
	pthread_mutex_t ioctl_count_lock;
	pthread_mutex_t interface_lock;
	int ioctl_count;
	uint8_t idx;
};

/*
 * A copy of this structure exists for each U3V device to contain its
 * private data.
 */
struct u3v_device {
	struct u3v_interface_info control_info;
	struct u3v_interface_info event_info;
	struct u3v_interface_info stream_info;
	struct usbd_pipe* control_pipe;
	struct usbd_device* udev;
	struct usb_interface *intf;
	struct usbd_device *device;
	struct u3v_device_info *u3v_info; /* device attributes */
	struct usb_driver *u3v_driver;
	int device_state;
	bool stalling_disabled;
	bool device_connected;

};

/*
 * Each u3v_device contains a u3v_device_info struct to store
 * device attributes
 */
struct u3v_device_info {
	uint32_t gen_cp_version;
	uint32_t u3v_version;
	char device_guid[U3V_MAX_STR];
	char vendor_name[U3V_MAX_STR];
	char model_name[U3V_MAX_STR];
	char family_name[U3V_MAX_STR];
	char device_version[U3V_MAX_STR];
	char manufacturer_info[U3V_MAX_STR];
	char serial_number_u3v[U3V_MAX_STR];
	char user_defined_name[U3V_MAX_STR];
	uint8_t speed_support;
	uint8_t previously_initialized;
	uint32_t host_byte_alignment;
	uint32_t os_max_transfer_size;
	uint64_t sirm_addr;
	uint32_t transfer_alignment;
	uint32_t segmented_xfer_supported;
	uint32_t segmented_xfer_enabled;
	uint32_t legacy_ctrl_ep_stall_enabled;
};


/* Helper functions for interfaces */
int min	(size_t a, size_t b);
int max(size_t a, size_t b);

/* Function for usb transfer */
int usb_bulk_msg(struct usbd_device *usb_dev, struct usbd_pipe *pipe, void *data, uint32_t len, uint32_t *actual_length, int timeout);
int usb_control_msg(struct usbd_device *dev, struct usbd_pipe *pipe, uint8_t request, uint8_t requesttype, uint16_t value, uint16_t index, void *data, uint16_t size, int timeout);
unsigned int usb_sndbulkpipe(struct usbd_device *dev, struct usbd_pipe *pipe);
unsigned int usb_rcvbulkpipe(struct usbd_device *dev, unsigned int endpoint);
unsigned int usb_sndctrlpipe(struct usbd_device *dev, unsigned int endpoint);
int usb_endpoint_num(const struct usbd_endpoint_descriptor *epd);
int usb_clear_halt(struct usbd_device *dev, struct usbd_pipe *pipe);

int wait_for_completion(struct completion *comp);
void init_completion(struct completion *comp);
void complete_all(struct completion *comp);
void init_usb_anchor(struct usb_anchor *anchor);
#endif /*  _U3V_H_ */
