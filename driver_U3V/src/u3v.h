#ifndef _U3V_H_
#define _U3V_H_

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
#include <sys/iofunc.h>
#include <sys/dispatch.h>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"
#include "u3v_sysfile.h"

/* Camera info */
#define VENDOR_ID_BAUMER_VCXU          0x2825
#define PRODUCT_ID_BAUMER_VCXU         0x1711
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
  int done;
};

struct u3v_interface_info {
	struct usbd_urb *urb;
	usbd_endpoint_descriptor_t *bulk_in;
	usbd_endpoint_descriptor_t *bulk_out;
	pthread_mutex_t mutex_ioctl_complete;
	pthread_cond_t cond_ioctl_complete;
	int done_ioctl_complete;
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
    /* Resource manager data, hdr must be first! */
    iofunc_attr_t          hdr[UVC_MAX_VS_COUNT];
    int                    resmgr_id[UVC_MAX_VS_COUNT];
    resmgr_io_funcs_t      io_funcs[UVC_MAX_VS_COUNT];
    resmgr_connect_funcs_t connect_funcs[UVC_MAX_VS_COUNT];
    iofunc_funcs_t         ocb_funcs[UVC_MAX_VS_COUNT];
    iofunc_mount_t         io_mount[UVC_MAX_VS_COUNT];
    resmgr_attr_t          rattr[UVC_MAX_VS_COUNT];

    /* Sysfs emulation data */
    struct u3v_sysfs_device* sysfs[UVC_MAX_VS_COUNT];


	struct u3v_interface_info control_info;
	struct u3v_interface_info event_info;
	struct u3v_interface_info stream_info;
	struct usbd_pipe* control_pipe;
	struct usbd_pipe* streaming_pipe;
	struct device *device;
	struct usbd_device *udev;
	struct usbd_interface *intf;
	struct u3v_device_info *u3v_info; /* device attributes */
	//struct usb_driver *u3v_driver;
	bool stalling_disabled;
	bool device_connected;
	int device_state;
	int total_vs_devices;
};

/*
 * Each u3v_device contains a u3v_device_info struct to store
 * device attributes
 */
struct u3v_device_info {

    /* USB data */
    uint16_t vendor_id;
    uint16_t device_id;
    char     vendor_id_str[256];
    char     device_id_str[256];
    char     serial_str[256];
    int      port_speed;

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
	uint8_t previously_initialized;
	uint32_t host_byte_alignment;
	uint32_t os_max_transfer_size;
	uint32_t transfer_alignment;
	uint32_t segmented_xfer_supported;
	uint32_t segmented_xfer_enabled;
	uint32_t legacy_ctrl_ep_stall_enabled;
	uint64_t sirm_addr;
};


/* Helper functions for interfaces */
int min	(size_t a, size_t b);
int max(size_t a, size_t b);

/* Function for usb transfer */
int usb_bulk_msg(struct usbd_device *device, unsigned int pipe, void *data, int len, int *actual_length, int timeout);
int qnx_control_msg(struct usbd_device *device, uint8_t request, uint8_t requesttype, uint16_t value, uint16_t index, void *data, uint16_t size, int timeout);
unsigned int qnx_usb_rcvbulkpipe(struct usbd_device *device, unsigned int endpoint_num);
unsigned int qnx_usb_sndbulkpipe(struct usbd_device *device, unsigned int endpoint_num);
int qnx_usb_endpoint_num(const usbd_endpoint_descriptor_t *epd);


/* Function for synchronisation */
void init_completion(pthread_mutex_t mutex, pthread_cond_t cond, int done);
void wait_for_completion(pthread_mutex_t mutex, pthread_cond_t cond, int done);
void completion_complete_all(pthread_mutex_t mutex, pthread_cond_t cond, int done);

#endif /*  _U3V_H_ */










