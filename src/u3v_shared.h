#ifndef _U3V_SHARED_H_
#define _U3V_SHARED_H_

#define U3V_ERR_BASE                              0xFFFE0000

/* Error Codes */
#define U3V_ERR_NO_ERROR				          0
#define U3V_ERR_INTERNAL				          (U3V_ERR_BASE + 0x0000)
#define U3V_ERR_INVALID_USB_DESCRIPTOR			  (U3V_ERR_BASE + 0x0001)
#define U3V_ERR_DEVICE_IN_USE				      (U3V_ERR_BASE + 0x0002)
#define U3V_ERR_INVALID_DEVICE_RESPONSE			  (U3V_ERR_BASE + 0x0003)
#define U3V_ERR_NO_CONTROL_INTERFACE			  (U3V_ERR_BASE + 0x0004)
#define U3V_ERR_MEMORY_ACCESS_CANCELED			  (U3V_ERR_BASE + 0x0005)
#define U3V_ERR_MEMORY_ACCESS_COMMAND_TIMEOUT	  (U3V_ERR_BASE + 0x0006)
#define U3V_ERR_MEMORY_ACCESS_ACKNOWLEDGE_TIMEOUT (U3V_ERR_BASE + 0x0007)
#define U3V_ERR_REQUESTED_MEMORY_ACCESS_TOO_LARGE (U3V_ERR_BASE + 0x0008)
#define U3V_ERR_NO_EVENT_INTERFACE			      (U3V_ERR_BASE + 0x0009)
#define U3V_ERR_EVENTS_NOT_CONFIGURED  	          (U3V_ERR_BASE + 0x000A)
#define U3V_ERR_EVENTS_ALREADY_CONFIGURED		  (U3V_ERR_BASE + 0x000B)
#define U3V_ERR_EVENTS_NOT_STARTED			      (U3V_ERR_BASE + 0x000C)
#define U3V_ERR_EVENT_ALREADY_QUEUED		      (U3V_ERR_BASE + 0x000D)
#define U3V_ERR_EVENT_WAIT_CANCELED			      (U3V_ERR_BASE + 0x000E)
#define U3V_ERR_NO_STREAM_INTERFACE			      (U3V_ERR_BASE + 0x000F)
#define U3V_ERR_STREAM_NOT_CONFIGURED			  (U3V_ERR_BASE + 0x0010)
#define U3V_ERR_STREAM_ALREADY_CONFIGURED		  (U3V_ERR_BASE + 0x0011)
#define U3V_ERR_BUFFER_WAIT_CANCELED			  (U3V_ERR_BASE + 0x0012)
#define U3V_ERR_BUFFER_NOT_CONFIGURED			  (U3V_ERR_BASE + 0x0013)
#define U3V_ERR_BUFFER_ALREADY_QUEUED			  (U3V_ERR_BASE + 0x0014)
#define U3V_ERR_BUFFER_NOT_QUEUED			      (U3V_ERR_BASE + 0x0015)
#define U3V_ERR_BUFFER_CANNOT_BE_COPIED			  (U3V_ERR_BASE + 0x0016)
#define U3V_ERR_BUFFER_STILL_IN_USE			      (U3V_ERR_BASE + 0x0017)
#define U3V_ERR_ALLOCATION_FAILED			      (U3V_ERR_BASE + 0x0018)
#define U3V_ERR_NOT_SUPPORTED				      (U3V_ERR_BASE + 0x0019)
#define U3V_ERR_INVALID_PARAMETER			      (U3V_ERR_BASE + 0x001A)
#define U3V_ERR_ALREADY_WAITING				      (U3V_ERR_BASE + 0x001B)
#define U3V_ERR_CONNECTION_SPEED_NOT_SUPPORTED	  (U3V_ERR_BASE + 0x001C)
#define U3V_ERR_DEVICE_NOT_FOUND			      (U3V_ERR_BASE + 0x001D)
#define U3V_ERR_DEVICE_NEEDS_MORE_POWER			  (U3V_ERR_BASE + 0x001E)
#define U3V_ERR_MAX_CURRENT_INVALID			      (U3V_ERR_BASE + 0x001F)
#define U3V_ERR_IMAGE_SIZE_NOT_ALIGNED			  (U3V_ERR_BASE + 0x0020)
#define U3V_ERR_CHUNK_DATA_SIZE_TOO_BIG			  (U3V_ERR_BASE + 0x0021)
#define U3V_ERR_SEGMENTED_XFER_NOT_AVAIL		  (U3V_ERR_BASE + 0x0022)
#define U3V_ERR_MAX_RANGE				          (U3V_ERR_BASE + 0xFFFF)

#define U3V_CONTROL_PREFIX                        0x43563355
//#define U3V_CTRL_ENDPOINT                         0
//Utiliser USB_ENDPOINT_CONTROL de usb100.h

/* Constants for leader and trailer "magic key" */
#define U3V_LEADER	                              0x4C563355 /* "U3VL" */
#define U3V_TRAILER	                              0x54563355 /* "U3VT" */

/* Constants for leader and trailer section field */
#define U3V_LEADER_SECTION	                      0x1
#define U3V_TRAILER_SECTION	                      0x2
#define U3V_MASK_SECTION	                      0xF

/* Constants for leader and trailer payload type */
#define U3V_IMAGE			                      0x0001
#define U3V_CHUNK			                      0x4000
#define U3V_IMAGE_EXTENDED_CHUNK	              0x4001

/* Command Codes */
#define READMEM_CMD	                              0x0800
#define	READMEM_ACK	                              0x0801
#define	WRITEMEM_CMD	                          0x0802
#define	WRITEMEM_ACK	                          0x0803
#define	PENDING_ACK	                              0x0805
#define	EVENT_CMD	                              0x0C00

/* Technology Agnostic Bootstrap Register Map (ABRM). */
#define ABRM_GENCP_VERSION		                  0x00000
#define ABRM_MANUFACTURER_NAME		              0x00004
#define ABRM_MODEL_NAME			                  0x00044
#define ABRM_FAMILY_NAME		                  0x00084
#define ABRM_DEVICE_VERSION		                  0x000C4
#define ABRM_MANUFACTURER_INFO		              0x00104
#define ABRM_SERIAL_NUMBER		                  0x00144
#define ABRM_USER_DEFINED_NAME		              0x00184
#define ABRM_DEVICE_CAPABILITY		              0x001C4
#define ABRM_MAX_DEVICE_RESPONSE_TIME	          0x001CC
#define ABRM_MANIFEST_TABLE_ADDRESS	              0x001D0
#define ABRM_SBRM_ADDRESS		                  0x001D8
#define ABRM_DEVICE_CONFIGURATION	              0x001E0
#define ABRM_HEARTBEAT_TIMEOUT		              0x001E8
#define ABRM_MESSAGE_CHANNEL_ID		              0x001EC
#define ABRM_TIMESTAMP			                  0x001F0
#define ABRM_TIMESTAMP_LATCH		              0x001F8
#define ABRM_TIMESTAMP_INCREMENT	              0x001FC
#define ABRM_ACCESS_PRIVILEGE		              0x00204
#define ABRM_PROTOCOL_ENDIANESS		              0x00208
#define ABRM_IMPLEMENTATION_ENDIANESS	          0x0020C
#define ABRM_RESERVED			                  0x00210

/*
 * Technology Specific Bootstrap Register Map (SBRM). Offsets are relative
 * to the SBRMAddress listed in the ABRM.
 */
#define SBRM_U3V_VERSION		                  0x00000
#define SBRM_U3VCP_CAPABILITY		              0x00004
#define SBRM_U3VCP_CONFIGURATION	              0x0000C
#define SBRM_MAX_CMD_TRANSFER		              0x00014
#define SBRM_MAX_ACK_TRANSFER		              0x00018
#define SBRM_NUM_STREAM_CHANNELS	              0x0001C
#define SBRM_SIRM_ADDRESS	                   	  0x00020
#define SBRM_SIRM_LENGTH		                  0x00028
#define SBRM_EIRM_ADDRESS		                  0x0002C
#define SBRM_EIRM_LENGTH		                  0x00034
#define SBRM_IIDC2_ADDRESS		                  0x00038
#define SBRM_CURRENT_SPEED		                  0x00040
#define SBRM_RESERVED			                  0x00044

#define SIRM_AVAILABLE_MASK		                  0x00000001

/*
 * Streaming Interface Bootstrap Register Map (SIRM) offsets.
 * The base SIRM address is a field in the SBRM
 */
#define SI_INFO			                          0x00
#define SI_CONTROL		                          0x04
#define SI_REQ_PAYLOAD_SIZE	                      0x08
#define SI_REQ_LEADER_SIZE	                      0x10
#define SI_REQ_TRAILER_SIZE	                      0x14
#define SI_MAX_LEADER_SIZE	                      0x18
#define SI_PAYLOAD_SIZE		                      0x1C
#define SI_PAYLOAD_COUNT	                      0x20
#define SI_TRANSFER1_SIZE	                      0x24
#define SI_TRANSFER2_SIZE	                      0x28
#define SI_MAX_TRAILER_SIZE	                      0x2C
#define SI_INFO_ALIGNMENT_MASK	                  0xFF000000
#define SI_INFO_ALIGNMENT_SHIFT	                  0x18

/* Stream interface shared structs */
struct buffer_complete_data {
	uint32_t structure_size;
	uint32_t status;
	uint32_t expected_urb_count;
	uint32_t incomplete_urb_count;
	uint64_t payload_bytes_received;
};

struct leader_header {
	uint32_t magic_key;
	uint16_t reserved0;
	uint16_t leader_size;
	uint64_t block_id;
	uint16_t reserved1;
	uint16_t payload_type;
};

struct leader {
	struct leader_header header;
	uint8_t payload_type_info[0];
};

struct image_leader_info {
	uint64_t timestamp;
	uint32_t pixel_format;
	uint32_t size_x;
	uint32_t size_y;
	uint32_t offset_x;
	uint32_t offset_y;
	uint16_t padding_x;
	uint16_t reserved;
};

struct chunk_leader_info {
	uint64_t timestamp;
};

struct trailer_header {
	uint32_t magic_key;
	uint16_t reserved0;
	uint16_t trailer_size;
	uint64_t block_id;
	uint16_t status;
	uint16_t reserved1;
	uint64_t valid_payload_size;
};

struct trailer {
	struct trailer_header header;
	uint8_t payload_type_info[0];
};

struct image_trailer_info {
	uint32_t size_y;
};

struct image_extended_chunk_trailer_info {
	uint32_t size_y;
	uint32_t chunk_layout_id;
};

struct chunk_trailer_info {
	uint32_t chunk_layout_id;
};

/* Command interface shared structs */
struct command_header {
	uint32_t prefix;
	uint16_t flags;
	uint16_t cmd;
	uint16_t length;
	uint16_t request_id;
};

struct command {
	struct command_header header;
	uint8_t payload[0];
};

struct ack_header {
	uint32_t prefix;
	uint16_t status;
	uint16_t cmd;
	uint16_t length;
	uint16_t ack_id;
};

struct ack {
	struct ack_header header;
	uint8_t payload[0];
};

struct read_mem_cmd_payload {
	uint64_t address;
	uint16_t reserved;
	uint16_t byte_count;
};

struct read_mem_ack_payload {
	uint8_t data[0];
};

struct write_mem_cmd_payload {
	uint64_t address;
	uint8_t data[0];
};

struct write_mem_ack_payload {
	uint16_t reserved;
	uint16_t bytes_written;
};

struct pending_ack_payload {
	uint16_t reserved;
	uint16_t timeout;
};

struct event_cmd_payload {
	uint16_t reserved;
	uint16_t event_id;
	uint64_t timestamp;
	uint8_t data[0];
};

/* Event interface shared struct */
struct event_complete_data {
	uint32_t structure_size;
	uint32_t event_callback_status;
	uint64_t overwritten_event_count;
};

/* Request values for U3V driver's ioctl entry point */
struct u3v_read_memory {
	uint64_t address;
	void *user_buffer;
	uint32_t transfer_size;
	uint32_t *bytes_read;
};

struct u3v_write_memory {
	uint64_t address;
	const void *user_buffer;
	uint32_t transfer_size;
	uint32_t *bytes_written;
};

struct u3v_get_stream_alignment {
	uint32_t *stream_alignment;
};

struct u3v_get_os_max_transfer {
	uint32_t *os_max_transfer_size;
};

struct u3v_configure_stream {
	uint64_t image_buffer_size;
	uint64_t chunk_data_buffer_size;
	uint32_t max_urb_size;
	uint32_t *max_leader_size;
	uint32_t *max_trailer_size;
};

struct u3v_configure_stream2 {
	uint64_t image_buffer_size;
	uint64_t chunk_data_buffer_size;
	uint32_t max_urb_size;
	uint32_t segment_padding;
	uint64_t segment_size;
	uint32_t *u_max_leader_size;
	uint32_t *u_max_trailer_size;
};
struct u3v_configure_buffer {
	void *user_image_buffer;
	void *user_chunk_data_buffer;
	uint64_t *buffer_handle;
};

struct u3v_unconfigure_buffer {
	uint64_t buffer_handle;
};

struct u3v_queue_buffer {
	uint64_t buffer_handle;
};

struct u3v_wait_for_buffer {
	uint64_t buffer_handle;
	void *user_leader_buffer;
	uint32_t *u_leader_size;
	void *user_trailer_buffer;
	uint32_t *trailer_size;
	void *buffer_complete_data;
};

struct u3v_control_msg {
	uint8_t request;
	uint8_t request_type;
	uint16_t value;
	uint16_t index;
	void *u_data;
	uint16_t size;
};

struct u3v_start_events {
	uint32_t event_transfer_max_size;
	uint32_t event_queue_depth;
};

struct u3v_wait_for_event {
	void *user_buffer;
	uint32_t *buffer_size;
	void *event_complete_buffer;
};


/*
 * ioctrl-number.txt shows the registered ioctl numbers for drivers.
 * 0x5D was arbitrarily picked here because it is currently an unused
 * block with room for expansion.
 */
#define U3V_MAGIC			0x5D

#define U3V_IOCTL_READ			\
	_IOR(U3V_MAGIC, 1, struct u3v_read_memory)

#define U3V_IOCTL_WRITE			\
	_IOW(U3V_MAGIC, 2, struct u3v_write_memory)

#define U3V_IOCTL_GET_STREAM_ALIGNMENT	\
	_IOR(U3V_MAGIC, 3, struct u3v_get_stream_alignment)

#define U3V_IOCTL_GET_OS_MAX_TRANSFER	\
	_IOR(U3V_MAGIC, 4, struct u3v_get_os_max_transfer)

#define U3V_IOCTL_CONFIGURE_STREAM	\
	_IOR(U3V_MAGIC, 5, struct u3v_configure_stream)

#define U3V_IOCTL_UNCONFIGURE_STREAM	\
	_IO(U3V_MAGIC, 6)

#define U3V_IOCTL_CONFIGURE_BUFFER	\
	_IOR(U3V_MAGIC, 7, struct u3v_configure_buffer)

#define U3V_IOCTL_UNCONFIGURE_BUFFER	\
	_IOW(U3V_MAGIC, 8, struct u3v_unconfigure_buffer)

#define U3V_IOCTL_QUEUE_BUFFER		\
	_IOW(U3V_MAGIC, 9, struct u3v_queue_buffer)

#define U3V_IOCTL_WAIT_FOR_BUFFER	\
	_IOR(U3V_MAGIC, 10, struct u3v_wait_for_buffer)

#define U3V_IOCTL_CANCEL_ALL_BUFFERS	\
	_IO(U3V_MAGIC, 11)

#define U3V_IOCTL_CTRL_MSG		\
	_IOR(U3V_MAGIC, 12, struct u3v_control_msg)

#define U3V_IOCTL_START_EVENTS		\
	_IOW(U3V_MAGIC, 13, struct u3v_start_events)

#define U3V_IOCTL_WAIT_FOR_EVENT	\
	_IOR(U3V_MAGIC, 14, struct u3v_wait_for_event)

#define U3V_IOCTL_STOP_EVENTS		\
	_IO(U3V_MAGIC, 15)

#define U3V_MAX_NR			15

#endif /* _U3V_SHARED_H */
