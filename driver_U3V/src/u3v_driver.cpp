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
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/resmgr.h>

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_event.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include "u3v_interrupt.h"
#include "video_class_code.h"



struct usbd_connection* u3v_connection;
uvc_device_mapping_t devmap[MAX_UVC_DEVICES];
void u3v_insertion(struct usbd_connection* connection, usbd_device_instance_t* instance) {
    struct usbd_device*              u3v_device;
    usbd_interface_descriptor_t*     u3v_interface_descriptor;
    usbd_configuration_descriptor_t* u3v_configuration_descriptor;
    usbd_device_descriptor_t*        u3v_device_descriptor;
    struct usbd_desc_node*           u3v_node;
    struct usbd_desc_node*           u3v_node2;
    usbd_descriptors_t*              u3v_descriptor;
}


void u3v_removal(struct usbd_connection* connection, usbd_device_instance_t* instance) {
}
void u3v_event(struct usbd_connection* connection, usbd_device_instance_t* instance, uint16_t type) {

}



usbd_device_ident_t u3v_device_ident= {
    USBD_CONNECT_WILDCARD, /* Vendor ID              */
    USBD_CONNECT_WILDCARD, /* Device ID              */
    NULL,                  /* Device Class           */
    USBD_CONNECT_WILDCARD, /* Device SubClass        */
    USBD_CONNECT_WILDCARD, /* Device Protocol        */
};

usbd_funcs_t u3v_functions= {
		_USBDI_NFUNCS,
	    u3v_insertion,
		u3v_removal,
		u3v_event
};

usbd_connect_parm_t u3v_connection_params= {
    NULL,              /* Default connection path        */
    USB_VERSION,       /* Version of USB stack           */
    USBD_VERSION,      /* Version of USB DDK             */
    0,                 /* Flags                          */
    0,                 /* Arguments count                */
    NULL,              /* Arguments                      */
    0,                 /* Default event buffer size      */
    &u3v_device_ident, /* Device identification settings */
    &u3v_functions,    /* Device attach/detach functions */
    USBD_CONNECT_WAIT, /* Default connection timeout     */
    0,                 /* Event priority                 */
};

int main(void) {
	return 0;
}
