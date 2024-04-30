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

int min(size_t a, size_t b) {
    return a < b ? a : b;
}

int max(size_t a, size_t b) {
    return a > b ? a : b;
}

unsigned int roundup(unsigned int x, unsigned int y) {
    return ((x + y - 1) / y) * y;
}

unsigned int rounddown(unsigned int x, unsigned int y) {
    return (x / y) * y;
}

/* Reproduis la fonction usb_control_msg de Linux */
int qnx_control_msg(struct usbd_device *device, uint8_t request, uint8_t requesttype, uint16_t value, uint16_t index, void *data, uint16_t size, int timeout) {
    // Obtenir le descripteur de l'endpoint 0 (contrôle)
    struct usbd_desc_node *node;
    usbd_descriptors_t *desc = usbd_parse_descriptors(device, NULL, USB_DESC_ENDPOINT, index, &node);

    // Ouvrir le pipe
    struct usbd_pipe *pipe;
    int ret = usbd_open_pipe(device, desc, &pipe);
    if (ret != EOK) {
        return ret;
    }

    // Allouer un URB
    struct usbd_urb *urb = usbd_alloc_urb(NULL);
    if (!urb) {
        usbd_close_pipe(pipe);
        return -ENOMEM;
    }

    // Définir la direction du transfert en fonction du type de requête
    uint32_t flags = (requesttype & URB_DIR_IN) ? URB_DIR_IN : URB_DIR_OUT ;

    // Configurer l'URB avec usbd_setup_vendor()
    ret = usbd_setup_vendor(urb, flags, request, requesttype, value, index, data, size);
    if (ret != EOK) {
        usbd_free_urb(urb);
        usbd_close_pipe(pipe);
        return ret;
    }

    // Envoyer la requête et attendre la fin
    ret = usbd_io(urb, pipe, NULL, NULL, timeout);

    // Obtenir le statut et la longueur du transfert
    uint32_t status, act_length;
    usbd_urb_status(urb, &status, &act_length);

    // Libérer les ressources
    usbd_free_urb(urb);
    usbd_close_pipe(pipe);

    // Retourner le nombre d'octets transférés en cas de succès
    if (status & USBD_STATUS_CMP) {
        return act_length;
    } else {
        // Retourner un code d'erreur en cas d'échec
        return -EIO;
    }
}

/* Reproduis la fonction usb_bulk_msg de Linux */
int usb_bulk_msg(struct usbd_device *device, unsigned int pipe, void *data, int len, int *actual_length, int timeout) {
    struct usbd_urb *urb = NULL;
    struct usbd_pipe *usb_pipe = NULL;
    int ret;
    uint32_t urb_status;
    uint8_t endpoint_num;
    usbd_device_descriptor_t *device_desc;

    /* Check for valid arguments */
    if (!device || !data || len < 0 || !actual_length) {
        std::cerr << "Invalid arguments\n";
        return -EINVAL;
    }

    /* Create a URB */
    urb = usbd_alloc_urb(NULL);
    if (!urb) {
        std::cerr << "Failed to allocate URB\n";
        return -ENOMEM;
    }

    /* Open the pipe based on the endpoint direction */
    if (pipe & USB_ENDPOINT_IN) {
        ret = usbd_open_pipe(device, NULL, &usb_pipe);
    } else {
        ret = usbd_open_pipe(device, NULL, &usb_pipe);
    }
    if (ret != EOK) {
        std::cerr << "Failed to open pipe\n";
        usbd_free_urb(urb);
        return ret;
    }

    /* Get endpoint number */
    endpoint_num = usbd_pipe_endpoint(usb_pipe);

    /* Set up the URB for bulk transfer */
    if (pipe & USB_ENDPOINT_IN) {
        ret = usbd_setup_bulk(urb, URB_DIR_IN, data, len);
    } else {
        ret = usbd_setup_bulk(urb, URB_DIR_OUT, data, len);
    }
    if (ret != EOK) {
        std::cerr << "Failed to set up URB\n";
        usbd_free_urb(urb);
        usbd_close_pipe(usb_pipe);
        return ret;
    }

    /* Get device descriptor for information */
    device_desc = usbd_device_descriptor(device, NULL);

    /* Submit the URB and wait for completion */
    std::cout << "Device: Vendor ID: 0x" << std::hex << device_desc->idVendor << ", Product ID: 0x" << device_desc->idProduct << std::dec << "\n";
    std::cout << "Endpoint: " << static_cast<int>(endpoint_num) << ", Direction: " << ((pipe & USB_ENDPOINT_IN) ? "IN" : "OUT") << ", Transfer Length: " << len << "\n";
    ret = usbd_io(urb, usb_pipe, NULL, NULL, timeout);
    if (ret != EOK) {
    	std::cerr << "URB transfer failed\n";
    	usbd_free_urb(urb);
        usbd_close_pipe(usb_pipe);
        return ret;
    }

    /* Get the actual length of data transferred and URB status */
    ret = usbd_urb_status(urb, &urb_status, (uint32_t *)actual_length);

    /* Print status information */
    std::cout << "URB transfer completed. Status: 0x" << std::hex << urb_status << ", Actual Length: " << *actual_length << std::dec << "\n";

    /* Free the URB and close the pipe */
    usbd_free_urb(urb);
    usbd_close_pipe(usb_pipe);

    return ret;
}

/*Reproduis la fonction usb_endpoint_num de Linux */
int qnx_usb_endpoint_num(const usbd_endpoint_descriptor_t *epd) {
    return epd->bEndpointAddress & USB_ENDPOINT_MASK;
}

/*Reproduis la fonction usb_rcvbulkpipe de Linux */
unsigned int qnx_usb_rcvbulkpipe(struct usbd_device *device, unsigned int endpoint_num) {
    usbd_endpoint_descriptor_t *epd;

    // Get the endpoint descriptor
    epd = usbd_endpoint_descriptor(device, 0, 0, 0, endpoint_num, NULL);

    // Check if it's a bulk IN endpoint
    if ((epd->bmAttributes & USB_ENDPOINT_MASK) == USB_ATTRIB_BULK && (epd->bEndpointAddress & USB_ENDPOINT_MASK) == USB_ENDPOINT_IN) {
        // Construct the pipe value using USB_ATTRIB_BULK and USB_DIRECTION_HOST
        return ((USB_ATTRIB_BULK << 30) | (endpoint_num << 15) | USB_DIRECTION_HOST);
    } else {
        // Handle error or return an invalid pipe value
        return 0; // or a specific error code
    }
}

/*Reproduis la fonction usb_sndbulkpipe de Linux */
unsigned int qnx_usb_sndbulkpipe(struct usbd_device *device, unsigned int endpoint_num) {
    usbd_endpoint_descriptor_t *epd;

    // Get the endpoint descriptor
    epd = usbd_endpoint_descriptor(device, 0, 0, 0, endpoint_num, NULL);

    // Check if it's a bulk OUT endpoint
    if ((epd->bmAttributes & USB_ENDPOINT_MASK) == USB_ATTRIB_BULK && (epd->bEndpointAddress & USB_ENDPOINT_MASK) == USB_ENDPOINT_OUT) {
        // Construct the pipe value using USB_ATTRIB_BULK and USB_DIRECTION_HOST
        return ((USB_ATTRIB_BULK << 30) | (endpoint_num << 15) | USB_DIRECTION_DEVICE);
    } else {
        // Handle error or return an invalid pipe value
        return 0; // or a specific error code
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/* Function for synchronisation*/
void init_completion(pthread_mutex_t mutex, pthread_cond_t cond, int done) {
  pthread_mutex_init(&mutex, NULL);
  pthread_cond_init(&cond, NULL);
  done = 0;
}

void wait_for_completion(pthread_mutex_t mutex, pthread_cond_t cond, int done) {
  pthread_mutex_lock(&mutex);
  while (!done) {
    pthread_cond_wait(&cond, &mutex);
  }
  pthread_mutex_unlock(&mutex);
}

void completion_complete_all(pthread_mutex_t mutex, pthread_cond_t cond, int done) {
  pthread_mutex_lock(&mutex);
  done = 1;
  pthread_cond_broadcast(&cond);
  pthread_mutex_unlock(&mutex);
}


