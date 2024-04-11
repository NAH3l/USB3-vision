#include <errno.h>
#include <sys/usbdi.h>
#include <iostream>
#include <stdlib.h>

#include "u3v.h"
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
int usb_control_msg(struct usbd_device *dev, struct usbd_pipe *pipe, uint8_t request, uint8_t requesttype, uint16_t value, uint16_t index, void *data, uint16_t size, int timeout) {
    struct usbd_urb *urb;
    int ret;

    // Allocation de l'URB
    urb = usbd_alloc_urb(NULL);
    if (!urb) {
        return -ENOMEM; // Insufficient memory available
    }

    // Configuration de l'URB pour le message de contrôle
    ret = usbd_setup_vendor(urb, 0, request, requesttype, value, index, data, size);
    if (ret != EOK) {
        usbd_free_urb(urb);
        return ret;
    }

    // Soumission de l'URB
    ret = usbd_io(urb, pipe, NULL, NULL, timeout);
    if (ret != EOK) {
        usbd_free_urb(urb);
        return ret;
    }

    // Attente de la complétion de l'URB
    ret = usbd_urb_status(urb, NULL, NULL);
    if (ret != EOK) {
        // Si une erreur survient pendant l'attente, libérer l'URB
        usbd_free_urb(urb);
        return ret;
    }

    // Récupération du nombre de bytes transférés
    uint32_t len;
    ret = usbd_urb_status(urb, NULL, &len);
    if (ret != EOK) {
        // Si une erreur survient pendant l'attente, libérer l'URB
        usbd_free_urb(urb);
        return ret;
    }

    // Libération de l'URB
    usbd_free_urb(urb);

    return len;
}

/* Reproduis la fonction usb_bulk_msg de Linux */
int usb_bulk_msg(struct usbd_device *usb_dev, struct usbd_pipe *pipe, void *data, uint32_t len, uint32_t *actual_length, int timeout) {
    struct usbd_urb *urb;
    int ret;

    // Allocation de l'URB
    urb = usbd_alloc_urb(NULL);
    if (!urb) {
        return -ENOMEM; // Insufficient memory available
    }

    // Configuration de l'URB pour le message en vrac
    ret = usbd_setup_bulk(urb, URB_DIR_OUT, data, len);
    if (ret != EOK) {
        usbd_free_urb(urb);
        return ret;
    }

    // Soumission de l'URB
    ret = usbd_io(urb, pipe, NULL, NULL, timeout);
    if (ret != EOK) {
        usbd_free_urb(urb);
        return ret;
    }

    // Attente de la complétion de l'URB
    ret = usbd_urb_status(urb, NULL, actual_length);
    if (ret != EOK) {
        // Si une erreur survient pendant l'attente, libérer l'URB
        usbd_free_urb(urb);
        return ret;
    }

    // Libération de l'URB
    usbd_free_urb(urb);

    return EOK; // Succès
}

/* Reproduis la fonction usb_sndbulkpipe de Linux */
unsigned int usb_sndbulkpipe(struct usbd_device *dev, struct usbd_pipe *pipe) {
    // Utilisation de usbd_pipe_endpoint pour récupérer le numéro de l'endpoint
    uint32_t endpoint_num = usbd_pipe_endpoint(pipe);

    // Maintenant, vous pouvez utiliser 'endpoint_num' comme nécessaire dans votre programme
    return endpoint_num;
}

/* Reproduis la fonction usb_endpoint_num de Linux */
int usb_endpoint_num(const struct usbd_endpoint_descriptor *epd) {
    // Vérifier si la structure est valide
    if (!epd) {
        // Gérer l'erreur si nécessaire
        return -1; // Valeur d'erreur
    }

    // Extraire le numéro de l'endpoint à partir de la structure usbd_endpoint_descriptor
    return (int)(epd->bEndpointAddress);
}

/* Reproduis la fonction usb_rcvbulkpipe de Linux */
unsigned int usb_rcvbulkpipe(struct usbd_device *dev, unsigned int endpoint) {
    struct usbd_pipe *pipe;

    // Utilisation de usbd_open_pipe pour initialiser le pipe associé à l'endpoint
    int ret = usbd_open_pipe(dev, NULL, &pipe);
    if (ret != EOK) {
        // Gérer l'erreur si nécessaire
        return NULL;
    }

    return (unsigned int)pipe;
}

unsigned int usb_sndctrlpipe(struct usbd_device *dev, unsigned int endpoint) {
    struct usbd_pipe *pipe;

    // Ouvrir un pipe de contrôle OUT pour le périphérique USB et l'endpoint spécifié
    int ret = usbd_open_pipe(dev, NULL, &pipe);
    if (ret != EOK) {
        // Gérer l'erreur si nécessaire
        return NULL;
    }

    return (unsigned int)pipe;
}

/* Reproduis la fonction usb_clear_halt de Linux */
int usb_clear_halt(struct usbd_device *dev, struct usbd_pipe *pipe) {
    // Utilisation de usbd_reset_pipe pour effacer la condition de stagnation (stall)
    int ret = usbd_reset_pipe(pipe);
    if (ret != EOK) {
        // Gestion de l'erreur
    }

    return ret;
}

int wait_for_completion(struct completion *comp) {
    if (comp == NULL) {
    	std::cout << "Error, completion is NULL" << std::endl;
    }

    pthread_mutex_lock(&comp->mutex);

    while (!comp->complete) {
    	pthread_cond_wait(&comp->cond, &comp->mutex);
    }

    pthread_mutex_unlock(&comp->mutex);

    return 0;
}

// Fonction pour initialiser une complétion
void init_completion(struct completion *comp) {
    pthread_mutex_init(&comp->mutex, NULL);
    pthread_cond_init(&comp->cond, NULL);
    comp->complete = false;
}

// Fonction pour signaler à tous les threads en attente sur cette complétion
void complete_all(struct completion *comp) {
    pthread_mutex_lock(&comp->mutex);
    comp->complete = true;
    pthread_cond_broadcast(&comp->cond); // Réveille tous les threads en attente
    pthread_mutex_unlock(&comp->mutex);
}



