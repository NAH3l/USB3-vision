#include <errno.h>
#include <sys/usbdi.h>
#include <iostream>

#include <unistd.h>

#include "u3v.h"


static struct usbd_connection *conn_usb = NULL;

// Déclaration des fonctions de rappel
static void cbinsert(struct usbd_connection *connection, usbd_device_instance_t *ins);
static void cbremove(struct usbd_connection *connection, usbd_device_instance_t *ins);



// Fonction d'initialisation
int init(void) {
    // Configuration des fonctions de rappel
    usbd_funcs_t funcs = { _USBDI_NFUNCS, cbinsert, cbremove, NULL };
    // Paramètres de connexion
    usbd_connect_parm_t parm = { NULL, USB_VERSION, USBD_VERSION, 0, 0, NULL, 0, NULL, &funcs, 0 };

    // Tentative de connexion
    if (usbd_connect(&parm, &conn_usb) != EOK) {
        // Gestion de l'erreur de connexion
        std::cout << "Error usbd_connect" << std::endl;
    }
	return 0;
}

// Fonction de rappel pour l'insertion d'un périphérique USB
static void cbinsert(struct usbd_connection *usb_connection, usbd_device_instance_t *usb_instance) {
    // Vérification du type de périphérique USB inséré
    if (usb_instance->ident.dclass == 8 && usb_instance->ident.subclass == 6) {
        // Périphérique de stockage de masse USB
        std::cout << "USB mass storage insert" << std::endl;

        std::cout << std::hex << usb_instance->ident.vendor << std::endl;
        std::cout << std::hex << usb_instance->ident.device << std::endl;
        std::cout << std::hex << usb_instance->ident.dclass << std::endl;
        std::cout << std::hex << usb_instance->ident.subclass << std::endl;
        std::cout << std::hex << usb_instance->ident.protocol << std::endl;


    } else if (usb_instance->ident.dclass == 3 && usb_instance->ident.subclass == 1) {
        // Souris USB
        std::cout << "USB mouse insert" << std::endl;
    } else {
        // Périphérique inconnu
        std::cout << "unknown device insert" << std::endl;

        std::cout << usb_instance->ident.vendor << std::endl;
        std::cout << usb_instance->ident.device << std::endl;
        std::cout << usb_instance->ident.dclass << std::endl;
        std::cout << usb_instance->ident.subclass << std::endl;
        std::cout << usb_instance->ident.protocol << std::endl;
    }
}

// Fonction de rappel pour le retrait d'un périphérique USB
static void cbremove(struct usbd_connection *usb_connection, usbd_device_instance_t *usb_instance) {
    // Vérification du type de périphérique USB retiré
    if (usb_instance->ident.dclass == 8 && usb_instance->ident.subclass == 6) {
        // Périphérique de stockage de masse USB
        std::cout << "USB mass storage remove" << std::endl;
    } else if (usb_instance->ident.dclass == 3 && usb_instance->ident.subclass == 1) {
        // Souris USB
        std::cout << "USB mouse remove" << std::endl;
    } else {
        // Périphérique inconnu
        std::cout << "unknown device remove" << std::endl;
    }
}


int main(){
	std::cout << "start driver" << std::endl;
	init();
}
