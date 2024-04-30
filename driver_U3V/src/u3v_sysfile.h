#ifndef _U3V_SYSFILE_H_
#define _U3V_SYSFILE_H_

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


#define U3V_SYSFS_FILES 9
#define U3V_SYSFS_FILE_DEV               0x00000000
#define U3V_SYSFS_FILE_INDEX             0x00000001
#define U3V_SYSFS_FILE_NAME              0x00000002
#define U3V_SYSFS_FILE_UEVENT            0x00000003
#define U3V_SYSFS_FILE_IDVENDOR          0x00000004
#define U3V_SYSFS_FILE_IDPRODUCT         0x00000005
#define U3V_SYSFS_FILE_SPEED             0x00000006
#define U3V_SYSFS_FILE_DEVICE_MODALIAS   0x00000007
#define U3V_SYSFS_FILE_DEV_CHAR          0x00000008
#define U3V_SYSFS_MAX_FILESIZE           128



#define MAX_UVC_DEVICES 16
#define UVC_MAX_VS_COUNT 8


typedef struct u3v_sysfs_device {
    iofunc_attr_t          hdr[U3V_SYSFS_FILES];
    int                    resmgr_id[U3V_SYSFS_FILES];
    resmgr_io_funcs_t      io_funcs[U3V_SYSFS_FILES];
    resmgr_connect_funcs_t connect_funcs[U3V_SYSFS_FILES];
    iofunc_funcs_t         ocb_funcs[U3V_SYSFS_FILES];
    iofunc_mount_t         io_mount[U3V_SYSFS_FILES];
    resmgr_attr_t          rattr[U3V_SYSFS_FILES];
    char                   filedata[U3V_SYSFS_FILES][U3V_SYSFS_MAX_FILESIZE];
} u3v_sysfs_device_t;

typedef struct _uvc_ocb {
    iofunc_ocb_t        hdr;
    struct u3v_device*  dev;
    void*               handle;
    int                 subdev;
    iofunc_notify_t     notify[3];
} uvc_ocb_t;

typedef struct _uvc_device_mapping {
    int initialized;             /* 1 - if this device is initialized          */
    int devid;                   /* device minors, i.e. /dev/video[devid]      */
    struct u3v_device* uvcd;     /* uvc_device_t pointer of associated device  */
    uint8_t  usb_devno;
    uint8_t  usb_path;
    uint16_t usb_generation;
} uvc_device_mapping_t;

#endif /*  _U3V_SYSFILE_H_ */
