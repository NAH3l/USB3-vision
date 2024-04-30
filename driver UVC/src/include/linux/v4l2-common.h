/*
 * include/linux/v4l2-common.h
 *
 * Common V4L2 and V4L2 subdev definitions.
 *
 * Users are advised to #include this file either through videodev2.h
 * (V4L2) or through v4l2-subdev.h (V4L2 subdev) rather than to refer
 * to this file directly.
 *
 * Copyright (C) 2012 Nokia Corporation
 * Contact: Sakari Ailus <sakari.ailus@iki.fi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __V4L2_COMMON__
#define __V4L2_COMMON__

#include <C:/Users/nverdier/ide-7.0-workspace/Camera_USB/src/include/linux/compiler.h>

#if defined(__QNXNTO__)
    #include <stdint.h>
    #include <devctl.h>
    #include <time.h>
    #include <sys/time.h>
    typedef int8_t   __s8;
    typedef uint8_t  __u8;
    typedef int16_t  __s16;
    typedef uint16_t __u16;
    typedef int32_t  __s32;
    typedef uint32_t __u32;
    typedef uint32_t __le32;
    typedef int64_t  __s64;
    typedef uint64_t __u64;
#endif

/*
 *
 * Selection interface definitions
 *
 */

/* Current cropping area */
#define V4L2_SEL_TGT_CROP		0x0000
/* Default cropping area */
#define V4L2_SEL_TGT_CROP_DEFAULT	0x0001
/* Cropping bounds */
#define V4L2_SEL_TGT_CROP_BOUNDS	0x0002
/* Current composing area */
#define V4L2_SEL_TGT_COMPOSE		0x0100
/* Default composing area */
#define V4L2_SEL_TGT_COMPOSE_DEFAULT	0x0101
/* Composing bounds */
#define V4L2_SEL_TGT_COMPOSE_BOUNDS	0x0102
/* Current composing area plus all padding pixels */
#define V4L2_SEL_TGT_COMPOSE_PADDED	0x0103

/* Backward compatibility target definitions --- to be removed. */
#define V4L2_SEL_TGT_CROP_ACTIVE	V4L2_SEL_TGT_CROP
#define V4L2_SEL_TGT_COMPOSE_ACTIVE	V4L2_SEL_TGT_COMPOSE
#define V4L2_SUBDEV_SEL_TGT_CROP_ACTUAL	V4L2_SEL_TGT_CROP
#define V4L2_SUBDEV_SEL_TGT_COMPOSE_ACTUAL V4L2_SEL_TGT_COMPOSE
#define V4L2_SUBDEV_SEL_TGT_CROP_BOUNDS	V4L2_SEL_TGT_CROP_BOUNDS
#define V4L2_SUBDEV_SEL_TGT_COMPOSE_BOUNDS V4L2_SEL_TGT_COMPOSE_BOUNDS

/* Selection flags */
#define V4L2_SEL_FLAG_GE		(1 << 0)
#define V4L2_SEL_FLAG_LE		(1 << 1)
#define V4L2_SEL_FLAG_KEEP_CONFIG	(1 << 2)

/* Backward compatibility flag definitions --- to be removed. */
#define V4L2_SUBDEV_SEL_FLAG_SIZE_GE	V4L2_SEL_FLAG_GE
#define V4L2_SUBDEV_SEL_FLAG_SIZE_LE	V4L2_SEL_FLAG_LE
#define V4L2_SUBDEV_SEL_FLAG_KEEP_CONFIG V4L2_SEL_FLAG_KEEP_CONFIG

#endif /* __V4L2_COMMON__ */
