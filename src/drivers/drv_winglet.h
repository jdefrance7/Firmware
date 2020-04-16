/**
 * @file Winglet driver interface.
 */

#ifndef _DRV_WINGLET_H
#define _DRV_WINGLET_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define UAVCAN_WINGLET_BASE_DEVICE_PATH		"/dev/uavcan/winglet"
#define UAVCAN0_WINGLET_DEVICE_PATH		"/dev/uavcan/winglet0"
#define UAVCAN1_WINGLET_DEVICE_PATH		"/dev/uavcan/winglet1"
#define UAVCAN2_WINGLET_DEVICE_PATH		"/dev/uavcan/winglet2"
#define UAVCAN3_WINGLET_DEVICE_PATH		"/dev/uavcan/winglet3"

#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/log_message.h>

#endif // _DRV_WINGLET_H
