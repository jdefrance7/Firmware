/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 /**
  *  @file winglet.cpp
  *
  *	UAVCAN sensor bridge class for winglet AngularCommand messages.
  *
  *	@author Joe DeFrance
  */


// Header file
#include "winglet.hpp"

// Timer
#include <drivers/drv_hrt.h>

// Error handling
#include <systemlib/err.h>

// PX4Winglet class (handles multiple winglet instances)
#include <lib/drivers/winglet/PX4Winglet.hpp>

// Class name
const char *const UavcanWingletBridge::NAME = "winglet";

// Base path
#define UAVCAN_WINGLET_BASE_DEVICE_PATH "/dev/uavcan/winglet"

// Constructor
UavcanWingletBridge::UavcanWingletBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase(
		"uavcan_winglet",
		UAVCAN_WINGLET_BASE_DEVICE_PATH,
		UAVCAN_WINGLET_BASE_DEVICE_PATH,
		ORB_ID(sensor_winglet)
	),
	_sub_uavcan_angular_command(node)
{
}

// Initializer
int UavcanWingletBridge::init()
{
	int res = device::CDev::init();

	if(res < 0) {
		return res;
	}

	res = _sub_uavcan_angular_command.start(AngularCommandCallback(this, &UavcanWingletBridge::uavcan_angular_command_cb));

	if(res < 0) {
		PX4_ERR("failed to start uavcan angular command sub: %d", res);
		return res;
	}

	return 0;
}

// Callback function
void UavcanWingletBridge::uavcan_angular_command_cb(const
	uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand> &msg)
{
	// Get allocated sensor channel
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	if(channel == nullptr)
	{
		// Something went wrong - no channel to publish on; return
		PX4_ERR("No channel to publush sensor_winglet on");
		return;
	}

	// Get instance of winglet publisher
	PX4Winglet *winglet = (PX4Winglet *)channel->h_driver;

	if(winglet == nullptr)
	{
		return;
	}

	// Extract node id from AngularCommand message
	uint8_t node_id =  msg.getSrcNodeID().get();

	// Extract wing segment id from AngularCommand message
	uint8_t gimbal_id = msg.gimbal_id;

	// Extract quaternion data from AngularCommand message
	float x = msg.quaternion_xyzw[0];
	float y = msg.quaternion_xyzw[1];
	float z = msg.quaternion_xyzw[2];
	float w = msg.quaternion_xyzw[3];

	// Send extracted values to winglet publisher
	winglet->update(hrt_absolute_time(), node_id, gimbal_id, x, y, z, w);
}

// Driver initializer (uses PX4Winglet to create multiple instances of winglet)
int UavcanWingletBridge::init_driver(uavcan_bridge::Channel *channel)
{
	DeviceId device_id{_device_id};

	device_id.devid_s.devtype = DRV_DEVTYPE_UNUSED;
	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

	channel->h_driver = new PX4Winglet(device_id.devid, ORB_PRIO_HIGH);

	if(channel->h_driver == nullptr)
	{
		return PX4_ERROR;
	}

	PX4Winglet *winglet = (PX4Winglet *)channel->h_driver;

	channel->class_instance = winglet->get_class_instance();

	if(channel->class_instance < 0)
	{
		PX4_ERR("UavcanWinget: unable to get a class instance");
		delete winglet;
		channel->h_driver = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}
