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
 *  @file wing_segment.hpp
 *
 *	UAVCAN sensor bridge class for wing segment AngularCommand messages.
 *
 *	@author Joe DeFrance
 */

#pragma once

// Parent Class
#include "sensor_bridge.hpp"

// Timer
#include <drivers/drv_hrt.h>

// UAVCAN Sources
#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>

// uORB Sources
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_wing_segment.h>

// UavcanWingSegmentBridge Class
class UavcanWingSegmentBridge : public UavcanCDevSensorBridgeBase
{
public:

	// Class name
	static const char *const NAME;

	// Constuctor
	UavcanWingSegmentBridge(uavcan::INode &node);

	// Name getter
	const char *get_name() const override { return NAME; }

	// Initializer
	int init() override;

private:

	// Driver initializer
	int init_driver(uavcan_bridge::Channel *channel) override;

	// UAVCAN callback function for AngularCommand broadcast (what eventually gets called when an AngularCommand braodcast is received)
	void uavcan_angular_command_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand> &msg);

	// UAVCAN callback binder
	typedef uavcan::MethodBinder<UavcanWingSegmentBridge *,
		void (UavcanWingSegmentBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand>&)>
		AngularCommandCallback;

	// UAVCAN subscription
	uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand, AngularCommandCallback> _sub_uavcan_angular_command;
};
