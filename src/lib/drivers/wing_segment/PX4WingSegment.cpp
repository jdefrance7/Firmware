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
  * 	@file PX4WingSegment.cpp
  *
  *		Driver file to publish wing segment orientation information.
  *
  * 	@author Joe DeFrance
  */

#include "PX4WingSegment.hpp"

#include <lib/drivers/device/Device.hpp>

PX4WingSegment::PX4WingSegment(uint32_t device_id, uint8_t priority) :
	CDev(nullptr),
	_sensor_wing_segment_pub{ORB_ID(sensor_wing_segment), priority}
{
	_class_device_instance = register_class_devname(UAVCAN_WING_SEGMENT_BASE_DEVICE_PATH);

	_sensor_wing_segment_pub.get().timestamp = 0;
	_sensor_wing_segment_pub.get().id = 0;
	_sensor_wing_segment_pub.get().callibration = 0;
	_sensor_wing_segment_pub.get().x = 0;
	_sensor_wing_segment_pub.get().y = 0;
	_sensor_wing_segment_pub.get().z = 0;
	_sensor_wing_segment_pub.get().w = 0;
}

PX4WingSegment::~PX4WingSegment()
{
	if(_class_device_instance != -1)
	{
		unregister_class_devname(UAVCAN_WING_SEGMENT_BASE_DEVICE_PATH, _class_device_instance);
	}
}

void PX4WingSegment::update(hrt_abstime timestamp_sample, uint8_t node_id, uint8_t id, uint8_t callibration, float x, float y, float z, float w)
{
	_id = id;
	_callibration = callibration;
	_xyzw[0] = x;
	_xyzw[1] = y;
	_xyzw[2] = z;
	_xyzw[3] = w;

	sensor_wing_segment_s &report = _sensor_wing_segment_pub.get();
	report.timestamp = timestamp_sample;
	report.id = _id;
	report.callibration = _callibration;
	report.x = _xyzw[0];
	report.y = _xyzw[1];
	report.z = _xyzw[2];
	report.w = _xyzw[3];
	_sensor_wing_segment_pub.update();
}

void PX4WingSegment::print_status()
{
	PX4_INFO(UAVCAN_WING_SEGMENT_BASE_DEVICE_PATH "device instance: %d", _class_device_instance);

	print_message(_sensor_wing_segment_pub.get());
}
