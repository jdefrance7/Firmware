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
  * 	@file PX4Winglet.cpp
  *
  *		Driver file to publish wing segment orientation information.
  *
  * 	@author Joe DeFrance
  */

#include "PX4Winglet.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Winglet::PX4Winglet(uint32_t device_id, uint8_t priority) :
	CDev(nullptr),
	_sensor_winglet_pub{ORB_ID(sensor_winglet), priority}
{
	_class_device_instance = register_class_devname(UAVCAN_WINGLET_BASE_DEVICE_PATH);

	_sensor_winglet_pub.get().timestamp = 0;
	_sensor_winglet_pub.get().id = 0;
	_sensor_winglet_pub.get().x = 0;
	_sensor_winglet_pub.get().y = 0;
	_sensor_winglet_pub.get().z = 0;
	_sensor_winglet_pub.get().w = 0;
}

PX4Winglet::~PX4Winglet()
{
	if(_class_device_instance != -1)
	{
		unregister_class_devname(UAVCAN_WINGLET_BASE_DEVICE_PATH, _class_device_instance);
	}
}

void PX4Winglet::update(hrt_abstime timestamp_sample, uint8_t id, char key, float value)
{
	_id = id;

	if(key == 'X')
	{
		_xyzw[0] = value;
	}
	else if(key == 'Y')
	{
		_xyzw[1] = value;
	}
	else if(key == 'Z')
	{
		_xyzw[2] = value;
	}
	else if(key == 'W')
	{
		_xyzw[3] = value;
	}
	else
	{
		PX4_WARN("unknown key: %c", key);
	}

	sensor_winglet_s &report = _sensor_winglet_pub.get();
	report.timestamp = timestamp_sample;
	report.id = _id;
	report.x = _xyzw[0];
	report.y = _xyzw[1];
	report.z = _xyzw[2];
	report.w = _xyzw[3];
	_sensor_winglet_pub.update();
}

void PX4Winglet::update(hrt_abstime timestamp_sample, uint8_t node_id, uint8_t wing_segment_id, float x, float y, float z, float w)
{
	_id = wing_segment_id;
	_xyzw[0] = x;
	_xyzw[1] = y;
	_xyzw[2] = z;
	_xyzw[3] = w;

	sensor_winglet_s &report = _sensor_winglet_pub.get();
	report.timestamp = timestamp_sample;
	report.id = _id;
	report.x = _xyzw[0];
	report.y = _xyzw[1];
	report.z = _xyzw[2];
	report.w = _xyzw[3];
	_sensor_winglet_pub.update();
}

void PX4Winglet::print_status()
{
	PX4_INFO(UAVCAN_WINGLET_BASE_DEVICE_PATH "device instance: %d", _class_device_instance);

	print_message(_sensor_winglet_pub.get());
}
