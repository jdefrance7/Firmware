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
 * 	@file PX4WingSegment.hpp
 *
 *	Driver file to publish wing segment orientation information.
 *
 * 	@author Joe DeFrance
 */

#pragma once

#include <drivers/drv_wing_segment.h>
#include <drivers/drv_hrt.h>

#include <lib/cdev/CDev.hpp>

#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_wing_segment.h>

class PX4WingSegment : public cdev::CDev
{
public:

	// Constructor
	PX4WingSegment(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT);

	// Destructor
	~PX4WingSegment() override;

	// uORB topic getter
	const sensor_wing_segment_s &get() { return _sensor_wing_segment_pub.get(); }

	// Publisher calls
	void update(hrt_abstime timestamp_sample, uint8_t node_id, uint8_t id, uint8_t callibration, float x, float y, float z, float w);

	// Class instance getter
	int get_class_instance() { return _class_device_instance; };

	// Printable status message
	void print_status();

private:

	// uORB Multi-Instance Topic
	uORB::PublicationMultiData<sensor_wing_segment_s> _sensor_wing_segment_pub;

	// Last received wing segment id
	uint8_t _id;

	// Last received wing segment callibration
	uint8_t _callibration;

	// Last received wing segment orientation
	float _xyzw[4];

	// Class instance number
	int _class_device_instance{-1};
};
