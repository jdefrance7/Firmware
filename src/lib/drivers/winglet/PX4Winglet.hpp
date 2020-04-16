#pragma once

#include <drivers/drv_winglet.h>
#include <drivers/drv_hrt.h>

#include <lib/cdev/CDev.hpp>

#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_winglet.h>

class PX4Winglet : public cdev::CDev
{
public:
	PX4Winglet(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT);
	~PX4Winglet() override;

	const sensor_winglet_s &get() { return _sensor_winglet_pub.get(); }

	void update(hrt_abstime timestamp_sample, uint8_t id, char key, float value);
	void update(hrt_abstime timestamp_sample, uint8_t node_id, uint8_t gimbal_id, float x, float y, float z, float w);

	int get_class_instance() { return _class_device_instance; };

	void print_status();

private:

	uORB::PublicationMultiData<sensor_winglet_s> _sensor_winglet_pub;

	uint8_t _id;
	float _xyzw[4];

	int _class_device_instance{-1};
};
