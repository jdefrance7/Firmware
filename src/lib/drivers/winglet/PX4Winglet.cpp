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

void PX4Winglet::update(hrt_abstime timestamp_sample, uint8_t node_id, uint8_t gimbal_id, float x, float y, float z, float w)
{
	_id = gimbal_id;
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
