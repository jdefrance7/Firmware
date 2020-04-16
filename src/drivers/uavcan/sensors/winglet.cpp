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
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	if(channel == nullptr)
	{
		// Something went wrong - no channel to publish on; return
		PX4_ERR("No channel to publush log message on");
		return;
	}

	PX4Winglet *winglet = (PX4Winglet *)channel->h_driver;

	if(winglet == nullptr)
	{
		return;
	}

	uint8_t node_id =  msg.getSrcNodeID().get();

	uint8_t gimbal_id = msg.gimbal_id;

	float x = msg.quaternion_xyzw[0];
	float y = msg.quaternion_xyzw[1];
	float z = msg.quaternion_xyzw[2];
	float w = msg.quaternion_xyzw[3];

	winglet->update(hrt_absolute_time(), node_id, gimbal_id, x, y, z, w);
}

// Driver initializer (uses PX4Winglet to handle multiple instances of winglet)
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
