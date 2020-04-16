#include "winglet.hpp"

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

#include <lib/drivers/winglet/PX4Winglet.hpp>

const char *const UavcanWingletBridge::NAME = "winglet";

#define UAVCAN_WINGLET_BASE_DEVICE_PATH "/dev/uavcan/winglet"

UavcanWingletBridge::UavcanWingletBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase(
		"uavcan_winglet",
		UAVCAN_WINGLET_BASE_DEVICE_PATH,
		UAVCAN_WINGLET_BASE_DEVICE_PATH,
		ORB_ID(sensor_winglet)
	),
	_sub_uavcan_key_value(node),
	// _sub_uavcan_log_message(node),
	_sub_uavcan_angular_command(node)
{
}

int UavcanWingletBridge::init()
{
	int res = device::CDev::init();

	if(res < 0) {
		return res;
	}

	res = _sub_uavcan_key_value.start(KeyValueCallback(this, &UavcanWingletBridge::uavcan_key_value_cb));

	if(res < 0) {
		PX4_ERR("failed to start uavcan key value sub: %d", res);
		return res;
	}

	// res = _sub_uavcan_log_message.start(LogMessageCallback(this, &UavcanWingletBridge::uavcan_log_message_cb));

	// if(res < 0) {
	// 	PX4_ERR("failed to start uavcan log message sub: %d", res);
	// 	return res;
	// }

	res = _sub_uavcan_angular_command.start(AngularCommandCallback(this, &UavcanWingletBridge::uavcan_angular_command_cb));

	if(res < 0) {
		PX4_ERR("failed to start uavcan angular command sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanWingletBridge::uavcan_key_value_cb(const
	uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg)
{
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	if(channel == nullptr)
	{
		// Something went wrong - no channel to publish on; return
		PX4_ERR("No channel to publush key value on");
		return;
	}

	PX4Winglet *winglet = (PX4Winglet *)channel->h_driver;

	if(winglet == nullptr)
	{
		return;
	}

	const uint8_t node_id = msg.getSrcNodeID().get();

	const char key = msg.key[0];

	const float value = msg.value;

	winglet->update(hrt_absolute_time(), node_id, key, value);
}

// void
// UavcanWingletBridge::uavcan_log_message_cb(const
// 	uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage> &msg)
// {
// 	uint8_t node_id = msg.getSrcNodeID().get();
// 	uint8_t level 	= 1;

// 	char text[90];
// 	int text_size;
// 	for(text_size = 0; text_size < 90; text_size++)
// 	{
// 		if(msg.text[text_size] != 0)
// 		{
// 			text[text_size] = msg.text[text_size];
// 		}
// 		else
// 		{
// 			break;
// 		}
// 	}

// 	PX4_INFO("node = %d", node_id);
// 	PX4_INFO("  level = %d", level);
// 	PX4_INFO("  text  = %s", text);

// 	lock();
// 	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());
// 	unlock();

// 	if(channel == nullptr)
// 	{
// 		// Something went wrong - no channel to publish on; return
// 		PX4_ERR("No channel to publush log message on");
// 		return;
// 	}

// 	PX4Winglet *winglet = (PX4Winglet *)channel->h_driver;

// 	if(winglet == nullptr)
// 	{
// 		return;
// 	}

// 	winglet->update(hrt_absolute_time(), 1, text, text_size);
// }

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
