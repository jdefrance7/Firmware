#pragma once

#include "sensor_bridge.hpp"

#include <drivers/drv_hrt.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>

#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
// #include <uORB/topics/log_message.h>
// #include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_winglet.h>

class UavcanWingletBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanWingletBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	int init_driver(uavcan_bridge::Channel *channel) override;

	void uavcan_key_value_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg);
	// void uavcan_log_message_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage> &msg);
	void uavcan_angular_command_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand> &msg);

	typedef uavcan::MethodBinder<UavcanWingletBridge *,
		void (UavcanWingletBridge::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>&)>
		KeyValueCallback;

	// typedef uavcan::MethodBinder<UavcanWingletBridge *,
	// 	void (UavcanWingletBridge::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>&)>
	// 	LogMessageCallback;

	typedef uavcan::MethodBinder<UavcanWingletBridge *,
		void (UavcanWingletBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand>&)>
		AngularCommandCallback;

	uavcan::Subscriber<uavcan::protocol::debug::KeyValue, KeyValueCallback> _sub_uavcan_key_value;
	// uavcan::Subscriber<uavcan::protocol::debug::LogMessage, LogMessageCallback> _sub_uavcan_log_message;
	uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand, AngularCommandCallback> _sub_uavcan_angular_command;
};
