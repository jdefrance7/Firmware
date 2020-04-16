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
#include <uORB/topics/sensor_winglet.h>

// UavcanWingletBridge Class
class UavcanWingletBridge : public UavcanCDevSensorBridgeBase
{
public:
	// Class name
	static const char *const NAME;

	// Constuctor
	UavcanWingletBridge(uavcan::INode &node);

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
	typedef uavcan::MethodBinder<UavcanWingletBridge *,
		void (UavcanWingletBridge::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand>&)>
		AngularCommandCallback;

	// UAVCAN subscription
	uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand, AngularCommandCallback> _sub_uavcan_angular_command;
};
