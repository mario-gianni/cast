#ifndef _DIAGNOSTIC_COMPONENT_HPP_
#define _DIAGNOSTIC_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <monitoring_msgs/Battery.h>
#include <monitoring_msgs/WiFi.h>
#include <planning_roma.hpp>
#include <diagnostic.hpp>

using namespace eu::nifti::Planning::slice;
using namespace cast;
using namespace cast::cdl;
using namespace eu::nifti::env::diagnostic;
using namespace monitoring_msgs;

class DiagnosticComponent : public cast::ManagedComponent
{
public:

	CheckBatteryActionPtr battery_current_action;
	CheckWifiActionPtr wifi_current_action;

	ros::Subscriber battery_sub;
	ros::Subscriber wifi_sub;
	void listen_battery_status(const BatteryConstPtr&);
	void listen_wifi_status(const WiFiConstPtr&);
protected:
	virtual void start();
	virtual void runComponent();
    virtual void readCheckBatteryAction(const WorkingMemoryChange&);
    virtual void readCheckWifiAction(const WorkingMemoryChange&);


};

#endif
