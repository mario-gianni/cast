#include <DiagnosticComponent.hpp>

void DiagnosticComponent::readCheckBatteryAction(const WorkingMemoryChange& _wmc)
{
}

void DiagnosticComponent::readCheckWifiAction(const WorkingMemoryChange& _wmc)
{
}

void DiagnosticComponent::listen_battery_status(const BatteryConstPtr& msg)
{
	Battery status = *msg;
	BatteryStatusPtr level = new BatteryStatus();

	if (0.5 <= status.currentBatteryLevel) {
		level->level = eu::nifti::env::diagnostic::BHIGH;
	}
	else if (0.25 <= status.currentBatteryLevel) {
		level->level = eu::nifti::env::diagnostic::BMEDIUM;
	}
	else {
		level->level = eu::nifti::env::diagnostic::BLOW;
	}
	addToWorkingMemory(newDataID(),level);
}

void DiagnosticComponent::listen_wifi_status(const WiFiConstPtr& msg)
{
	WiFi status = *msg;
	WiFiStatusPtr link = new WiFiStatus();

	if (status.linkQuality > 0.75) {
		link->quality = eu::nifti::env::diagnostic::GOOD;
	}
	else if (status.linkQuality >= 0.5) {
		link->quality = eu::nifti::env::diagnostic::MODERATE;
	}
	else if (status.linkQuality < 0.1) {
		link->quality = eu::nifti::env::diagnostic::LOST;
	}
	else if (status.linkQuality < 0.5) {
		link->quality = eu::nifti::env::diagnostic::WEAK;
	}
	addToWorkingMemory(newDataID(),link);
}

void DiagnosticComponent::start()
{
	println("************************************************************");
	println("********* DiagnosticComponent ROS CAST Component ***********");
	println("******************* Status: starting ***********************");
	println("************************************************************");
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);

	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "DiagnosticComponent");
}

void DiagnosticComponent::runComponent()
{
	println("************************************************************");
	println("********* DiagnosticComponent ROS CAST Component ***********");
	println("******************** Status: running ***********************");
	println("************************************************************");

	ros::NodeHandle node;
	battery_sub = node.subscribe("/monitoring/batteryInfo",10,&DiagnosticComponent::listen_battery_status,this);
	wifi_sub = node.subscribe("/monitoring/wifiLink",10,&DiagnosticComponent::listen_wifi_status,this);

	ros::Rate r(20);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}

extern "C"
{
cast::CASTComponentPtr newComponent()
{
	return new DiagnosticComponent();
}
}
