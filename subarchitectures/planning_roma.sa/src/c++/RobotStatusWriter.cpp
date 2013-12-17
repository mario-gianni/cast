#include <RobotStatusWriter.hpp>

void RobotStatusWriter::listen_current_robot_status(const nifti_robot_driver_msgs::RobotStatus& msg)
{
    this->current_status = msg;
    
    eu::nifti::env::status::RobotStatusPtr rs = new eu::nifti::env::status::RobotStatus();	
	eu::nifti::env::status::ControllersStatusPtr s = new eu::nifti::env::status::ControllersStatus(); 
	eu::nifti::env::status::ControllersStatusPtr e = new eu::nifti::env::status::ControllersStatus(); 
	
	rs->batteryLevel = this->current_status.battery_level;
	rs->batteryStatus = this->current_status.battery_status;
	rs->brakeOn = this->current_status.brake_on;
	rs->scanningSpeed = this->current_status.scanning_speed;
	s->core = this->current_status.controllers_status.core;
	s->trackLeft = this->current_status.controllers_status.track_left;
	s->trackRight = this->current_status.controllers_status.track_right;
	s->flipperFrontLeft = this->current_status.controllers_status.flipper_front_left;
	s->flipperFrontRight = this->current_status.controllers_status.flipper_front_right;
	s->flipperRearLeft = this->current_status.controllers_status.flipper_rear_left;
	s->flipperRearRight = this->current_status.controllers_status.flipper_rear_right;
	e->core = this->current_status.controllers_error.core;
	e->trackLeft = this->current_status.controllers_error.track_left;
	e->trackRight = this->current_status.controllers_error.track_right;
	e->flipperFrontLeft = this->current_status.controllers_error.flipper_front_left;
	e->flipperFrontRight = this->current_status.controllers_error.flipper_front_right;
	e->flipperRearLeft = this->current_status.controllers_error.flipper_rear_left;
	e->flipperRearRight = this->current_status.controllers_error.flipper_rear_right;
	rs->status = s;
	rs->error = e;
	addToWorkingMemory(newDataID(), rs);
    
}

void RobotStatusWriter::start()
{
    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	ros::init(argc, argv, "RobotStatusWriter");
}

void RobotStatusWriter::runComponent()
{
    ros::NodeHandle node;
    ros::Subscriber sub_current_status;
    sub_current_status = node.subscribe("/robot_status",10,&RobotStatusWriter::listen_current_robot_status,this);
    
    ros::Rate r(10);
	
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
    	return new RobotStatusWriter();
  	}
}
