/*
    A ROS CAST Component which has to read a ROS msg from a topic and convert it into an ICE data structure
    has to be structured as follows:
    - a global variable in which store the ROS messages
    - start:
        - initialize ros
    - runComponent
        - NodeHandle
        - Subscriber -> Callback
        - while cycle on the lifecycle of the NodeHandle
    - callback
        - converts the ROS msg into ICE data structure
        - writes the ICE data structure into WM
*/

#include <string>
#include <POSNAVWriter.hpp>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_listener.h"

/* this function converts a double ia a string */
std::string POSNAVWriter::to_string(double value)
{
	std::stringstream ss;
	ss << value;
	return ss.str();
}

/* this function converts a ICE Node in a string */
std::string POSNAVWriter::to_string(eu::nifti::env::topograph::NodePtr n)
{
	std::string node = "node(" + n->label + ",";
	std::stringstream _x;
	_x << n->x;
	node = node + _x.str() + ",";
	std::stringstream _y;
	_y << n->y;
	node = node + _y.str() + ",";
	std::stringstream _flag;
	_flag << n->flag;
	node = node + _flag.str() + ")";
	return node;
}

void POSNAVWriter::listen_current_position(const voronoiseg::PoseAndTopologicalID& msg)
{
	this->current_position = msg;
	ROS_INFO("Callback function...");
	ROS_INFO("Subscribed to the current region in which the robot is");
	eu::nifti::env::topograph::NodePtr node = new eu::nifti::env::topograph::Node();
	node->label = "n" + to_string((int)current_position.id);
	node->x = current_position.pose.position.x;
	node->y = current_position.pose.position.y;
	node->flag = 0;
	eu::nifti::env::position::CurrentPosPtr cp = new eu::nifti::env::position::CurrentPos();
	cp->node = node;
	println("Current position of the robot: %s",to_string(cp->node).c_str());
	addToWorkingMemory(newDataID(), cp);
	
}

void POSNAVWriter::listen_base_position(const voronoiseg::PoseAndTopologicalID& msg)
{
	this->base_station = msg;
	ROS_INFO("Callback function...");
	ROS_INFO("Subscribed to the region in which the robot starts the task");
	eu::nifti::env::topograph::NodePtr node = new eu::nifti::env::topograph::Node();
	node->label = "n" + to_string((int)base_station.id);
	node->x = base_station.pose.position.x;
	node->y = base_station.pose.position.y;
	node->flag = 0;
	eu::nifti::env::position::BasePosPtr base = new eu::nifti::env::position::BasePos();
	base->node = node;
	println("Base position of the robot: %s",to_string(base->node).c_str());
	addToWorkingMemory(newDataID(), base);
	/* 
	    The base position has to be stored in the beginning when the robot starts 
	*/
	this->temp.shutdown();
}

void POSNAVWriter::start()
{
    println("POSNAVWriter ROS CAST Component");
    println("Status: starting...");
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	ros::init(argc, argv, "POSNAVWriter");
}

void POSNAVWriter::runComponent()
{	
    println("POSNAVWriter ROS CAST Component");
    println("Status: running...");
	ros::NodeHandle node;
	ros::Subscriber sub;
	
	sub = node.subscribe("/toposeg/currentpose_and_id",10,&POSNAVWriter::listen_current_position,this);
	this->temp = node.subscribe("/toposeg/currentpose_and_id",10,&POSNAVWriter::listen_base_position,this);
	
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
    	return new POSNAVWriter();
  	}
}
