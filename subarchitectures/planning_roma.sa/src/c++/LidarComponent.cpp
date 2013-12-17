#include <LidarComponent.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

void LidarComponent::rotatingLaserActionReceiver(const WorkingMemoryChange& _wmc)
{
    RotatingLaserActionPtr current = getMemoryEntry<RotatingLaserAction>(_wmc.address);
    std::string id_action = _wmc.address.id;
    
    std_msgs::Float64 speed;
    speed.data = current->speed;
    this->laser_pub.publish(speed);
    sleep(current->time);
    
    current->status = EXECUTED;
    overwriteWorkingMemory(id_action,current);
}

void LidarComponent::centerLaserActionReceiver(const WorkingMemoryChange& _wmc)
{
    CenterLaserActionPtr current = getMemoryEntry<CenterLaserAction>(_wmc.address);
    std::string id_action = _wmc.address.id;
    
    std_msgs::Bool command;
    command.data = true;
    this->center_laser_pub.publish(command);
    
    current->status = EXECUTED;
    overwriteWorkingMemory(id_action,current);
}

void LidarComponent::start()
{
    println("************************************************************");
    println("************* LidarComponent ROS CAST Component ************");
    println("******************* Status: starting ***********************");
    println("************************************************************");
    
    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	
	println("******************* ROS init() *****************************");
	ros::init(argc, argv, "LidarComponent");
	
	ros::NodeHandle node;
	this->laser_pub = node.advertise<std_msgs::Float64>("scanning_speed_cmd",1);
	this->center_laser_pub = node.advertise<std_msgs::Bool>("laser_center",1);
	
	println("***************** Initializing filters *********************");
	addChangeFilter(createLocalTypeFilter<RotatingLaserAction>(ADD),new MemberFunctionChangeReceiver<LidarComponent>(this,&LidarComponent::rotatingLaserActionReceiver));
	addChangeFilter(createLocalTypeFilter<CenterLaserAction>(ADD),new MemberFunctionChangeReceiver<LidarComponent>(this,&LidarComponent::centerLaserActionReceiver));	
}

void LidarComponent::runComponent()
{
    println("************************************************************");
    println("************* LidarComponent ROS CAST Component ************");
    println("******************* Status: running ************************");
    println("************************************************************");
}

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
    	return new LidarComponent();
  	}
}
