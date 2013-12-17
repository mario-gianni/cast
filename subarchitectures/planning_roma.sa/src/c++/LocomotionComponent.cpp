#include <LocomotionComponent.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nifti_robot_driver_msgs/FlipperCommand.h>

void LocomotionComponent::flipperActionReceiver(const WorkingMemoryChange& _wmc)
{
    FlipperActionPtr current = getMemoryEntry<FlipperAction>(_wmc.address);
    std::string id_action = _wmc.address.id;
    
    nifti_robot_driver_msgs::FlipperCommand command;
    std::string temp = current->component;
        
    if(temp.compare("flipper1") == 0)
    {
        //command.object_id = nifti_robot_driver_msgs::FlipperCommand::ID_FLIPPER_FRONT_LEFT
        command.object_id = 3;
    }
    else if(temp.compare("flipper2") == 0)
    {
        //command.object_id = nifti_robot_driver_msgs::FlipperCommand::ID_FLIPPER_FRONT_RIGHT
        command.object_id = 4;
    }
    else if(temp.compare("flipper3") == 0)
    {
        //command.object_id = nifti_robot_driver_msgs::FlipperCommand::ID_FLIPPER_REAR_RIGHT
        command.object_id = 6;
    }
    else if(temp.compare("flipper4") == 0)
    {
        //command.object_id = nifti_robot_driver_msgs::FlipperCommand::ID_FLIPPER_REAR_LEFT
        command.object_id = 5;
    }
    else
    {
        println("Action Unknown");
    }
        
    command.angle = current->alfa;
    this->flipper_pub.publish(command);
    
    current->status = EXECUTED;
    overwriteWorkingMemory(id_action,current);
}

void LocomotionComponent::differentialActionReceiver(const WorkingMemoryChange& _wmc)
{
    DifferentialActionPtr current = getMemoryEntry<DifferentialAction>(_wmc.address);
    std::string id_action = _wmc.address.id;
    
    std_msgs::Bool flag;
    if(current->flag == ON)
    {
        flag.data = true;
    }
    else if(current->flag == OFF)
    {
        flag.data = false;
    }
    else
    {
        println("Action Unknown");
    }
        
    this->diff_pub.publish(flag);
    
    current->status = EXECUTED;
    overwriteWorkingMemory(id_action,current);
}

void LocomotionComponent::start()
{
    println("************************************************************");
    println("********** LocomotionComponent ROS CAST Component **********");
    println("******************* Status: starting ***********************");
    println("************************************************************");
    
    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	
	println("******************* ROS init() *****************************");
	ros::init(argc, argv, "LocomotionComponent");
	
	ros::NodeHandle node;

	this->diff_pub = node.advertise<std_msgs::Bool>("brake",1);
	this->flipper_pub = node.advertise<nifti_robot_driver_msgs::FlipperCommand>("flipper_cmd",1);
	
	println("***************** Initializing filters *********************");
	addChangeFilter(createLocalTypeFilter<FlipperAction>(ADD),new MemberFunctionChangeReceiver<LocomotionComponent>(this,&LocomotionComponent::flipperActionReceiver));
	addChangeFilter(createLocalTypeFilter<DifferentialAction>(ADD),new MemberFunctionChangeReceiver<LocomotionComponent>(this,&LocomotionComponent::differentialActionReceiver));
}

void LocomotionComponent::runComponent()
{
    println("************************************************************");
    println("********** LocomotionComponent ROS CAST Component **********");
    println("******************* Status: running ************************");
    println("************************************************************");
}

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
    	return new LocomotionComponent();
  	}
}
