#include "RobotStateReader.hpp"

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
		return new RobotStateReader();
	}
}	                  
			                       
void RobotStateReader::start() 
{
	char* argv[] = {};
	int argc = sizeof(argv)/sizeof(char *);
	ros::init(argc, argv, "nifti_planning_robotstatereader");

        ros::NodeHandle n;

        //ros::Subscriber sub = n.subscribe("chatter", 10, statusCallback);

        //ros::spin();

	//addChangeFilter(cast::createLocalTypeFilter<eu::nifti::navigation::slice::NavigationGoal>(cast::cdl::ADD),
	//	new cast::MemberFunctionChangeReceiver<NiftiNavigationGoals>(this, &NiftiNavigationGoals::ForwardGoal));

	println("started ros-cast robot state reader");
}

void RobotStateReader::runComponent() 
{
	return;
}

//void statusCallback(const std_msgs::String::ConstPtr& msg)
//{
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
//}


