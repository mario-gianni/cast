#include "ROSActionPublisher.hpp"

extern "C" 
{
	cast::CASTComponentPtr ROSActionPublisherComponent() 
	{
		return new ROSActionPublisher();
	}
}      	                  
			                       
void ROSActionPublisher::start() 
{
	char* argv[] = {};
	int argc = sizeof(argv)/sizeof(char *);
	ros::init(argc, argv, "NavigationActionMonitor");

	addChangeFilter(cast::createLocalTypeFilter<eu::nifti::navigation::slice::TwistMsg>(cast::cdl::ADD),
		new cast::MemberFunctionChangeReceiver<ROSActionPublisher>(this, &ROSActionPublisher::GetParameters));

	println("started ros action publisher");
}            
					                       
void ROSActionPublisher::GetParameters(const cast::cdl::WorkingMemoryChange & _wmc) 
{
	println("got an action");
	eu::nifti::navigation::slice::TwistMsgPtr ActionMsg = getMemoryEntry<eu::nifti::navigation::slice::TwistMsg>(_wmc.address);
	TwistActionMsg.linear.x = ActionMsg->linx;
	TwistActionMsg.linear.y = ActionMsg->liny;
	TwistActionMsg.linear.z = ActionMsg->linz;
	TwistActionMsg.angular.x = ActionMsg->angx;
	TwistActionMsg.angular.y = ActionMsg->angy;
	TwistActionMsg.angular.z = ActionMsg->angz;
							            
	ros::NodeHandle NavigationPublisherNode;
	NavigationPublisher = NavigationPublisherNode.advertise<geometry_msgs::Twist>("cmd_vel",10);
	ros::Rate loop_rate(0);
	NavigationPublisher.publish(TwistActionMsg);
	ros::spinOnce();
	ROS_INFO("started ros node");
}
									      
void ROSActionPublisher::runComponent() 
{
	return;
}

