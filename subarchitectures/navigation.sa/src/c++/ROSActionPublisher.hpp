#ifndef ROS_ACTION_PUBLISHER_HPP_
#define ROS_ACTION_PUBLISHER_HPP_
      
#include <cast/architecture.hpp>
#include <ros/ros.h> 
#include "autogen/navigation.hpp"
#include "geometry_msgs/Twist.h"
			                 
class ROSActionPublisher: public cast::ManagedComponent 
{
				 
	public: 
	float linearx;
	float lineary;
	float linearz;
	float angularx;
	float angulary;
	float angularz;

	geometry_msgs::Twist TwistActionMsg;

	virtual void start();
                  
	protected:
	virtual void runComponent();
	virtual void GetParameters(const cast::cdl::WorkingMemoryChange&);
	//virtual void ForwardAction(const cast::cdl::WorkingMemoryChange&);
         
	private:
	ros::Publisher NavigationPublisher;

	//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
};     
      
#endif
