#ifndef ROS_NAVIGATION_GOALS_HPP_
#define ROS_NAVIGATION_GOALS_HPP_
      
#include <cast/architecture.hpp>
#include <ros/ros.h> 
#include "autogen/navigation.hpp"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

#define PI 3.14159265

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
			                 
class NiftiNavigationGoals: public cast::ManagedComponent 
{				 
	public: 
	MoveBaseClient* ac;
	virtual void start();
                  
	protected:
	virtual void runComponent();
	virtual void ExecuteAction(const cast::cdl::WorkingMemoryChange&);
         
	private:
	ros::Publisher NavigationPublisher;
};     
      
#endif
