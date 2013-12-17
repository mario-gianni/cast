#ifndef ROS_ACTION_PUBLISHER_HPP_
#define ROS_ACTION_PUBLISHER_HPP_
      
#include <cast/architecture.hpp>
#include <ros/ros.h>
#include "autogen/planning.hpp"
			                 
class RobotStateReader: public cast::ManagedComponent 
{				 
	public: 

	virtual void start();
                  
	protected:
	virtual void runComponent();
	//virtual void ForwardGoal(const cast::cdl::WorkingMemoryChange&);
         
	private:
	//ros::Publisher NavigationPublisher;
};     
      
#endif
