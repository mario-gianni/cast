#ifndef ROS_VP_PLANNER_HPP_
#define ROS_VP_PLANNER_HPP_
      
#include <cast/architecture.hpp>
#include <ros/ros.h>
			                 
class VPPlanner: public cast::ManagedComponent 
{				 
	public: 

	virtual void start();
                  
	protected:
	virtual void runComponent();
	virtual void vantagePointsReceived(const cast::cdl::WorkingMemoryChange&);
	ros::Publisher vantagePointsPub;
         
	//private:
};     
      
#endif
