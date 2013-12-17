#ifndef ROS_PLANNING_BRIDGE_HPP_
#define ROS_PLANNING_BRIDGE_HPP_
      
#include <cast/architecture.hpp>
#include <ros/ros.h>
#include "autogen/planning.hpp"
#include <eclipse_prolog_msgs/Task.h>
#include <math.h>
#include <IceUtil/Mutex.h>

#define PI 3.14159265

using namespace std;
using namespace eu::nifti::planning::slice;
			                 
class PlanningBridge: public cast::ManagedComponent 
{				 
	public: 
	ros::ServiceClient client;
	PlanningTaskPtr currentPlanningTask;
	bool sendPlan;
	virtual void start();
                  
	protected:
	virtual void runComponent();
	virtual void planningTaskReceived(const cast::cdl::WorkingMemoryChange&);

	private:
	IceUtil::Mutex mut;
	
};     
      
#endif
