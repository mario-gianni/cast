#ifndef _ACTION_FAILURE_MANAGER_HPP_
#define _ACTION_FAILURE_MANAGER_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <planning_roma.hpp>

using namespace eu::nifti::Planning::slice;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ActionFailureManager : public cast::ManagedComponent
{
	public:
        ros::Publisher exe_pub;
    protected:
        virtual void start();
        virtual void runComponent();
        virtual void actionFailure(const cast::cdl::WorkingMemoryChange&);
};

#endif
