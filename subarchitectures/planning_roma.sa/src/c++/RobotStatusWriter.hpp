#ifndef _ROBOT_STATUS_WRITER_HPP_
#define _ROBOT_STATUS_WRITER_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <robot_status.hpp>
#include <nifti_robot_driver_msgs/RobotStatus.h>

class RobotStatusWriter : public cast::ManagedComponent
{
    public:
        nifti_robot_driver_msgs::RobotStatus current_status;
        void listen_current_robot_status(const nifti_robot_driver_msgs::RobotStatus&);
        
    protected:
        virtual void start();
        virtual void runComponent();
};

#endif
