#ifndef _POSE_WRITER_HPP_
#define _POSE_WRITER_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <robot_position.hpp>

class PoseWriter : public cast::ManagedComponent
{
    protected:
        virtual void start();
        virtual void runComponent();
    private:
        std::string to_string(double);
        std::string to_string(eu::nifti::env::position::PosePtr);   
};

#endif
