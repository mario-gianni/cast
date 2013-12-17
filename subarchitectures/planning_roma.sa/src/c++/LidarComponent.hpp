#ifndef _LIDAR_COMPONENT_HPP_
#define _LIDAR_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <planning_roma.hpp>

using namespace eu::nifti::Planning::slice;
using namespace cast;
using namespace cast::cdl;
using namespace ros;

class LidarComponent : public ManagedComponent
{
    public:
    
        Publisher laser_pub;
		Publisher center_laser_pub;
    
    protected:
        virtual void start();
        virtual void runComponent();
        
        virtual void rotatingLaserActionReceiver(const WorkingMemoryChange&);
        virtual void centerLaserActionReceiver(const WorkingMemoryChange&);
};

#endif
