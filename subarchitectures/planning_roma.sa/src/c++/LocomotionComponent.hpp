#ifndef _LOCOMOTION_COMPONENT_HPP_
#define _LOCOMOTION_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <planning_roma.hpp>

using namespace eu::nifti::Planning::slice;
using namespace cast;
using namespace cast::cdl;
using namespace ros;

class LocomotionComponent : public ManagedComponent
{
    public:
    
		Publisher flipper_pub;
		Publisher diff_pub;
		
    protected:
        virtual void start();
        virtual void runComponent();
        
        virtual void flipperActionReceiver(const WorkingMemoryChange&);
        virtual void differentialActionReceiver(const WorkingMemoryChange&);
};

#endif
