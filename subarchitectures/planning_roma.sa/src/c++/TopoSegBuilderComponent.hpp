#ifndef _TOPO_SEG_BUILDER_COMPONENT_HPP_
#define _TOPO_SEG_BUILDER_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <planning_roma.hpp>
#include <module_control_action/ControlAction.h>

using namespace eu::nifti::Planning::slice;
using namespace cast;
using namespace cast::cdl;
using namespace eu::nifti::env::topograph;

typedef actionlib::SimpleActionClient<module_control_action::ControlAction> TopoSegClient;

class TopoSegBuilderComponent : public ManagedComponent
{
    public:
        TopoSegClient* client;
    
    protected:
        virtual void start();
        virtual void runComponent();
        
        virtual void topoSegActionReceived(const WorkingMemoryChange&);
};

#endif
