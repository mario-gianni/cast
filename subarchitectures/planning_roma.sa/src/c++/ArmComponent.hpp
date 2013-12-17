#ifndef _ARM_COMPONENT_HPP_
#define _ARM_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <planning_roma.hpp>

class ArmComponent : public cast::ManagedComponent
{
protected:
	virtual void start();
	virtual void runComponent();
	virtual void setHeight(const cast::cdl::WorkingMemoryChange&);
};

#endif
