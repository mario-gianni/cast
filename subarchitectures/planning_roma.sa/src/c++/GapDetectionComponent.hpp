#ifndef _GAP_DETECTION_COMPONENT_HPP_
#define _GAP_DETECTION_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <planning_roma.hpp>
#include <gap.hpp>

using namespace cast;
using namespace cast::cdl;
using namespace eu::nifti::Planning::slice;
using namespace ros;
using namespace eu::nifti::env::terrain;
using namespace std;

class GapDetectionComponent : public ManagedComponent
{

public:
	ServiceClient client;

protected:
	virtual void start();
	virtual void runComponent();
	virtual void readAction(const WorkingMemoryChange&);

private:
	string to_string(double);

};

#endif
