#ifndef ROS_DUMMY_PLANNINGTASK_GENERATOR_HPP_
#define ROS_DUMMY_PLANNINGTASK_GENERATOR_HPP_
      
#include <cast/architecture.hpp>
#include "autogen/planning.hpp"
#include <math.h>

using namespace std;
using namespace eu::nifti::planning::slice;
	                 
class DummyPlanningTaskGenerator: public cast::ManagedComponent 
{				 
	public: 

	virtual void start();
                  
	protected:
	virtual void runComponent();

	private:
};     
      
#endif
