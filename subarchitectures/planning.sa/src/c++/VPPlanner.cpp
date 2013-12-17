#include "VPPlanner.hpp"
#include <eu/nifti/env/CarObjectOfInterest.hpp>
#include <geometry_msgs/PoseArray.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
		return new VPPlanner();
	}
}	                  
			                       
void VPPlanner::start() 
{
	char* argv[] = {};
	int argc = sizeof(argv)/sizeof(char *);
	ros::init(argc, argv, "nifti_planning_vp_planner");

	//addChangeFilter(cast::createGlobalTypeFilter<eu::nifti::env::CarObjectOfInterest>(cast::cdl::OVERWRITE),
	//	new cast::MemberFunctionChangeReceiver<VPPlanner>(this, &VPPlanner::vantagePointsReceived));

	addChangeFilter(cast::createChangeFilter<eu::nifti::env::CarObjectOfInterest>(cast::cdl::OVERWRITE, "FEMAreasGenerator", "", "", cast::cdl::ALLSA),
		new cast::MemberFunctionChangeReceiver<VPPlanner>(this, &VPPlanner::vantagePointsReceived));


	println("started ros-cast component nifti_planning_vp_planner");
}

void VPPlanner::runComponent() 
{
	ros::NodeHandle n;
	vantagePointsPub = n.advertise<geometry_msgs::PoseArray>("/vpplanner/vantage_points", 5);

	return;
}

void VPPlanner::vantagePointsReceived(const cast::cdl::WorkingMemoryChange & _wmc)
{
	println("vantage points received");
	
	eu::nifti::env::CarObjectOfInterestPtr car = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(_wmc.address);
	geometry_msgs::PoseArray vantagePoints;
	
	for(unsigned int i = 0; i < car->vantagePoints.size(); i++)
	{
		geometry_msgs::Pose pose;
		pose.position.x = car->vantagePoints[i]->pose.x;
		pose.position.y = car->vantagePoints[i]->pose.y;
		pose.position.z = car->vantagePoints[i]->pose.z;
		btVector3 axisOfRotation(0,0,1);
		btQuaternion quat(axisOfRotation, car->vantagePoints[i]->pose.theta);
		pose.orientation.x = quat.x();
		pose.orientation.y = quat.y();
		pose.orientation.z = quat.z();
		pose.orientation.w = quat.w();
		vantagePoints.poses.push_back(pose);
	}

	println("now publisheing to ros");
	vantagePointsPub.publish(vantagePoints);
}
