// AUTHOR : SHANKER KESHAVDAS , DFKI SAARBRUECKEN

#ifndef FM_Find_Farthest_Free_Space_Point_HPP_
#define FM_Find_Farthest_Free_Space_Point_HPP_

#include <cast/architecture.hpp>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <LinearMath/btQuaternion.h>
#include <context.hpp>

#define MAP_FRAME "/map"
#define ROBOT_FRAME "/base_link"
#define MARKER_ARRAY_TOPIC "/toposeg/topo_marker_array" 
#define TIMEOUT_FOR_FINDING_FREESPACE 5.00 // Timeout for successfully finding freespace in front of the robot (seconds)
#define ROBOT_DIRECTION_SEARCH_ANGLE_DEGREES 30.00 // Search cone angle for the robot, in which the farthest free point is found
#define INCREMENT_IN_ROBOT_ANGLE_DEGREES 5.00 // Increment in the search cone, generating different search rays
#define GEOMETRICAL_EPSILON 0.001
#define ROBOT_LENGTH 0.67
#define OBSTACLE_SAFETY_RADIUS 0.1

class findFarthestFreeSpacePoint : public cast::ManagedComponent
{
	IceUtil::Mutex freeSpaceMutex;

	ros::NodeHandle n;
	tf::TransformListener listener;
	bool giveGoalDirectlyToNavStack;
	geometry_msgs::Quaternion xAxisQuat;
	visualization_msgs::MarkerArray recvdMarkerArray;
	visualization_msgs::MarkerArray freeSpaceMarkerArray; // Contains only markers that designate free space
	struct furthestPoseAndDistanceToIt
	{
		geometry_msgs::Pose furthest;
		double distanceToFurthest;
	};
	struct lineSegment
	{
		geometry_msgs::Point p1;
		geometry_msgs::Point p2;
	};

	std::vector<lineSegment> findLineSegmentsOfSquare(double, double, double);
	furthestPoseAndDistanceToIt findIntersectionOfRayAndLineSegment(btVector3, btVector3, lineSegment);
	void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr&);
	void processDialogueRequest(const cast::cdl::WorkingMemoryChange& _wmc);

	public:
	findFarthestFreeSpacePoint();
	~findFarthestFreeSpacePoint();

	bool checkForAddedMarkerCubesInMarkerArray();
	geometry_msgs::Pose findFarthestPoseInDirectionOfRobot();
	furthestPoseAndDistanceToIt findFurthestPoseAndDistanceToItInRayDirection(btVector3, btVector3);
	bool issueNavigationalGoal(geometry_msgs::Pose);
	void subscribeToAndStoreMarkerArray();

	// helper functions
	inline geometry_msgs::Pose convertTFTransformToGeoPose(tf::StampedTransform transform)
	{
		geometry_msgs::Pose robotPose;
		robotPose.position.x = transform.getOrigin().x();
		robotPose.position.y = transform.getOrigin().y();
		robotPose.position.z = transform.getOrigin().z();
		robotPose.orientation.x = transform.getRotation().x();
		robotPose.orientation.y = transform.getRotation().y();
		robotPose.orientation.z = transform.getRotation().z();
		robotPose.orientation.w = transform.getRotation().w();
		return robotPose;
	}

	inline geometry_msgs::Quaternion convertBTQuaternionToGeoQuaternion(btQuaternion quat)
	{
		geometry_msgs::Quaternion gQuat;
		gQuat.x = quat.x();
		gQuat.y = quat.y();
		gQuat.z = quat.z();
		gQuat.w = quat.w();
		return gQuat;	
	}

	inline void printLineSegment(lineSegment l)
	{
		log("Printing point 1 of line segment");
		printGeoPoint(l.p1);
		log("Printing point 2 of line segment");		
		printGeoPoint(l.p2);
	}

	inline void printGeoPoint(geometry_msgs::Point p)
	{
		log("Point x: %3.2f y: %3.2f z: %3.2f",p.x,p.y,p.z);	
	}

	inline lineSegment createLineSegment(btVector3 px, btVector3 py)
	{
		lineSegment l;
		l.p1.x = px.x();
		l.p1.y = px.y();
		l.p1.z = px.z();
		l.p2.x = py.x();
		l.p2.y = py.y();
		l.p2.z = py.z();
		return l;
	}

	// test functions
	
	bool testSuccessfulIntersectionOfRayAndLineSegment(btVector3, btVector3, lineSegment);

	protected:
	virtual void start();
	virtual void runComponent();
	virtual void configure(const std::map<std::string, std::string>&);
};

#endif
