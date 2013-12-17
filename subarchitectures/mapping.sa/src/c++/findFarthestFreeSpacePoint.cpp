
#ifndef FM_Find_Farthest_Free_Space_Point_CPP_
#define FM_Find_Farthest_Free_Space_Point_CPP_

// This program calculates the maximal reachable free space point in the forward direction (within an angle) that the robot can proceed to. This is calculated using the visualization marker array from the toposeg node (which denotes visible areas by cubes). 
// This calculation is requested by the dialogue SA, and is responded to. The request is eu::nifti::slice::context::WhatIsForward

#include <findFarthestFreeSpacePoint.hpp>

findFarthestFreeSpacePoint::findFarthestFreeSpacePoint() : n("~")
{
	xAxisQuat.x = 0;
	xAxisQuat.y = 0;
	xAxisQuat.z = 0;
	xAxisQuat.w = 1;
}

findFarthestFreeSpacePoint::~findFarthestFreeSpacePoint()
{
	
}

// Copy the elements of the marker array that designate free space. Return if any such elements have been found.
bool findFarthestFreeSpacePoint::checkForAddedMarkerCubesInMarkerArray()
{
	freeSpaceMutex.lock();

	bool erasedFreeSpaceMarkerArray = false;
	bool foundFreeSpace = false;

	for(unsigned int ctr = 0; ctr < recvdMarkerArray.markers.size(); ctr++)
	{
		// If the type of marker is cube (value:1) and the action is ADD (value:0), it designates free space that has been added (current). We save these elements.
		if(recvdMarkerArray.markers[ctr].type==1 && recvdMarkerArray.markers[ctr].action==0)
		{
			if(erasedFreeSpaceMarkerArray == false)
			{
				freeSpaceMarkerArray.markers.clear();
				erasedFreeSpaceMarkerArray = true;
				foundFreeSpace = true;
			}
			freeSpaceMarkerArray.markers.push_back(recvdMarkerArray.markers[ctr]);
		}
	}

	freeSpaceMutex.unlock();

	return foundFreeSpace;
}

// Find farthest free space pose in the direction of the robot
geometry_msgs::Pose findFarthestFreeSpacePoint::findFarthestPoseInDirectionOfRobot()
{

	freeSpaceMutex.lock();

	// Find the current pose of the robot
	tf::StampedTransform transform;
	bool gotTransform = false;
	while(gotTransform == false)
	{
		ros::Duration(1).sleep();
		try{
			debug("Waiting for tf transform between MAP_FRAME and ROBOT_FRAME");
			listener.waitForTransform(MAP_FRAME, ROBOT_FRAME, ros::Time(0), ros::Duration(5.0));
			listener.lookupTransform(MAP_FRAME, ROBOT_FRAME, ros::Time(0), transform);
			gotTransform = true;
		}
		catch(tf::TransformException e)
		{
			log("Was looking up transform between MAP_FRAME and ROBOT_FRAME. Will try again. Error: %s",e.what());
		}
	}
	debug("Got transform between MAP_FRAME and ROBOT_FRAME");
	geometry_msgs::Pose robotPose = convertTFTransformToGeoPose(transform);
	robotPose.position.z = 0;

	// Find the vector corresponding to this pose
	btQuaternion robotPosQuat(robotPose.orientation.x,robotPose.orientation.y,robotPose.orientation.z,robotPose.orientation.w);
	btVector3 unitXVec(1,0,0);
	// Rotate the unit X vector by the Quaternion to give the robot direction as a vector
	btVector3 robotDirVec = quatRotate(robotPosQuat,unitXVec);
	robotDirVec = robotDirVec.normalized();
	// Force the robot direction to be 2D
	btVector3 robotDirVec2D(robotDirVec.x(),robotDirVec.y(),0);

	// Represent the position of the robot as a btVector3 for computation ease
	btVector3 robotPosition(robotPose.position.x,robotPose.position.y,0);
	debug("The robot position is %6.4f, %6.4f, %6.4f and the direction is vector %6.4f, %6.4f, %6.4f", robotPosition.x(), robotPosition.y(), robotPosition.z(), robotDirVec2D.x(), robotDirVec2D.y(), robotDirVec2D.z());

	// A Ray can be represented by a point and a vector 
	// We would like to have a few rays that represent a variation in the robot's direction
	// The resultant rays would have the same start point, but different vectors
	double robotSearchConeAngle = ROBOT_DIRECTION_SEARCH_ANGLE_DEGREES * (M_PI/180);
	double searchConeIncrement = INCREMENT_IN_ROBOT_ANGLE_DEGREES * (M_PI/180);
	geometry_msgs::Pose furthestPoseFound;
	furthestPoseFound.position = robotPose.position;
	furthestPoseFound.orientation = xAxisQuat;
	double furthestDistanceFound = 0; 
	for(double searchConeModification = -(robotSearchConeAngle/2); searchConeModification <= (robotSearchConeAngle/2); searchConeModification += searchConeIncrement)
	{
		// Find the quaternion that represents a rotation about the Z axis (XY plane) by the searchConeModification angle
		btVector3 unitZAxis(0,0,1);
		btQuaternion searchConeModQuat(unitZAxis,searchConeModification);
		// Rotate the Robot Direction vector with this Quaternion to get the modified direction of the robot
		btVector3 modRobotDirVec = quatRotate(searchConeModQuat,robotDirVec2D);
		
		// Find the furthest Pose and Distance to it in this direction
		furthestPoseAndDistanceToIt furPoseDist = findFurthestPoseAndDistanceToItInRayDirection(robotPosition,modRobotDirVec);

		// Check if this is the furthest among the directions of the search cone
		if(furthestDistanceFound < furPoseDist.distanceToFurthest)
		{
			debug("In the direction %6.4f radians from robot position we found a far pose at %6.4f m", searchConeModification , furPoseDist.distanceToFurthest);
			furthestDistanceFound = furPoseDist.distanceToFurthest;
			furthestPoseFound = furPoseDist.furthest;
		}
	}
	
	debug("The furthest robot pose has been found to be position %6.4f, %6.4f, %6.4f and the direction  %6.4f, %6.4f, %6.4f, %6.4f", furthestPoseFound.position.x, furthestPoseFound.position.y, furthestPoseFound.position.z, furthestPoseFound.orientation.x,  furthestPoseFound.orientation.y,  furthestPoseFound.orientation.z,  furthestPoseFound.orientation.w);
	debug("The direction to this pose was %6.4f meters", furthestDistanceFound);

	freeSpaceMutex.unlock();

	return furthestPoseFound;
}

// Given a specific position and direction (a ray) , find the furthest pose and distance to the pose in the free space that is represented by the free space marker array in the direction of the ray
findFarthestFreeSpacePoint::furthestPoseAndDistanceToIt findFarthestFreeSpacePoint::findFurthestPoseAndDistanceToItInRayDirection (btVector3 position, btVector3 direction)
{
	debug("Checking in direction %3.2f , %3.2f , %3.2f ", direction.x(),direction.y(),direction.z());
	furthestPoseAndDistanceToIt furPoseDist;
	// Default values for furPoseDist
	// Converting the direction into a quaternion
	direction = direction.normalized();
	btVector3 unitXAxis(1,0,0);
	// Angle between the direction and X axis atan2(y,x) -> -pi to  pi
	double angleBet = atan2(direction.y(),direction.x());
	btQuaternion dirRay(btVector3(0,0,1),angleBet);
	furPoseDist.furthest.position.x = position.x();
	furPoseDist.furthest.position.y = position.y(); 
	furPoseDist.furthest.position.z = position.z();
	furPoseDist.furthest.orientation = convertBTQuaternionToGeoQuaternion(dirRay);
 	furPoseDist.distanceToFurthest = 0;
	
	debug("There are a total of %d freespacemarkers",(int) freeSpaceMarkerArray.markers.size());
	for(unsigned int ctr = 0; ctr < freeSpaceMarkerArray.markers.size(); ctr++)
	{
		// Compute the line segments corresponding to a square given center and edge length
		std::vector<lineSegment> lines = findLineSegmentsOfSquare(freeSpaceMarkerArray.markers[ctr].pose.position.x, freeSpaceMarkerArray.markers[ctr].pose.position.y, freeSpaceMarkerArray.markers[ctr].scale.x);
		
		for(unsigned int lineCtr = 0; lineCtr < lines.size(); lineCtr ++)
		{ 
			// Find intersection of ray and lineSegment giving the resulting pose and distance
			furthestPoseAndDistanceToIt furPoseDistIntersec = findIntersectionOfRayAndLineSegment(position, direction, lines[lineCtr]);
			// Compare it to the previous distances and store the largest one
			if(furPoseDist.distanceToFurthest < furPoseDistIntersec.distanceToFurthest)
			{
				debug("We found a max distance to a marker at %6.4f m",furPoseDistIntersec.distanceToFurthest);
				furPoseDist.distanceToFurthest = furPoseDistIntersec.distanceToFurthest;
				furPoseDist.furthest = furPoseDistIntersec.furthest;
			}
		}
	}
	return furPoseDist;
}


// Given a specific position and a direction (a ray), find the intersection between it and a line segment. Return the resulting pose and the distance to the intersection.
findFarthestFreeSpacePoint::furthestPoseAndDistanceToIt findFarthestFreeSpacePoint::findIntersectionOfRayAndLineSegment(btVector3 position, btVector3 direction, findFarthestFreeSpacePoint::lineSegment line)
{

	furthestPoseAndDistanceToIt furPosDist;
	// Default values for furPosDist
	// Converting the direction into a quaternion
	direction = direction.normalized();
	btVector3 unitXAxis(1,0,0);
	// Angle between the direction and X axis atan2(y,x) -> -pi to  pi
	double angleBet = atan2(direction.y(),direction.x());
	btQuaternion dirRay(btVector3(0,0,1),angleBet);
	furPosDist.furthest.position.x = position.x();
	furPosDist.furthest.position.y = position.y(); 
	furPosDist.furthest.position.z = position.z();
	furPosDist.furthest.orientation = convertBTQuaternionToGeoQuaternion(dirRay);
 	furPosDist.distanceToFurthest = 0;

	// In 2D Space,
	// A ray can be represented as Point(x1,y1) + i * Vector(vx,vy) {i > 0}
	// A line segment as Point(x2,y2) + j * Distance(x3-x2,y3-y2) {0 < j < 1 , the other point is (x3,y3)}
	// Solution : x1 + i(vx) = x2 + j(x3-x2) , y1 + i(vy) = y2 + j(y3-y2)
	// Or : Vx * i + (x2 - x3) * j = (x2-x1) , Vy * i + (y2 - y3) * j = (y2 - y1)
	// Or : a1 * i + b1 * j = c1 , a2 * i + b2 * j = c2
	double x1 = position.x(); double y1 = position.y(); double vx = direction.x(); double vy = direction.y();
	double x2 = line.p1.x; double y2 = line.p1.y; double x3 = line.p2.x; double y3 = line.p2.y;
	double a1 = vx; double b1 = x2 - x3; double c1 = x2 - x1;
	double a2 = vy; double b2 = y2 - y3; double c2 = y2 - y1;

	// If a1 = 0 or a1b2 - a2b1 = 0; the system is indeterminate 
	if( a1==0 || fabs((a1 * b2) - (a2 * b1)) < GEOMETRICAL_EPSILON)
	{
		// Return the start position and orientation
		return furPosDist;
	}

	// Otherwise, solve
	// i = (c1(a1b2-a2b1) - b1(a1c2-a2c1)) / a1(a1b2-a2b1)
	// j = (a1c2-a2c1) / (a1b2-a2b1)
	double i = (c1*((a1*b2)-(a2*b1)) - b1*((a1*c2)-(a2*c1))) / (a1*((a1*b2)-(a2*b1)));
	double j = ((a1*c2)-(a2*c1)) / ((a1*b2)-(a2*b1));

	// Ray / Line Segment conditions
	if( i <=0 || j > 1 || j < 0 )
	{
		return furPosDist;
	}

	// Distance to that point is root of sum of square of the ray components
	// i * pow((vx^2 + vy^2),0.5)
	double distance = i * pow(pow(vx,2) + pow(vy,2),0.5);

	// For safety reasons, we reduce this distance by the robotlength/2 and the safety radius for obstacles
	double safetymeasure = (ROBOT_LENGTH/2) + (OBSTACLE_SAFETY_RADIUS);
	double safedistance  = distance - safetymeasure;
	double safetyratio = 0;
	if(safedistance > 0)
	{
		safetyratio = safedistance / distance;
	}
	else
	{
		safetyratio = 0;
		safedistance = 0;
	}

	// Else calculate the furthest point
	furPosDist.furthest.position.x = x1 + safetyratio * i * vx;
	furPosDist.furthest.position.y = y1 + safetyratio * i * vy;
	furPosDist.furthest.position.z = 0;
	// Also calculate the distance
	furPosDist.distanceToFurthest = safedistance;	
	
	return furPosDist;
}

// Find line segments of square given edge length and center
std::vector<findFarthestFreeSpacePoint::lineSegment> findFarthestFreeSpacePoint::findLineSegmentsOfSquare(double centerX, double centerY, double edgeLen)
{
	// Find corner points
	double inc = edgeLen / (pow(2,0.5));
	geometry_msgs::Point cor1; cor1.x = centerX + inc; cor1.y = centerY + inc; cor1.z = 0;
	geometry_msgs::Point cor2; cor2.x = centerX + inc; cor2.y = centerY - inc; cor2.z = 0;
	geometry_msgs::Point cor3; cor3.x = centerX - inc; cor3.y = centerY - inc; cor3.z = 0;
	geometry_msgs::Point cor4; cor4.x = centerX - inc; cor4.y = centerY + inc; cor4.z = 0;
	
	// Make line segments
	std::vector<lineSegment> lines;
	lineSegment line; line.p1 = cor1; line.p2 = cor2; lines.push_back(line);
	line.p1 = cor2; line.p2 = cor3; lines.push_back(line);
	line.p1 = cor3; line.p2 = cor4; lines.push_back(line);
	line.p1 = cor4; line.p2 = cor1; lines.push_back(line);
	return lines;
}

// Issue the navigational goal
bool findFarthestFreeSpacePoint::issueNavigationalGoal(geometry_msgs::Pose navPose)
{
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		log("Waiting for move_base server");
	}
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id= MAP_FRAME;
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose = navPose;

	bool goalsuccess = false;
	log("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState()  == actionlib::SimpleClientGoalState::SUCCEEDED)
	 goalsuccess = true;
	else
	 goalsuccess = false;

	return goalsuccess;
}

// Read and copy the entire read marker array
void findFarthestFreeSpacePoint::markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& marArrMsg)
{
	recvdMarkerArray.markers.clear();
	for(unsigned int ctr = 0; ctr < marArrMsg->markers.size(); ctr++)
	{
		// Copy the entire array
		recvdMarkerArray.markers.push_back(marArrMsg->markers[ctr]);
	}
}


// Accept Dialogue Request And Give Response
void findFarthestFreeSpacePoint::processDialogueRequest(const cast::cdl::WorkingMemoryChange& _wmc)
{
	log("GOT THE REQUEST FROM DIALOGUE");
	eu::nifti::context::slice::WhatIsForwardPtr whatIsForward;
	try
        {
            whatIsForward = getMemoryEntry<eu::nifti::context::slice::WhatIsForward> (_wmc.address);
        }
    catch(char *e)
        {
            log("Could not read Dialogue response for WhatIsForward: %s",e);
        }
    catch(std::exception e)
	{
	    log("Could not read Dialogue response for WhatIsForward: %s",e.what());
	}

	if(!whatIsForward->request)
	{
		log("Ignored request from dialogue 'WhatIsForward' since request flag is false");
	}

	geometry_msgs::Pose farthestPose = findFarthestPoseInDirectionOfRobot();
	log("CALCULATED THE FARTHEST POSE IN DIRECTION OF ROBOT %6.4f %6.4f %6.4f", farthestPose.position.x, farthestPose.position.y, farthestPose.position.z);

	if(giveGoalDirectlyToNavStack==false)
	{
		// Give response to dialogue
		try{
			eu::nifti::context::slice::ThisIsForwardPtr forwardPosition = new eu::nifti::context::slice::ThisIsForward(farthestPose.position.x, farthestPose.position.y);
			overwriteWorkingMemory(_wmc.address, forwardPosition);
			log("Overwrote forward position to working memory");
		}
	   	catch(char *e)
		{
		    log("Caught exception while writing to ThisIsForward %s",e);
		}
		catch(std::exception e)
		{
			log("Caught exception while writing to ThisIsForward %s",e.what());
		}
		log("SENT THE RESPONSE TO DIALOGUE");
	}
	else
	{
		// Send goal to navigation stack
		log("Sending goal to farthest free pose");
		printGeoPoint(farthestPose.position);
		log("With quaternion angle x %3.2f y %3.2f z %3.2f w %3.2f",farthestPose.orientation.x, farthestPose.orientation.y,farthestPose.orientation.z,farthestPose.orientation.w); 
		bool goalReached = issueNavigationalGoal(farthestPose);
		if(goalReached==false)
		{
			log("We reached the farthest free pose");
		}
		else
		{
			log("We plotted a goal to farthest free pose, but did not reach it");
		}
	}
	return;
}

// We subscribe to the marker array and store elements that designate free space. Return if free space has been found.
void findFarthestFreeSpacePoint::subscribeToAndStoreMarkerArray()
{
	// 'cubes' are the type of marker msgs used to designate free space. So we check if the marker array contains any free space.
	bool doesMarkerArrayMsgContainAddedMarkerCubes = false; 
	ros::Subscriber sub = n.subscribe(MARKER_ARRAY_TOPIC, 5, &findFarthestFreeSpacePoint::markerArrayCallback, this);
	while(n.ok())
	{
		ros::spinOnce();
		ros::Duration(1.0).sleep();
		doesMarkerArrayMsgContainAddedMarkerCubes = checkForAddedMarkerCubesInMarkerArray();
	}
}

void findFarthestFreeSpacePoint::start()
{
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	ros::init(argc, argv, "findFarthestFreeSpacePoint");

	//Register the change filter for context
	addChangeFilter(cast::createGlobalTypeFilter<eu::nifti::context::slice::WhatIsForward>(cast::cdl::ADD),new cast::MemberFunctionChangeReceiver<findFarthestFreeSpacePoint>(this,&findFarthestFreeSpacePoint::processDialogueRequest));

	
}

void findFarthestFreeSpacePoint::configure(const std::map<std::string, std::string> &config)
{
	// Upon receiving the request from dialogue, do we give the response to dialogue, or instead send goal to nav stack
	std::map<std::string, std::string>::const_iterator it = config.find("--giveGoalDirectlyToNavStack");
	if (it!=config.end())
	{
		if((it->second).compare("true")==0)
		{
			giveGoalDirectlyToNavStack = true;
		}
		else
		{
			giveGoalDirectlyToNavStack = false;
		}
	}	
	else
	{
		giveGoalDirectlyToNavStack = false;
	}
}


void findFarthestFreeSpacePoint::runComponent()
{
	// Subscribe to the marker array continuously. 
        subscribeToAndStoreMarkerArray();

	/*if(freeSpaceFound == false)
	{
		ROS_INFO("We did not find any free space in front of the robot, while looking at the topological graph. So we did not send a navigation goal");
	return 0;
	}
	
	// Within the free space, find the farthest pose in direction of the robot
	geometry_msgs::Pose farthestPose = navGoalIssuer.findFarthestPoseInDirectionOfRobot();*/
		
	
	return;
}

extern "C"
{

    cast::CASTComponentPtr newComponent()
    {
        return new findFarthestFreeSpacePoint();
    }
}

// Test successful intersection of ray and line segment
bool findFarthestFreeSpacePoint::testSuccessfulIntersectionOfRayAndLineSegment(btVector3 pos, btVector3 dir, findFarthestFreeSpacePoint::lineSegment line)
{
	bool success = false;
	furthestPoseAndDistanceToIt furPoseDist;
	furPoseDist = findIntersectionOfRayAndLineSegment(pos, dir, line);
	if(furPoseDist.distanceToFurthest > 0)
	{
		success = true;
	}
	return success;
}

#endif

