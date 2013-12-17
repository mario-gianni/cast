// AUTHOR : SHANKER KESHAVDAS , DFKI SAARBRUECKEN

#ifndef FM_Find_Forward_CPP_
#define FM_Find_Forward_CPP_

#include <findForwardUsingPointClouds.hpp>

findForwardUsingPointClouds::findForwardUsingPointClouds() : n("~")
{
	filePath.assign("stacks/nifti_mapping/functional_understanding_of_unstructured_environments/working_files");
	fileNamePrefix.assign("currentscene");
	fileName.assign(fileNamePrefix);
	fileName.append("1.pcd");
	pointCloudTopic.assign("/dynamic_point_cloud");
	xClipPointCloud = 5.0;
	yClipPointCloud = 5.0;
	zClipPointCloud = 1.5;
	searchSpaceX = 5;
	searchSpaceY = 5;
	searchSpaceZ = 0;
	gridCubeEdge = 0.2;
	xGridCubeRobot = 3;
	yGridCubeRobot = 2;
	zGridCubeRobot = 2;

	poseComputedMutex.lock();
	poseComputedSuccessfully = false;
	poseComputedMutex.unlock();

	
	reader = boost::shared_ptr<PointCloudReader>(new PointCloudReader(TOPIC_FOR_POINTCLOUD2_MSGS));
	visualizer = boost::shared_ptr<PointCloudVisualizer>(new PointCloudVisualizer());
	clipper = boost::shared_ptr<PointCloudClipper>(new PointCloudClipper());
	featComp = boost::shared_ptr<PointCloudFeatureComputer>(new PointCloudFeatureComputer());
	clusterer = boost::shared_ptr<PointCloudClusterer>(new PointCloudClusterer());
}

findForwardUsingPointClouds::~findForwardUsingPointClouds()
{

}


// Accept Dialogue Request And Give Response
void findForwardUsingPointClouds::processDialogueRequest(const cast::cdl::WorkingMemoryChange& _wmc)
{
	log("GOT REQUEST FROM DIALOGUE TO MOVE FORWARD");
	eu::nifti::context::slice::WhatIsForwardPtr whatIsForward;
	try
        {
            whatIsForward = getMemoryEntry<eu::nifti::context::slice::WhatIsForward> (_wmc.address);
        }
    catch(char *e)
        {
            log("Could not read Dialogue response for WhatIsForward: %s",e);
        }
    catch(std::exception &e)
	{
	    log("Could not read Dialogue response for WhatIsForward: %s",e.what());
	}

	if(!whatIsForward->request)
	{
		log("Ignored request from dialogue 'WhatIsForward' since request flag is false");
		return;
	}

	geometry_msgs::Pose farthestPose = analyzePointCloudsFindForward();
	log("FINISHED ANALYZING WHAT MOVE FORWARD IS");
	
	poseComputedMutex.lock();	
	bool poseSuccess = poseComputedSuccessfully;
	poseComputedMutex.unlock();

	if(poseSuccess)
	{
		log("Sending the position %6.4f , %6.4f to the Dialogue component",farthestPose.position.x, farthestPose.position.y);

		// Give response to dialogue
		try{
			eu::nifti::context::slice::ThisIsForwardPtr forwardPosition = new eu::nifti::context::slice::ThisIsForward(farthestPose.position.x, farthestPose.position.y);
			overwriteWorkingMemory(_wmc.address, forwardPosition);
			log("GAVE RESPONSE TO DIALOGUE");
		}
	   	catch(char *e)
		{
			log("Caught exception while writing to ThisIsForward %s",e);
		}
		catch(std::exception e)
		{
			log("Caught exception while writing to ThisIsForward %s",e.what());
		}
	}
	else
	{		
		log("DID NOT CALCULATE POSE SUCCESSFULLY, SENDING NO RESPONSE TO DIALOGUE");
	}
	return;
}

geometry_msgs::Pose findForwardUsingPointClouds::analyzePointCloudsFindForward()
{
	geometry_msgs::Pose finalPose = constructPose(0,0,0,0,0,0,1);
	poseComputedMutex.lock();
	poseComputedSuccessfully = false;
	poseComputedMutex.unlock();

	if(!reader->readFromTopicAndSave(filePath.c_str(),fileNamePrefix.c_str(),1,2,30))
	{
		ROS_ERROR("findForwardUsingPointClouds::analyzePointCloudsFindForward : Could not read from given point cloud topic %s",pointCloudTopic.c_str());
		return finalPose;
	}

	try{
		cloud = reader->readFromPCDFile(filePath.c_str(),fileName.c_str());	 // Read from file
		cloud = clipper->rectangularClipInYDirection(cloud,yClipPointCloud);  // Clip in all directions
		cloud = clipper->roughClipFurthestDistance(cloud,xClipPointCloud);
		cloud = clipper->roughClipHeight(cloud,zClipPointCloud);
		cloud = clipper->roughFloorClip(cloud);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudColorNormal(new pcl::PointCloud<pcl::PointXYZRGBNormal>); 
		cloudColorNormal = featComp->computeNormals(cloud);
		pcClust = clusterer->EuclideanClustering(cloudColorNormal, 0.02, 0.8, 0.01); // Perform clustering
		boost::shared_ptr<AxisOrientedNearnessMatrix> axGrid(new AxisOrientedNearnessMatrix(xClipPointCloud,yClipPointCloud,zClipPointCloud,gridCubeEdge));
		boost::shared_ptr<RobotCubeStructure> robotCube(new RobotCubeStructure(xGridCubeRobot,yGridCubeRobot,zGridCubeRobot));
		axGrid->calculateRobotMovementFreeSpaceAndOccupiedCubes(pcClust,robotCube);
		axGrid->givenRobotComputeNearnessCubes(robotCube);
		axGrid->computeNearnessCosts();
		boost::shared_ptr<OverallCostComputer> costComp(new OverallCostComputer(costPath,costDev,costDist));
		costComp->inputStartPose(constructPose(0,0,0,0,0,0,1));
		costComp->generatePoseMatrixGivenSearchSpace(searchSpaceX,searchSpaceY,searchSpaceZ);
		costComp->computeCostsForPoseMatrix(axGrid,robotCube);
		OverallCostComputer::PoseCosts minPoseCost = costComp->getMinCostFromPoseMatrix(true);
		finalPose = minPoseCost.endPose;

		// Transform to right frame ( MESSY CODE ) : TODO : Clean and consolidate as components
 		log("GOT FINAL POSE FROM CALCULATION IN FRAME BASE_LINK IT IS %6.4f %6.4f %6.4f", finalPose.position.x, finalPose.position.y, finalPose.position.z); 
		geometry_msgs::PoseStamped preTransformPoseStamped;
		preTransformPoseStamped.pose = finalPose;
		preTransformPoseStamped.header.stamp = ros::Time::now();
		preTransformPoseStamped.header.frame_id = "/base_link";

		std::string targetFrame = "/map";
		geometry_msgs::PoseStamped finalPoseStamped;
		tfList.transformPose(targetFrame, preTransformPoseStamped, finalPoseStamped);
		finalPose = finalPoseStamped.pose;
		log("TRANSFORMED TO FRAME MAP %6.4f %6.4f %6.4f",finalPose.position.x,finalPose.position.y,finalPose.position.z); 		

		poseComputedMutex.lock();
		poseComputedSuccessfully = true;
		poseComputedMutex.unlock();
	}
	catch(FMGeneralException &e)
	{
		ROS_ERROR("findForwardUsingPointClouds::analyzePointCloudsFindForward : Caught exception %s",e.getMessage());	
	}

	return finalPose;
}

void findForwardUsingPointClouds::start()
{
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	ros::init(argc, argv, "findForwardUsingPointClouds");


	//Register the change filter for context
	addChangeFilter(cast::createGlobalTypeFilter<eu::nifti::context::slice::WhatIsForward>(cast::cdl::ADD),new cast::MemberFunctionChangeReceiver<findForwardUsingPointClouds>(this,&findForwardUsingPointClouds::processDialogueRequest));

}

void findForwardUsingPointClouds::configure(const std::map<std::string, std::string>& config)
{
     std::map<std::string, std::string>::const_iterator it = config.find("--costPath");
     if (it!=config.end())
     {
        char *_chrcopy = new char [it->second.size()+1];
        strcpy (_chrcopy, it->second.c_str());
        costPath = std::atof(_chrcopy);
     }
     else
     {
	log("WARNING: Did not receive parameters for Path costs, taking default parameters");
         costPath = DEFAULT_PATH_COST;
     }
     it = config.find("--costDev");
     if (it!=config.end())
     {
        char *_chrcopy = new char [it->second.size()+1];
        strcpy (_chrcopy, it->second.c_str());
        costDev = std::atof(_chrcopy);
     }
     else
     {
	log("WARNING: Did not receive parameters for Deviation costs, taking default parameters");
         costDev = DEFAULT_DEVIATION_COST;
     }
     it = config.find("--costDist");
     if (it!=config.end())
     {
        char *_chrcopy = new char [it->second.size()+1];
        strcpy (_chrcopy, it->second.c_str());
        costDist = std::atof(_chrcopy);
     }
     else
     {
	log("WARNING: Did not receive parameters for Distance costs, taking default parameters");
         costDist = DEFAULT_DISTANCE_COST;
     }
}

void findForwardUsingPointClouds::runComponent()
{
	
}

extern "C"
{

    cast::CASTComponentPtr newComponent()
    {
        return new findForwardUsingPointClouds();
    }
}

#endif
