/*
 * findForwardUsingPointCloudsIterative.cpp
 *
 *  Created on: Sep 19, 2013
 *      Author: shanker
 */

#ifndef FINDFORWARDUSINGPOINTCLOUDSITERATIVE_CPP_
#define FINDFORWARDUSINGPOINTCLOUDSITERATIVE_CPP_



#include <findForwardUsingPointCloudsIterative.hpp>

findForwardUsingPointCloudsIterative::findForwardUsingPointCloudsIterative() : n("~")
{
	filePath.assign("stacks/nifti_mapping/functional_understanding_of_unstructured_environments/working_files");
	fileNamePrefix.assign("currentscene");
	fileName.assign(fileNamePrefix);
	fileName.append("1.pcd");

    xClipPointCloud = 5.0;
	yClipPointCloud = 5.0;
	zClipPointCloud = 1.5;

	gridCubeEdge = 0.2;
	xGridCubeRobot = 3;
	yGridCubeRobot = 2;
	zGridCubeRobot = 2;
	
	searchSpaceY = 5.0;
	searchSpaceZ = 1.5;
	searchSpaceX = 5.0;

	cumulativeSumOfOccupiedCubes = 0;
	iterationFlag = false;

	reader = boost::shared_ptr<PointCloudReader>(new PointCloudReader(TOPIC_FOR_POINTCLOUD2_MSGS));
	visualizer = boost::shared_ptr<PointCloudVisualizer>(new PointCloudVisualizer());
	clipper = boost::shared_ptr<PointCloudClipper>(new PointCloudClipper());
	featComp = boost::shared_ptr<PointCloudFeatureComputer>(new PointCloudFeatureComputer());
	clusterer = boost::shared_ptr<PointCloudClusterer>(new PointCloudClusterer());
	
	gotFirstMessage = false;
}

findForwardUsingPointCloudsIterative::~findForwardUsingPointCloudsIterative()
{

}


// Processes request from Planning regarding forward
void findForwardUsingPointCloudsIterative::processForwardRequest(const cast::cdl::WorkingMemoryChange& _wmc)
{
	log("Got forward request");
	gotFirstMessage = true;
	eu::nifti::context::forwardRequestPtr fReq;
	eu::nifti::context::rosPosePtr emptyPose = new eu::nifti::context::rosPose(0,0,0,0,0,0,1);
	eu::nifti::context::rosPoses emptyPoses;
	eu::nifti::context::forwardResponsePtr fRes;
	try
	{
		fReq = getMemoryEntry<eu::nifti::context::forwardRequest> (_wmc.address);
	}
    catch(char *e)
	{
		log("Could not read forward request: %s",e);
	}
    catch(std::exception &e)
	{
	    log("Could not read forward request: %s",e.what());
	}
	
	log("Message is copied");

    // If request flag is false, ignore request
	if(fReq->requestFlag == false)
	{
		log("Replying an invalid response since request flag is false");
		try{
			fRes = new eu::nifti::context::forwardResponse(fReq->requestID,emptyPose,emptyPoses,false,false);
			overwriteWorkingMemory(_wmc.address, fRes);
			cumulativeSumOfOccupiedCubes = 0;
			iterationFlag = false;
		}
	   	catch(char *e)
		{
			log("Caught exception while writing response %s",e);
		}
		catch(std::exception &e)
		{
			log("Caught exception while writing response %s",e.what());
		}
		return;
	}

	if(iterationFlag == true)
	{
		// If iteration flag is true, an iteration was reccommended in the last cycle. We reset the iteration flag for this cycle
		iterationFlag = false;
	}
	else
	{
		// If iteration flag is false, an iteration was not reccommended in the last cycle. We reset the iteration flag and the cumulative sum of occupied cubes.
		iterationFlag = false;
		cumulativeSumOfOccupiedCubes = 0;
	}

    log("Start analyzing");
	findForwardUsingPointCloudsIterative::forwardResponseMsg frMsg = analyzePointCloudsFindForward();
	log("Done analyzing");

	// Copy this message to the response
	eu::nifti::context::rosPosePtr finalPose = new eu::nifti::context::rosPose(frMsg.targetPose.position.x,frMsg.targetPose.position.y,frMsg.targetPose.position.z,frMsg.targetPose.orientation.x,frMsg.targetPose.orientation.y,frMsg.targetPose.orientation.z,frMsg.targetPose.orientation.w);
	eu::nifti::context::rosPoses interPoses;
	for(unsigned int ctr = 0; ctr < frMsg.intermediatePoses.size(); ctr++)
	{
		interPoses.push_back(new eu::nifti::context::rosPose(frMsg.intermediatePoses[ctr].position.x,frMsg.intermediatePoses[ctr].position.y,frMsg.intermediatePoses[ctr].position.z,frMsg.intermediatePoses[ctr].orientation.x,frMsg.intermediatePoses[ctr].orientation.y,frMsg.intermediatePoses[ctr].orientation.z,frMsg.intermediatePoses[ctr].orientation.w));
	}
	fRes = new eu::nifti::context::forwardResponse(fReq->requestID,finalPose,interPoses,frMsg.responseFlag,frMsg.repeatFlag);

	// Send the response
	try{
			overwriteWorkingMemory(_wmc.address, fRes);
			log("Overwritten working memory");
		}
	catch(char *e)
	{
		log("Caught exception while writing response %s",e);
	}
	catch(std::exception &e)
	{
		log("Caught exception while writing response %s",e.what());
	}

	return;
}

findForwardUsingPointCloudsIterative::forwardResponseMsg findForwardUsingPointCloudsIterative::analyzePointCloudsFindForward()
{
	findForwardUsingPointCloudsIterative::forwardResponseMsg frMsg;
	frMsg.targetPose = constructPose(0,0,0,0,0,0,1);
	frMsg.repeatFlag = false;
	frMsg.responseFlag = false;

	try{
		cloud = reader->readSingleMsgFromTopic();
	}
	catch(FMGeneralException &e)
	{
		log("findForwardUsingPointCloudsIterative::analyzePointCloudsFindForward : Received a timeout while reading from the topic. Returning an invalid message with negative response flag");
		return frMsg;
	}
	
	log("Read message");

	try{
	    log("Starting the clipping");
		cloud = clipper->rectangularClipInYDirection(cloud,yClipPointCloud);  // Clip in all directions
		log("Done the y clip");
		cloud = clipper->roughClipFurthestDistance(cloud,xClipPointCloud);
		log("done the x clip");
		cloud = clipper->roughClipHeight(cloud,zClipPointCloud);
		log("done the z clip");
		cloud = clipper->roughFloorClip(cloud);
		log("clipped the point cloud");
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudColorNormal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		cloudColorNormal = featComp->computeNormals(cloud);
		pcClust = clusterer->EuclideanClustering(cloudColorNormal, 0.05, 1, 0.005); // Perform clustering
		log("performed the clustering");
		boost::shared_ptr<AxisOrientedNearnessMatrix> axGrid(new AxisOrientedNearnessMatrix(xClipPointCloud,yClipPointCloud,zClipPointCloud,gridCubeEdge));
		boost::shared_ptr<RobotCubeStructure> robotCube(new RobotCubeStructure(xGridCubeRobot,yGridCubeRobot,zGridCubeRobot));
		axGrid->calculateRobotMovementFreeSpaceAndOccupiedCubes(pcClust,robotCube);
		log("computed the freespace and occupied cubes");
		axGrid->givenRobotComputeNearnessCubes(robotCube);
		log("computed nearness cubes");
		axGrid->computeNearnessCosts();
		log("computed nearness costs");
		cumulativeSumOfOccupiedCubes += axGrid->getNumberOfOccupiedSpaceCubes();
        log("made cumulative sum");
		boost::shared_ptr<OverallCostComputer> costComp(new OverallCostComputer(costPath,costDev,costDist));
		costComp->inputStartPose(constructPose(0,0,0,0,0,0,1));
		costComp->generatePoseMatrixGivenSearchSpace(searchSpaceX,searchSpaceY,gridCubeEdge);
		log("generated search space");
		costComp->computeCostsForPoseMatrix(axGrid,robotCube);
		log("computed costs for pose matrix");
		OverallCostComputer::PoseCosts minPoseCost = costComp->getMinCostFromPoseMatrix(true);
		log("Got the minimum cost from the pose matrix");
		frMsg.targetPose = minPoseCost.endPose;
		frMsg.intermediatePoses = costComp->computeLocalMinimaBetweenStartAndMinimumPose(true);
		log("Got the intermediate poses");
		if((costComp->isTargetPoseOnXLimit()==true)&&(cumulativeSumOfOccupiedCubes <= DEFAULT_CUMULATIVE_OBSTACLE_LIMIT))
		{
			frMsg.repeatFlag = true;
			iterationFlag = true;
		}
		else
		{
			frMsg.repeatFlag = false;
			iterationFlag = false;
		}
		frMsg.responseFlag = true;

		/*// Transform the target and intermediate poses to frame /map
		log("TARGET POSE FROM CALCULATION IN FRAME BASE_LINK  IS %6.4f %6.4f %6.4f", frMsg.targetPose.position.x, frMsg.targetPose.position.y, frMsg.targetPose.position.z);
		geometry_msgs::PoseStamped preTransformPoseStamped;
		preTransformPoseStamped.pose = frMsg.targetPose;
		preTransformPoseStamped.header.stamp = ros::Time::now();
		preTransformPoseStamped.header.frame_id = "/base_link";
		log("base frame");

		std::string targetFrame = "/map";
		geometry_msgs::PoseStamped finalPoseStamped;
		log("target frame defined");
		
		tfList.waitForTransform(targetFrame, preTransformPoseStamped.header.frame_id, ros::Time::now(), ros::Duration(15));
        try{
            tfList.transformPose(targetFrame, preTransformPoseStamped, finalPoseStamped);
		    frMsg.targetPose = finalPoseStamped.pose;
		}
		catch(tf::TransformException &e)
	    {
	        std::cout<<"Caught tf exception : "<<e.what();
	        log("Transform error, sending false flag");
		    frMsg.targetPose = constructPose(0,0,0,0,0,0,1);
		    frMsg.responseFlag = false;
		    return frMsg;
		}
		log("TRANSFORMED TO FRAME MAP %6.4f %6.4f %6.4f",frMsg.targetPose.position.x,frMsg.targetPose.position.y,frMsg.targetPose.position.z);

		for(unsigned int i = 0; i < frMsg.intermediatePoses.size(); i++)
		{
			geometry_msgs::PoseStamped preTransformPoseStamped;
			preTransformPoseStamped.pose = frMsg.intermediatePoses[i];
			std::cout<<"\n Intermediate point "<<i<<" position x "<<preTransformPoseStamped.pose.position.x<<" position y "<<preTransformPoseStamped.pose.position.y;
			preTransformPoseStamped.header.stamp = ros::Time::now();
			preTransformPoseStamped.header.frame_id = "/base_link";

			std::string targetFrame = "/map";
			geometry_msgs::PoseStamped finalPoseStamped;
			bool transformSuccess = tfList.waitForTransform(targetFrame, preTransformPoseStamped.header.frame_id, ros::Time::now(), ros::Duration(15));
			try
		    {
		        tfList.transformPose(targetFrame, preTransformPoseStamped, finalPoseStamped);
		        frMsg.intermediatePoses[i] = finalPoseStamped.pose;
		        log("transform success");
	        }
	        catch(tf::TransformException &e)
	        {
	            std::cout<<"Caught tf exception"<<e.what();
	            log("Transform error, sending false flag");
		        frMsg.intermediatePoses[i] = constructPose(0,0,0,0,0,0,1);
		        frMsg.responseFlag = false;
		        return frMsg;
            }
		 }*/
	}
	catch(FMGeneralException &e)
	{
		ROS_ERROR("findForwardUsingPointClouds::analyzePointCloudsFindForward : Caught exception %s , hence setting the response flag to false",e.getMessage());
		frMsg.responseFlag = false;
	}

	return frMsg;
}

void findForwardUsingPointCloudsIterative::start()
{
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	ros::init(argc, argv, "findForwardUsingPointCloudsIterative");

	//Register the change filter for context
	//addChangeFilter(cast::createGlobalTypeFilter<eu::nifti::context::forwardRequest>(cast::cdl::ADD),new cast::MemberFunctionChangeReceiver<findForwardUsingPointCloudsIterative>(this,&findForwardUsingPointCloudsIterative::processForwardRequest));

	std::string src = "NavigationComponent"; //name of the component
    std::string id;
    std::string sa;
    addChangeFilter(cast::createChangeFilter<eu::nifti::context::forwardRequest>(cast::cdl::ADD,src,id,sa,cast::cdl::ALLSA),new cast::MemberFunctionChangeReceiver<findForwardUsingPointCloudsIterative>(this,&findForwardUsingPointCloudsIterative::processForwardRequest)); 
    


}

void findForwardUsingPointCloudsIterative::configure(const std::map<std::string, std::string>& config)
{
     std::map<std::string, std::string>::const_iterator it = config.find("--linFactPath");
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
     it = config.find("--linFactDev");
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
     it = config.find("--linFactDist");
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
     it = config.find("--expFactDist");
     if (it!=config.end())
     {
        char *_chrcopy = new char [it->second.size()+1];
        strcpy (_chrcopy, it->second.c_str());
        expDistCost = std::atof(_chrcopy);
     }
     else
     {
	log("WARNING: Did not receive parameters for exponential distance costs, taking default parameters");
         expDistCost = DEFAULT_EXP_DIST_COST;
     }
     it = config.find("--expFactDev");
     if (it!=config.end())
     {
        char *_chrcopy = new char [it->second.size()+1];
        strcpy (_chrcopy, it->second.c_str());
        expDevCost = std::atof(_chrcopy);
     }
     else
     {
	log("WARNING: Did not receive parameters for exponential deviation costs, taking default parameters");
         expDevCost = DEFAULT_EXP_DEV_COST;
     }
     it = config.find("--cloudClipX");
     if (it!=config.end())
     {
        char *_chrcopy = new char [it->second.size()+1];
        strcpy (_chrcopy, it->second.c_str());
        xClipPointCloud = std::atof(_chrcopy);
     }
     else
     {
	log("WARNING: Did not receive parameters for cloud clip in X Direction, taking default parameters");
         xClipPointCloud = DEFAULT_CLOUD_CLIP_X;
     }
     it = config.find("--topicName");
     if (it!=config.end())
     {
    	 pointCloudTopic = it->second;
     }
     else
     {
	log("WARNING: Did not receive parameters for point cloud topic, taking default parameters");
         pointCloudTopic = TOPIC_FOR_POINTCLOUD2_MSGS;
     }
}

void findForwardUsingPointCloudsIterative::runComponent()
{
       ros::Time start = ros::Time::now();
    while(ros::ok())
    {
        if((ros::Time::now()).toSec() - start.toSec() > 10.00 && gotFirstMessage==true)
        {
            log("Waiting for message from planning");
            start = ros::Time::now();
        }
    }
}

extern "C"
{

    cast::CASTComponentPtr newComponent()
    {
        return new findForwardUsingPointCloudsIterative();
    }
}



#endif /* FINDFORWARDUSINGPOINTCLOUDSITERATIVE_CPP_ */
