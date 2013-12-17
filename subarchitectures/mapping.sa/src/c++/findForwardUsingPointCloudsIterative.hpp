/*
 * findForwardUsingPointCloudsIterative.hpp
 *
 *  Created on: Sep 19, 2013
 *      Author: shanker
 */

#ifndef FINDFORWARDUSINGPOINTCLOUDSITERATIVE_HPP_
#define FINDFORWARDUSINGPOINTCLOUDSITERATIVE_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <forwardRequest.hpp>
#include <forwardResponse.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/PointCloudReader.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/PointCloudVisualizer.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/PointCloudClipper.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/PointCloudFeatureComputer.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/PointCloudClusterer.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/PointCloudAndCluster.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/RobotCubeStructure.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/AxisOrientedNearnessMatrix.hpp>
#include <functional_understanding_of_unstructured_environments/PointCloudProcessing/OverallCostComputer.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <functional_mapping_helper/Exceptions/FMGeneralException.hpp>

/* In this file, we find what forward means through point cloud processing */

#define PATH_TO_WORKING_FILES "subarchitectures/mapping.sa/src/c++/workingFiles"
#define FILE_TO_VISUALIZE "Scene11.pcd"
#define TOPIC_FOR_POINTCLOUD2_MSGS "/dynamic_point_cloud"
#define DEFAULT_PATH_COST 2.24
#define DEFAULT_DISTANCE_COST 0.64
#define DEFAULT_DEVIATION_COST 0.64
#define DEFAULT_EXP_DIST_COST 0.8
#define DEFAULT_EXP_DEV_COST 0.4
#define DEFAULT_CLOUD_CLIP_X 5.0
#define DEFAULT_CUMULATIVE_OBSTACLE_LIMIT 100 // When the cumulative cubes seen in an iterative loop reaches a 100, we reset the loop

class findForwardUsingPointCloudsIterative : public cast::ManagedComponent
{
	ros::NodeHandle n;
	std::string filePath;
	std::string fileNamePrefix;
	std::string fileName;
	std::string pointCloudTopic;
	float xClipPointCloud, yClipPointCloud, zClipPointCloud, gridCubeEdge, xGridCubeRobot, yGridCubeRobot, zGridCubeRobot, costPath, costDev, costDist;
	float expDevCost, expDistCost;
	float searchSpaceX, searchSpaceY, searchSpaceZ;

	int cumulativeSumOfOccupiedCubes;
	bool iterationFlag;
	bool gotFirstMessage;

	boost::shared_ptr<PointCloudReader> reader;
	boost::shared_ptr<PointCloudVisualizer> visualizer;
	boost::shared_ptr<PointCloudClipper> clipper;
	boost::shared_ptr<PointCloudFeatureComputer> featComp;
	boost::shared_ptr<PointCloudClusterer> clusterer;
	boost::shared_ptr<PointCloudAndCluster> pcClust;
	boost::shared_ptr<RobotCubeStructure> robotCube;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	tf::TransformListener tfList;

	struct forwardResponseMsg
	{
		int requestID;
		geometry_msgs::Pose targetPose;
		std::vector<geometry_msgs::Pose> intermediatePoses;
		bool repeatFlag;
		bool responseFlag;
	};

	void processForwardRequest(const cast::cdl::WorkingMemoryChange& _wmc);
	forwardResponseMsg analyzePointCloudsFindForward();

public:
	findForwardUsingPointCloudsIterative();
	~findForwardUsingPointCloudsIterative();

protected:
	virtual void start();
	virtual void runComponent();
	virtual void configure(const std::map<std::string, std::string>&);
};


#endif /* FINDFORWARDUSINGPOINTCLOUDSITERATIVE_HPP_ */
