// AUTHOR : SHANKER KESHAVDAS , DFKI SAARBRUECKEN

#ifndef FM_Find_Forward_HPP_
#define FM_Find_Forward_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <context.hpp>
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
#include <functional_mapping_helper/Exceptions/FMGeneralException.hpp>

/* In this file, we find what forward means through point cloud processing */

#define PATH_TO_WORKING_FILES "subarchitectures/mapping.sa/src/c++/workingFiles"
#define FILE_TO_VISUALIZE "Scene11.pcd"
#define TOPIC_FOR_POINTCLOUD2_MSGS "/dynamic_point_cloud"
#define DEFAULT_PATH_COST 2.24
#define DEFAULT_DISTANCE_COST 0.64
#define DEFAULT_DEVIATION_COST 0.64

class findForwardUsingPointClouds : public cast::ManagedComponent
{

	ros::NodeHandle n;
	std::string filePath;
	std::string fileNamePrefix;
	std::string fileName;
	std::string pointCloudTopic;	
	float xClipPointCloud, yClipPointCloud, zClipPointCloud, gridCubeEdge, xGridCubeRobot, yGridCubeRobot, zGridCubeRobot, costPath, costDev, costDist;
	float searchSpaceX, searchSpaceY, searchSpaceZ;

	boost::shared_ptr<PointCloudReader> reader;
	boost::shared_ptr<PointCloudVisualizer> visualizer;
	boost::shared_ptr<PointCloudClipper> clipper;
	boost::shared_ptr<PointCloudFeatureComputer> featComp;
	boost::shared_ptr<PointCloudClusterer> clusterer;
	boost::shared_ptr<PointCloudAndCluster> pcClust;
	boost::shared_ptr<RobotCubeStructure> robotCube;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	bool poseComputedSuccessfully;
	boost::mutex poseComputedMutex;

	tf::TransformListener tfList;

	void processDialogueRequest(const cast::cdl::WorkingMemoryChange& _wmc);
	geometry_msgs::Pose analyzePointCloudsFindForward();

public:
	findForwardUsingPointClouds();
	~findForwardUsingPointClouds();

protected:
	virtual void start();
	virtual void runComponent();
	virtual void configure(const std::map<std::string, std::string>&);
};

#endif
