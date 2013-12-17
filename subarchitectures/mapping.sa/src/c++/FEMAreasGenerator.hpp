// AUTHOR : SHANKER KESHAVDAS , DFKI SAARBRUECKEN

#ifndef FEM_Areas_Generator_HPP_
#define FEM_Areas_Generator_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include "autogen/mapping.hpp"
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <eu_nifti_env_msg_ros/ElementOfInterestMessage.h>
#include <eu/nifti/ConverterUtil_Mapping.hpp>
#include <geometry_msgs/Vector3.h>
#include <kdl/frames.hpp>
#include "autogen/eu/nifti/env/CarObjectOfInterest.hpp"
#include <planning_roma.hpp>
#include <eu_nifti_env/CarObjectOfInterest.h>
#include <eu_nifti_env/LocationOfInterest.h>
#include <eu_nifti_env/AreaOfInterest.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include "eu_nifti_env_msg_ros/Util.h"
#include <fstream>
#include <pnpoly.hpp>



class FEMAreasGenerator : public cast::ManagedComponent {
public:

    // For control by planning
    bool planningControl; // Specifies whether mapping.sa should be controlled by planning_ROMA or not
    eu::nifti::Planning::slice::FunctionalMappingActionPtr planningControlAction;  

    eu_nifti_env::ObjectOfInterest lastOOIReceived;
    int8_t lastActionReceived;
    bool firstMessageReceived;
    std::string _hfcservername;
    eu::nifti::mapping::HFCInterfacePrx _hfcserver;
    std::vector<eu_nifti_env_msg_ros::ElementOfInterestMessage> eoiDrawCarVisibilityAreasList;
    typedef std::map<int, std::string> mapUUIDToWMA; // Mapping UUIDs to Working Memory Addresses
    mapUUIDToWMA mapUUIDToWMAForDetectedObjects;
    std::map<std::string, cast::MemberFunctionChangeReceiver<FEMAreasGenerator>*> tempMap;
    std::vector<Ice::Int> processingUUIDs;

    int tempWindowNumber;
    
    void onElementOfInterestMessageReceived(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr &msg);
    bool checkRepetitionOfOOI(const eu_nifti_env::ObjectOfInterest*);
    bool isUUIDBeingProcessed(Ice::Int);
    void removeUUIDFromProcessingList(Ice::Int);
    void waitTillUUIDIsFree(Ice::Int);
    void addUUIDToProcessingList(Ice::Int);
    void addUUIDToProcessingListWhenFree(Ice::Int);
    float queryOntologyFloat(char*);
    float convertXsdFloatToFloat(std::string &_str);
    geometry_msgs::Point constructPoint(float x, float y, float z);
    geometry_msgs::Vector3 constructVector3(float x, float y, float z);
    geometry_msgs::Point rotateAndTranslatePoint(geometry_msgs::Point, geometry_msgs::Vector3, float, float, float);
    geometry_msgs::Point translateAndRotatePoint(geometry_msgs::Point, geometry_msgs::Vector3, float, float, float);
    eu_nifti_env::Polygon rotateAndTranslateROSPolygon(eu_nifti_env::Polygon, geometry_msgs::Vector3, float, float, float);
    eu::nifti::env::Polygon rotateAndTranslateCASTPolygon(eu::nifti::env::Polygon, geometry_msgs::Vector3, float, float, float);
    float computeDistanceBetweenPoints(geometry_msgs::Point, geometry_msgs::Point);
    eu_nifti_env::Polygon computeWindowPointsFromDimensions(eu::nifti::env::Window);
    eu_nifti_env::Polygon constructCOISearchSpace(eu::nifti::env::Window, float);
    float computeIntersectionOfVizConeAndPolygon(geometry_msgs::Point, float,eu_nifti_env::Polygon, float, float, float);
    std::vector<float> computePlaneFromPolygon(eu_nifti_env::Polygon); // Compute plane equation constants from Polygon points
    int pointInPolygonHelper(geometry_msgs::Point, eu_nifti_env::Polygon);
    void addCarObjectOfInterest(eu::nifti::env::CarObjectOfInterestPtr);
    void addVictimObjectOfInterest(eu::nifti::env::VictimObjectOfInterestPtr);
    void addSignObjectOfInterest(eu::nifti::env::SignObjectOfInterestPtr);
    void modifyCarObjectOfInterest(eu::nifti::env::CarObjectOfInterestPtr);
    void modifyVictimObjectOfInterest(eu::nifti::env::VictimObjectOfInterestPtr);
    void modifySignObjectOfInterest(eu::nifti::env::SignObjectOfInterestPtr);
    void deleteCarObjectOfInterest(eu::nifti::env::CarObjectOfInterestPtr);
    void deleteVictimObjectOfInterest(eu::nifti::env::VictimObjectOfInterestPtr);
    void deleteSignObjectOfInterest(eu::nifti::env::SignObjectOfInterestPtr);
    void addLocationOfInterest(eu::nifti::env::LocationOfInterestPtr loi);
    eu::nifti::env::CarObjectOfInterestPtr computeCOIFunctionalProperties(eu::nifti::env::CarObjectOfInterestPtr);
    eu::nifti::env::FunctionalAreaPtr computeCOIFunctionalArea(eu::nifti::env::Pose,eu::nifti::env::Window,geometry_msgs::Point, float, float, float);
    eu::nifti::env::ListOfVantagePoints computeVantagePointsFromFunctionalAreas(eu::nifti::env::ListOfFunctionalAreas, eu::nifti::env::ListOfWindows, eu::nifti::env::Pose);
    eu::nifti::env::FunctionalAreaPtr computeFunctionalAreaInSearchSpace(eu_nifti_env::Polygon clusterOfPoints, eu_nifti_env::Polygon searchSpacePolygon);

    

protected:
    virtual void start();
    virtual void runComponent();
    virtual void configure(const std::map<std::string, std::string>&);
    virtual void detectCOIModificationInWM(const cast::cdl::WorkingMemoryChange&);
    virtual void acceptPlanningTask(const cast::cdl::WorkingMemoryChange&); 

private:
    ros::Publisher eoiDrawCarFunctionalAreasPublisher; //Publisher to publish polygons
    ros::Subscriber CarInfoSubscriber; //To subscribe to Visual Car Info messages
    ros::Subscriber subscriberEOI; // Listens to ElementOfInterestMessages
    ros::Subscriber OdomSubscriber; //To subscribe to Odometry messages
};

#endif
