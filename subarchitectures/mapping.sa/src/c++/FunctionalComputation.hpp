/* 
 * File:   FunctionalAreaComputation.hpp
 * Author: shanker
 *
 * Created on October 12, 2011, 3:25 PM
 */

#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP

#include "FEMAreasGenerator.hpp"
#include "GeometricalComputation.hpp"
#define SIZE_OF_FACE 0.04
#define GAIN_PER_FUNCTIONAL_AREA 0.8 // This is a value used by planning

eu_nifti_env::Polygon FEMAreasGenerator::computeWindowPointsFromDimensions(eu::nifti::env::Window currentWindow)
{
    // Find coordinates of window
    geometry_msgs::Vector3 _positionOfWinCenter = FEMAreasGenerator::constructVector3(currentWindow.pose.x,currentWindow.pose.y,currentWindow.pose.z);
    eu_nifti_env::Polygon winPolygon;
    winPolygon.points.push_back(constructPoint(0, currentWindow.width/2, currentWindow.height/2));
    winPolygon.points.push_back(constructPoint(0, -currentWindow.width/2, currentWindow.height/2));
    winPolygon.points.push_back(constructPoint(0, -currentWindow.width/2, -currentWindow.height/2));
    winPolygon.points.push_back(constructPoint(0, currentWindow.width/2, -currentWindow.height/2));
    winPolygon = FEMAreasGenerator::rotateAndTranslateROSPolygon(winPolygon,_positionOfWinCenter,0,0,currentWindow.pose.theta);
    return(winPolygon);
}

eu_nifti_env::Polygon FEMAreasGenerator::constructCOISearchSpace(eu::nifti::env::Window currentWindow, float camRange)
{
    // Temporarily construct search space of 3m*3m rectangle
    eu_nifti_env::Polygon searchSpacePolygon;
    searchSpacePolygon.points.push_back(constructPoint(0,-0.8*camRange,0));
    searchSpacePolygon.points.push_back(constructPoint(camRange,-0.8*camRange,0));
    searchSpacePolygon.points.push_back(constructPoint(camRange,0.8*camRange,0));
    searchSpacePolygon.points.push_back(constructPoint(0,0.8*camRange,0));
    geometry_msgs::Vector3 _positionOfWinGround = FEMAreasGenerator::constructVector3(currentWindow.pose.x,currentWindow.pose.y,0);
    searchSpacePolygon = FEMAreasGenerator::rotateAndTranslateROSPolygon(searchSpacePolygon,_positionOfWinGround,0,0,currentWindow.pose.theta);
    return(searchSpacePolygon);
}

eu::nifti::env::FunctionalAreaPtr FEMAreasGenerator::computeCOIFunctionalArea(eu::nifti::env::Pose carPose,eu::nifti::env::Window viewWindow, geometry_msgs::Point cameraPositionRelativeToRobot,
                                                                                float vertAOVSensor, float horizAOVSensor, float camRange)
{
    eu_nifti_env::Polygon winPolygon = FEMAreasGenerator::computeWindowPointsFromDimensions(viewWindow); // Polygon of Window points
    eu_nifti_env::Polygon searchSpacePolygon = FEMAreasGenerator::constructCOISearchSpace(viewWindow, camRange);   // Polygon of Search Space points
    float _angleOfRobotFacingWindow = M_PI + viewWindow.pose.theta;                                      //Angle of robot facing the window (is M_PI + angle of window)
    cameraPositionRelativeToRobot = rotateAndTranslatePoint(cameraPositionRelativeToRobot, FEMAreasGenerator::constructVector3(0,0,0) , 0 , 0 , _angleOfRobotFacingWindow); //Turning the point to face the window

    eu_nifti_env::Polygon clusterOfPointsPolygon; // These points afford visibility
    for(float i = 0; i<=1; i+=0.05)
    {  for(float j = 0; j<=1 ; j+=0.05)
        {
            geometry_msgs::Point pointInSS = constructPoint( ((1-i)*searchSpacePolygon.points[0].x + i*searchSpacePolygon.points[3].x)*(1-j) + ((1-i)*searchSpacePolygon.points[1].x + i*searchSpacePolygon.points[2].x)*(j) ,
                                                                ((1-i)*searchSpacePolygon.points[0].y + i*searchSpacePolygon.points[3].y)*(1-j) + ((1-i)*searchSpacePolygon.points[1].y + i*searchSpacePolygon.points[2].y)*(j) , 0); // Point (i,j) in rectangle
            geometry_msgs::Point _posofcam = constructPoint((pointInSS.x+cameraPositionRelativeToRobot.x),(pointInSS.y+cameraPositionRelativeToRobot.y),(pointInSS.z+cameraPositionRelativeToRobot.z));
            float patchSize = computeIntersectionOfVizConeAndPolygon(_posofcam, _angleOfRobotFacingWindow ,winPolygon, vertAOVSensor, horizAOVSensor, camRange);     //Compute visualization function , how much of the window can the camera see at that position, look in search space
            if(patchSize>=SIZE_OF_FACE) // Patch size should be greater than size of face for reliable detection
            {
               clusterOfPointsPolygon.points.push_back(pointInSS); 
            }
        }
    }
    eu::nifti::env::FunctionalAreaPtr _functionalArea = computeFunctionalAreaInSearchSpace(clusterOfPointsPolygon,searchSpacePolygon); //Compute a functional area from the visibility points
    geometry_msgs::Vector3 carDisplacement = constructVector3(carPose.x,carPose.y,0);    
    for(unsigned int ctr = 0; ctr<_functionalArea->area.points.size(); ctr++) // Now transform the functional areas to the global coordinates , they are currently in car frame
    {
        _functionalArea->area.points[ctr] = eu::nifti::ConverterUtil_Mapping::convertPointToCAST(rotateAndTranslatePoint(eu::nifti::ConverterUtil_Mapping::convertPoint3DToROS(_functionalArea->area.points[ctr]),carDisplacement,0,0,carPose.theta));
    }
    return(_functionalArea);
}

 eu::nifti::env::FunctionalAreaPtr FEMAreasGenerator::computeFunctionalAreaInSearchSpace(eu_nifti_env::Polygon clusterOfPoints, eu_nifti_env::Polygon searchSpacePolygon)
 {
    // This function computes the closest points in the cluster, to the searchSpacePolygon points and returns them as a functional area
    // It is assumed that the searchSpacePolygon points are in anticlockwise pattern
     eu::nifti::env::FunctionalAreaPtr functionalArea = new eu::nifti::env::FunctionalArea();
     functionalArea->function = "Victim Search";

     for(unsigned int ctr = 0;ctr<searchSpacePolygon.points.size();ctr++)
     {
         float minDistance = computeDistanceBetweenPoints(searchSpacePolygon.points[ctr],clusterOfPoints.points[0]);
         unsigned int marker = 0;
         for(unsigned int ctr2 = 0;ctr2<clusterOfPoints.points.size();ctr2++)
         {
             if(computeDistanceBetweenPoints(searchSpacePolygon.points[ctr],clusterOfPoints.points[ctr2])<minDistance)
             {
                 minDistance = computeDistanceBetweenPoints(searchSpacePolygon.points[ctr],clusterOfPoints.points[ctr2]);
                 marker = ctr2;
             }
         }
         eu::nifti::env::Point3D tempPoint = eu::nifti::ConverterUtil_Mapping::convertPointToCAST(clusterOfPoints.points[marker]);
         functionalArea->area.points.push_back(tempPoint);
     }
     return(functionalArea);
 }

 eu::nifti::env::ListOfVantagePoints FEMAreasGenerator::computeVantagePointsFromFunctionalAreas(eu::nifti::env::ListOfFunctionalAreas listFA, eu::nifti::env::ListOfWindows listWin, eu::nifti::env::Pose carPose)
 {
     eu::nifti::env::ListOfVantagePoints listVP;
     //Compute vantage point poses from Functional Areas
    for(unsigned int ctr = 0; ctr<listFA.size();ctr++)
    {
        eu::nifti::env::VantagePointPtr _vantagePoint = new eu::nifti::env::VantagePoint(); // Are these initialized to zero?
	_vantagePoint->pose.x = 0; _vantagePoint->pose.y = 0; _vantagePoint->pose.z = 0; _vantagePoint->pose.theta = 0; 
	//log("The vantage point poses are initialized as x: %6.4f y: %6.4f z: %6.4f",_vantagePoint->pose.x,_vantagePoint->pose.y,_vantagePoint->pose.z);
        _vantagePoint->function = listFA[ctr]->function;
        _vantagePoint->gain = GAIN_PER_FUNCTIONAL_AREA;
        for(unsigned int ctr2 = 0; ctr2<listFA[ctr]->area.points.size(); ctr2++)
        {
            _vantagePoint->pose.x = _vantagePoint->pose.x + listFA[ctr]->area.points[ctr2].x;
            _vantagePoint->pose.y = _vantagePoint->pose.y + listFA[ctr]->area.points[ctr2].y;
            _vantagePoint->pose.z = _vantagePoint->pose.z + listFA[ctr]->area.points[ctr2].z;
        }
        _vantagePoint->pose.x = _vantagePoint->pose.x/listFA[ctr]->area.points.size(); // This is the center of the functional area
        _vantagePoint->pose.y = _vantagePoint->pose.y/listFA[ctr]->area.points.size();
        _vantagePoint->pose.z = _vantagePoint->pose.z/listFA[ctr]->area.points.size();
	//log("The position of the vantagePoints was determined as as x: %6.4f y: %6.4f z: %6.4f",_vantagePoint->pose.x,_vantagePoint->pose.y,_vantagePoint->pose.z);
	//log("The position of the car is x: %6.4f y: %6.4f z: %6.4f theta: %6.4f",carPose.x,carPose.y,carPose.z,carPose.theta);
	//log("The relevant window pose is theta: %6.4f",listWin[ctr].pose.theta);
        _vantagePoint->pose.theta = M_PI + (carPose.theta + listWin[ctr].pose.theta); // CarPose brings the WindowPose to the global frame
	//log("So the vantage point pose is theta: %6.4f",_vantagePoint->pose.theta);
        listVP.push_back(_vantagePoint);
    }
     return(listVP);
 }

 eu::nifti::env::CarObjectOfInterestPtr FEMAreasGenerator::computeCOIFunctionalProperties(eu::nifti::env::CarObjectOfInterestPtr coi)
 {
     // Computes all functional properties : Functional Areas, Bounding Box ...
    // Query Ontology
     char dummyStr[200];
    geometry_msgs::Point _cameraPositionWRTRobot = constructPoint(FEMAreasGenerator::queryOntologyFloat(strcpy(dummyStr,"SELECT ?x WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasPosX> ?x"))
                                                    ,FEMAreasGenerator::queryOntologyFloat(strcpy(dummyStr,"SELECT ?y WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasPosY> ?y"))
                                                    ,FEMAreasGenerator::queryOntologyFloat(strcpy(dummyStr,"SELECT ?z WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasPosZ> ?z")) );
    float _camRange = FEMAreasGenerator::queryOntologyFloat(strcpy(dummyStr,"SELECT ?range WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasVisualRange> ?range"));
    float _camVertAOV = FEMAreasGenerator::queryOntologyFloat(strcpy(dummyStr,"SELECT ?vAOV WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasVerticalAngleOfView> ?vAOV"));
    float _camHorizAOV = FEMAreasGenerator::queryOntologyFloat(strcpy(dummyStr,"SELECT ?hAOV WHERE <funcmap:nifti-ugv> <funcmap:hasPart> ?camera & ?camera <rdf:type> <funcmap:Camera> & ?camera <funcmap:hasHorizontalAngleOfView> ?hAOV"));

    //Compute Functional Areas
    for(unsigned int ctr = 0; ctr<coi->windows.size();ctr++)
    {
        this->tempWindowNumber = ctr;
        eu::nifti::env::FunctionalAreaPtr generatedFunctionalArea = new eu::nifti::env::FunctionalArea();
        generatedFunctionalArea = FEMAreasGenerator::computeCOIFunctionalArea(coi->pose,coi->windows[ctr],_cameraPositionWRTRobot,_camVertAOV,_camHorizAOV,_camRange);
        debug("Automatically computing areas for window %d",ctr);
        coi->functionalAreas.push_back(generatedFunctionalArea);
    }

    coi->vantagePoints = FEMAreasGenerator::computeVantagePointsFromFunctionalAreas(coi->functionalAreas,coi->windows, coi->pose); //Compute Vantage Points

    //Aligning the bounding box to the Car's coordinates
    coi->boundingBox = FEMAreasGenerator::rotateAndTranslateCASTPolygon(coi->boundingBox,FEMAreasGenerator::constructVector3(coi->pose.x,coi->pose.y,coi->pose.z) ,0,0,coi->pose.theta);

    return(coi);
 }


#endif	/* FUNCTIONALCOMPUTATION_HPP */

