// Benoit & Shanker 2011-07-12

#ifndef EU_NIFTI_CONVERTER_UTIL_MAPPING_HPP_
#define EU_NIFTI_CONVERTER_UTIL_MAPPING_HPP_

#include <eu_nifti_env/CarObjectOfInterest.h>
#include <eu_nifti_env/VictimObjectOfInterest.h>
#include <eu_nifti_env/SignObjectOfInterest.h>
#include <eu_nifti_env/LocationOfInterest.h>
#include <eu_nifti_env/AreaOfInterest.h>
#include <eu/nifti/env/CarObjectOfInterest.hpp>
#include <eu/nifti/env/LocationOfInterest.hpp>
#include <eu/nifti/env/VictimObjectOfInterest.hpp>
#include <eu/nifti/env/SignObjectOfInterest.hpp>
#include <eu/nifti/env/AreaOfInterest.hpp>
#include <eu/nifti/env/FunctionalArea.hpp>
#include <eu/nifti/env/Window.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <tf/transform_datatypes.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btScalar.h>


namespace eu
{
    namespace nifti
    {

        class ConverterUtil_Mapping
        {
        public:

	   static inline eu::nifti::env::Point3D convertPointToCAST(const geometry_msgs::Point& rosPoint)
	   {
		eu::nifti::env::Point3D castPoint;
		castPoint.x = rosPoint.x;
		castPoint.y = rosPoint.y;
		castPoint.z = rosPoint.z;
		return castPoint; 
	   }


	   static inline geometry_msgs::Point convertPoint3DToROS(const eu::nifti::env::Point3D castPoint)
	   {
		geometry_msgs::Point rosPoint;
		rosPoint.x = castPoint.x;
		rosPoint.y = castPoint.y;
		rosPoint.z = castPoint.z;
		return rosPoint;
	   }

	   static inline eu::nifti::env::Polygon convertPolygonToCAST(const eu_nifti_env::Polygon& rosPolygon)
	   {
		int arraySize = rosPolygon.points.size();				
                eu::nifti::env::Polygon castPolygon;
                for(int ctr = 0;ctr<arraySize;ctr++)
		   {
			castPolygon.points.push_back(convertPointToCAST(rosPolygon.points[ctr]));
		   }
		return castPolygon;	
	   }

	   static inline eu_nifti_env::Polygon convertPolygonToROS(const eu::nifti::env::Polygon castPolygon)
	   {
		int arraySize = castPolygon.points.size();				
		eu_nifti_env::Polygon rosPolygon;
		for(int ctr = 0;ctr<arraySize;ctr++)
		   {
			rosPolygon.points.push_back(convertPoint3DToROS(castPolygon.points[ctr]));
		   }
		return rosPolygon;	
	   }

	   static inline eu::nifti::env::Pose convertPoseToCAST(const geometry_msgs::Pose& rosPose)
	   {
		eu::nifti::env::Pose castPose;
		castPose.x = rosPose.position.x;
		castPose.y = rosPose.position.y;
		castPose.z = rosPose.position.z;
		geometry_msgs::Quaternion tempQuat;
		tempQuat.x = rosPose.orientation.x;
		tempQuat.y = rosPose.orientation.y;
		tempQuat.z = rosPose.orientation.z;
		tempQuat.w = rosPose.orientation.w;		
		const geometry_msgs::Quaternion quat(tempQuat);
		castPose.theta = tf::getYaw(quat);
		return castPose;
	   }

	   static inline geometry_msgs::Pose convertPoseToROS(const eu::nifti::env::Pose castPose)
	   {
		geometry_msgs::Pose rosPose;
		rosPose.position.x = castPose.x;
		rosPose.position.y = castPose.y;
		rosPose.position.z = castPose.z;
		btVector3 axisOfRotation(0,0,1);
		btQuaternion quat(axisOfRotation,castPose.theta);		
		rosPose.orientation.x = quat.x();
		rosPose.orientation.y = quat.y();
		rosPose.orientation.z = quat.z();
		rosPose.orientation.w = quat.w();
		return rosPose;
	   }

	   static inline eu::nifti::env::ListOfFunctionalAreas convertFunctionalAreaArrayToCAST(const std::vector<eu_nifti_env::FunctionalArea>& rosFunctionalAreaArray)
	   {
		eu::nifti::env::ListOfFunctionalAreas listFA;
		int listSize = rosFunctionalAreaArray.size();
		for(int ctr = 0;ctr<listSize;ctr++)
		   {
		        const eu::nifti::env::FunctionalAreaPtr tempFA = new eu::nifti::env::FunctionalArea(rosFunctionalAreaArray[ctr].function,convertPolygonToCAST(rosFunctionalAreaArray[ctr].area));	   	
			listFA.push_back(tempFA);	
		   }
		return(listFA);
	   }

	   static inline std::vector<eu_nifti_env::FunctionalArea> convertListOfFunctionalAreasToROS(const eu::nifti::env::ListOfFunctionalAreas castFunctionalAreas)
	   {
		std::vector<eu_nifti_env::FunctionalArea> listFA;
		eu_nifti_env::FunctionalArea tempFA;	
		int listSize = castFunctionalAreas.size();
		for(int ctr = 0;ctr<listSize;ctr++)
		   {
			tempFA.function = castFunctionalAreas[ctr]->function;
			tempFA.area = convertPolygonToROS(castFunctionalAreas[ctr]->area);
		   	listFA.push_back(tempFA);
		   }
		return(listFA);
	   }

	   static inline eu::nifti::env::ListOfVantagePoints convertVantagePointArrayToCAST(const std::vector<eu_nifti_env::VantagePoint>& rosVantagePointArray)
	   {
		eu::nifti::env::ListOfVantagePoints listVP;
		int listSize = rosVantagePointArray.size();
		for(int ctr = 0;ctr<listSize;ctr++)
		   {
		        const eu::nifti::env::VantagePointPtr tempVP = new eu::nifti::env::VantagePoint(rosVantagePointArray[ctr].gain,convertPoseToCAST(rosVantagePointArray[ctr].pose),rosVantagePointArray[ctr].function);	   	
			listVP.push_back(tempVP);	
		   }
		return(listVP);
	   }

	   static inline std::vector<eu_nifti_env::VantagePoint> convertListOfVantagePointsToROS(const eu::nifti::env::ListOfVantagePoints castVantagePoints)
	   {
		std::vector<eu_nifti_env::VantagePoint> listVP;
		eu_nifti_env::VantagePoint tempVP;	
		int listSize = castVantagePoints.size();
		for(int ctr = 0;ctr<listSize;ctr++)
		   {
			tempVP.gain = castVantagePoints[ctr]->gain;
			tempVP.function = castVantagePoints[ctr]->function;
			tempVP.pose = convertPoseToROS(castVantagePoints[ctr]->pose);
		   	listVP.push_back(tempVP);
		   }
		return(listVP);
	   }

	   static inline eu::nifti::env::ListOfWindows convertWindowArrayToCAST(const std::vector<eu_nifti_env::Window>& rosWindowArray)
	   {
		eu::nifti::env::ListOfWindows listWin;
		int listSize = rosWindowArray.size();
		eu::nifti::env::Window tempWin;
		for(int ctr = 0;ctr<listSize;ctr++)
		   {
		        tempWin.width = rosWindowArray[ctr].width;
			tempWin.height = rosWindowArray[ctr].height;
			tempWin.pose = convertPoseToCAST(rosWindowArray[ctr].pose);
		   	listWin.push_back(tempWin);	
		   }
		return(listWin);
	   }

	   static inline std::vector<eu_nifti_env::Window> convertListOfWindowsToROS(const eu::nifti::env::ListOfWindows castWindows)
	   {
		std::vector<eu_nifti_env::Window> listWin;
		eu_nifti_env::Window tempWin;
		int listSize = castWindows.size();
		for(int ctr = 0;ctr<listSize;ctr++)
		   {
			tempWin.width = castWindows[ctr].width;
			tempWin.height = castWindows[ctr].height;
			tempWin.pose = convertPoseToROS(castWindows[ctr].pose);
		   	listWin.push_back(tempWin);	
		   }
		return(listWin);
	   }
	  
            static inline eu::nifti::env::CarObjectOfInterestPtr convertCarOfInterestToCAST(const eu_nifti_env::CarObjectOfInterest& coi)
	    {
		const eu::nifti::env::Polygon castBoundingBox = convertPolygonToCAST(coi.object.boundingBox);
		const eu::nifti::env::Pose castPose = convertPoseToCAST(coi.object.pose);
		const eu::nifti::env::ListOfFunctionalAreaWMPs castFuncAreaWMPs;
		const eu::nifti::env::ListOfFunctionalAreas castFunctionalAreasList = convertFunctionalAreaArrayToCAST(coi.functionalAreas);
		const eu::nifti::env::ListOfWindows castWindowsList = convertWindowArrayToCAST(coi.windows);
		const eu::nifti::env::ListOfVantagePoints castVantagePointsList = convertVantagePointArrayToCAST(coi.vantagePoints);
		return new eu::nifti::env::CarObjectOfInterest(
			coi.object.element.uuid,
			coi.object.element.name,
			eu::nifti::env::ElementOfInterestType(coi.object.element.type),
			eu::nifti::env::ElementOfInterestSourceType(coi.object.element.sourceType),
			coi.object.element.confidence,
			eu::nifti::env::ElementOfInterestStatus(coi.object.element.status),
			castBoundingBox,
			castPose,
			coi.carClass,
			castFuncAreaWMPs,
			castFunctionalAreasList,
			castVantagePointsList,
			castWindowsList
			);
	    }

	    static inline eu_nifti_env::CarObjectOfInterest convertCarOfInterestToROS(const eu::nifti::env::CarObjectOfInterestPtr coi)
	    {
		eu_nifti_env::CarObjectOfInterest rosCOI;
		rosCOI.object.element.uuid = coi->uuid;
		rosCOI.object.element.name = coi->name;
		rosCOI.object.element.type = coi->type;
		rosCOI.object.element.sourceType = coi->sourceType;
		rosCOI.object.element.confidence = coi->confidence;
		rosCOI.object.element.status = coi->status;
		rosCOI.object.boundingBox = convertPolygonToROS(coi->boundingBox);
		rosCOI.object.pose = convertPoseToROS(coi->pose);
		rosCOI.carClass = coi->carClass;
		rosCOI.functionalAreas = convertListOfFunctionalAreasToROS(coi->functionalAreas);
		rosCOI.vantagePoints = convertListOfVantagePointsToROS(coi->vantagePoints);
		rosCOI.windows = convertListOfWindowsToROS(coi->windows);
		return eu_nifti_env::CarObjectOfInterest(rosCOI); 
	    }

            static inline eu::nifti::env::VictimObjectOfInterestPtr convertVictimOfInterestToCAST(const eu_nifti_env::VictimObjectOfInterest& voi)
	    {
		const eu::nifti::env::Polygon castBoundingBox = convertPolygonToCAST(voi.object.boundingBox);
		const eu::nifti::env::Pose castPose = convertPoseToCAST(voi.object.pose);
		return new eu::nifti::env::VictimObjectOfInterest(
			voi.object.element.uuid,
			voi.object.element.name,
			eu::nifti::env::ElementOfInterestType(voi.object.element.type),
			eu::nifti::env::ElementOfInterestSourceType(voi.object.element.sourceType),
			voi.object.element.confidence,
			eu::nifti::env::ElementOfInterestStatus(voi.object.element.status),
			castBoundingBox,
			castPose
			);
	    }

	    static inline eu_nifti_env::VictimObjectOfInterest convertVictimOfInterestToROS(const eu::nifti::env::VictimObjectOfInterestPtr voi)
	    {
		eu_nifti_env::VictimObjectOfInterest rosVOI;
		rosVOI.object.element.uuid = voi->uuid;
		rosVOI.object.element.name = voi->name;
		rosVOI.object.element.type = voi->type;
		rosVOI.object.element.sourceType = voi->sourceType;
		rosVOI.object.element.confidence = voi->confidence;
		rosVOI.object.element.status = voi->status;
		rosVOI.object.boundingBox = convertPolygonToROS(voi->boundingBox);
		rosVOI.object.pose = convertPoseToROS(voi->pose);
		return(rosVOI); 
	    }
            
	    static inline eu::nifti::env::SignObjectOfInterestPtr convertSignOfInterestToCAST(const eu_nifti_env::SignObjectOfInterest& soi)
	    {
		const eu::nifti::env::Polygon castBoundingBox = convertPolygonToCAST(soi.object.boundingBox);
		const eu::nifti::env::Pose castPose = convertPoseToCAST(soi.object.pose);
		return new eu::nifti::env::SignObjectOfInterest(
			soi.object.element.uuid,
			soi.object.element.name,
			eu::nifti::env::ElementOfInterestType(soi.object.element.type),
			eu::nifti::env::ElementOfInterestSourceType(soi.object.element.sourceType),
			soi.object.element.confidence,
			eu::nifti::env::ElementOfInterestStatus(soi.object.element.status),
			castBoundingBox,
			castPose,
			eu::nifti::env::SignObjectType(soi.signType)
			);
	    }

	    static inline eu_nifti_env::SignObjectOfInterest convertSignOfInterestToROS(const eu::nifti::env::SignObjectOfInterestPtr soi)
	    {
		eu_nifti_env::SignObjectOfInterest rosSOI;
		rosSOI.object.element.uuid = soi->uuid;
		rosSOI.object.element.name = soi->name;
		rosSOI.object.element.type = soi->type;
		rosSOI.object.element.sourceType = soi->sourceType;
		rosSOI.object.element.confidence = soi->confidence;
		rosSOI.object.element.status = soi->status;
		rosSOI.object.boundingBox = convertPolygonToROS(soi->boundingBox);
		rosSOI.object.pose = convertPoseToROS(soi->pose);
		rosSOI.signType = soi->signType;
		return(rosSOI); 
	    }

	    static inline eu::nifti::env::ElementOfInterestPtr convertElementOfInterestToCAST(const eu_nifti_env::ElementOfInterest& eoi)
	    {
		return new eu::nifti::env::ElementOfInterest(
			eoi.uuid,
			eoi.name,
			eu::nifti::env::ElementOfInterestType(eoi.type),
			eu::nifti::env::ElementOfInterestSourceType(eoi.sourceType),
			eoi.confidence,
			eu::nifti::env::ElementOfInterestStatus(eoi.status)
			);
	    }

            static inline eu::nifti::env::LocationOfInterestPtr convertLocationOfInterestToCAST(const eu_nifti_env::LocationOfInterest& loi)
	    {
		//const eu::nifti::env::Polygon castBoundingBox = convertPolygonToCAST(loi.object.boundingBox);
		//const eu::nifti::env::Pose castPose = convertPoseToCAST(loi.object.pose);
		return new eu::nifti::env::LocationOfInterest(
			loi.element.uuid,
			loi.element.name,
			eu::nifti::env::ElementOfInterestType(loi.element.type),
			eu::nifti::env::ElementOfInterestSourceType(loi.element.sourceType),
			loi.element.confidence,
			eu::nifti::env::ElementOfInterestStatus(loi.element.status),
			convertPointToCAST(loi.point)
			);
	    }

          static inline eu::nifti::env::AreaOfInterestPtr convertAreaOfInterestToCAST(const eu_nifti_env::AreaOfInterest& aoi)
          {
        	  return new eu::nifti::env::AreaOfInterest(
        			  aoi.element.uuid,
        			  aoi.element.name,
        			  eu::nifti::env::ElementOfInterestType(aoi.element.type),
        			  eu::nifti::env::ElementOfInterestSourceType(aoi.element.sourceType),
        			  aoi.element.confidence,
        			  eu::nifti::env::ElementOfInterestStatus(aoi.element.status),
        			  convertPolygonToCAST(aoi.polygon)
        			  );
          }

        };

    };

};

#endif // EU_NIFTI_CONVERTER_UTIL_MAPPING_HPP_

