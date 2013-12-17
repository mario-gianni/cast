/*
 * PlanningUtils.hpp
 *
 *  Created on: Dec 29, 2011
 *      Author: alcor
 */

#ifndef PLANNINGUTILS_HPP_
#define PLANNINGUTILS_HPP_

#include <eu/nifti/env/CarObjectOfInterest.hpp>
#include <planning_roma.hpp>
#include <topograph.hpp>
#include <robot_position.hpp>
#include <are.hpp>
#include <diagnostic.hpp>

using namespace eu::nifti::Planning::slice;
using namespace eu::nifti::env::topograph;
using namespace eu::nifti::env;
using namespace eu::nifti::env::position;
using namespace eu::nifti::env::are;
using namespace eu::nifti::env::diagnostic;

class PlanningUtils
{
public:
	PlanningUtils();
	~PlanningUtils();

	void storeNodes(NodePtr,std::string);
	void storeEdges(EdgePtr,std::string);

	void storeBaseStation(BasePosPtr,std::string);

	void storeDetectedObject(CarObjectOfInterestPtr,std::string);
	void storeVantagePoints(CarObjectOfInterestPtr,std::string);

	void storeArtefact(ArtefactPtr,std::string);
	void storeBatteryStatus(BatteryStatusPtr,std::string);
	void storeWifiStatus(WiFiStatusPtr,std::string);

	void storeNodeWifiStrenght(CurrentPosPtr,WiFiStatusPtr,std::string);

 	std::string to_string(double);
	std::string to_string(NodePtr);
	std::string to_string(EdgePtr);
	std::string to_string(ActionPtr);
	std::string to_string(PosePtr);
};


#endif /* PLANNINGUTILS_HPP_ */
