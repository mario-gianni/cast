// AUTHOR : SHANKER KESHAVDAS , DFKI SAARBRUECKEN

#include "FEMAreasGenerator.hpp"
#include "FunctionalComputation.hpp"
#include "GeometricalComputation.hpp"
#include "ROSMsgConstructors.hpp"
#include "Conversions.hpp"
#include "OntologyQuerier.hpp"


//Nov 9, 2010: The code reads from Visual Car Info messages on ROS and infers topodebugical regions around the detected car and writes them
//             Working Memory (as Polygons and Quadtree Nodes) and publishes them in ROS as well.
//Dec 16, 2010: The code reads from vision_msgs/DetectedObject, infers topodebugical regions around the object and writes them to WM as
//             polygons and publishes on ROS. It cooperates on getting the right ontodebugical inferences about the object from "MapObjectMonitor"
//             component. At the moment, it assumes the object is a car.
//Mar 18, 2011: The code inputs only unique ObjectIDs. Repeated IDs are ignored. Also, Window size is read from the update by the component 
//             "MapObjectMonitor" and used in generating the polygons. Also, the code no longer uses segmentation from toposeg component. 
//Mar 25, 2011: The code has been modified to include modifications and deletions which is reflected in the WM and in ROS polygons being published.
//Mar 28, 2011: When an object is deleted or modified, a new entry with that name cannot be added to WM.
//Apr 7, 2011: Refactored code to increase modularity, fixed bug of polygons having same colours in GUI.
//Oct 1, 2011: Perform 3D inferencing

// Check if a UUID is being processed
bool FEMAreasGenerator::isUUIDBeingProcessed(Ice::Int checkUUID)
{
    bool beingProcessed = false;
    for(unsigned int ctr = 0;ctr<this->processingUUIDs.size();ctr++)
    {
        if(this->processingUUIDs[ctr]==checkUUID)
        {
            beingProcessed = true;
            break;
        }
    }
    return(beingProcessed);
}

// Remove a UUID from the processingUUIDs list
void FEMAreasGenerator::removeUUIDFromProcessingList(Ice::Int removeUUID)
{
   for(unsigned int ctr = 0;ctr<this->processingUUIDs.size();ctr++)
        {
            if(this->processingUUIDs[ctr]==removeUUID)
            {
                this->processingUUIDs.erase(processingUUIDs.begin() + ctr);
            }
        }
}

// Wait till the processing of a UUID is complete
void FEMAreasGenerator::waitTillUUIDIsFree(Ice::Int newUUID)
{
   while(FEMAreasGenerator::isUUIDBeingProcessed(newUUID)==true)
   {
       sleep(0.1);
   }
   return;
}

// Add a UUID to the processingUUIDs list
void FEMAreasGenerator::addUUIDToProcessingList(Ice::Int newUUID)
{
    this->processingUUIDs.push_back(newUUID);
    return;
}

// Add a UUID to the processingUUIDs list, when it is free
void FEMAreasGenerator::addUUIDToProcessingListWhenFree(Ice::Int newUUID)
{
    FEMAreasGenerator::waitTillUUIDIsFree(newUUID);
    FEMAreasGenerator::addUUIDToProcessingList(newUUID);
    return;
}

// Accept new Planning Task
void FEMAreasGenerator::acceptPlanningTask(const cast::cdl::WorkingMemoryChange& _wmc)
{
    try
        {
            this->planningControlAction = getMemoryEntry<eu::nifti::Planning::slice::FunctionalMappingAction> (_wmc.address);
        }
    catch(char *str)
        {
            log("Could not get planning control action: %s",str);
        }
    return;
}

// Detect overwrite made by "MapObjectMonitor" component, calculate and publish functional areas in CAST and ROS
void FEMAreasGenerator::detectCOIModificationInWM(const cast::cdl::WorkingMemoryChange & _wmc)
{
    if (_wmc.src.compare("FEMAreasGenerator") == 0) //Is it this component that is making the changes?
        {
            debug("A modification is made by our own component, this is ignored");
            return;
        }
    debug("A modification has been made by %s , and we continued ", _wmc.src.c_str());

    eu::nifti::env::CarObjectOfInterestPtr readCar;
    try
        {
            readCar = getMemoryEntry<eu::nifti::env::CarObjectOfInterest > (_wmc.address);
        }
    catch(char *str)
        {
            log("Could not getMemoryEntry during adding of Car: %s",str);
            return;
        }

    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(readCar->uuid);

    //Removing the address filter
    std::map<std::string, cast::MemberFunctionChangeReceiver<FEMAreasGenerator>*>::iterator it;
    it = this->tempMap.find(_wmc.address.id);
    if ((it != this->tempMap.end()) && (!this->tempMap.empty()))
        {
            debug("Found the Working Memory Change Receiver corresponding to %s",_wmc.address.id.c_str());
            removeChangeFilter(it->second);
        }

    readCar = FEMAreasGenerator::computeCOIFunctionalProperties(readCar);

    std::string dataID = _wmc.address.id;
    try{
    overwriteWorkingMemory(dataID, readCar);
    }
    catch(char *str)
    {
        log("Could not overwrite WM: %s",str);
    }

    // Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(readCar->uuid);

    //Making ROS message to publish
    eu_nifti_env_msg_ros::ElementOfInterestMessage tempEOIMsg;
    tempEOIMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_ADD;
    tempEOIMsg.header.stamp = ros::Time::now();
    tempEOIMsg.header.frame_id = "/map";
    tempEOIMsg.type = eu_nifti_env::ElementOfInterest::TYPE_CAR;
    tempEOIMsg.car = eu::nifti::ConverterUtil_Mapping::convertCarOfInterestToROS(readCar);
    tempEOIMsg.castWorkingMemoryPointer.address.id = _wmc.address.id;
    tempEOIMsg.castWorkingMemoryPointer.address.subarchitecture = _wmc.address.subarchitecture;
    tempEOIMsg.castWorkingMemoryPointer.type = "VictimSearch";

    this->eoiDrawCarFunctionalAreasPublisher.publish(tempEOIMsg);
    ros::spinOnce();
    return;
}

void FEMAreasGenerator::onElementOfInterestMessageReceived(const eu_nifti_env_msg_ros::ElementOfInterestMessageConstPtr &EOIMsg)
{
    //println("Message received");
    if(!((EOIMsg->type==eu_nifti_env::ElementOfInterest::TYPE_CAR)||(EOIMsg->type==eu_nifti_env::ElementOfInterest::TYPE_VICTIM)||(EOIMsg->type==eu_nifti_env::ElementOfInterest::TYPE_SIGN)))
    {	
	// This should not be in this class
	if(EOIMsg->type==eu_nifti_env::ElementOfInterest::TYPE_LOCATION)
	{
            eu::nifti::env::LocationOfInterestPtr loi = eu::nifti::ConverterUtil_Mapping::convertLocationOfInterestToCAST(EOIMsg->location);
            addLocationOfInterest(loi);
	}
	return; // A location and area don't have an object, so there is nothing further to process
    }

    const eu_nifti_env::ObjectOfInterest* OOIReceived = eu::nifti::env::msg::ros::Util::getOOI(EOIMsg);
	
    bool isMessageRepeated = FEMAreasGenerator::checkRepetitionOfOOI(OOIReceived); // Check if this message has been received before
    if((isMessageRepeated==true)&&(this->lastActionReceived==EOIMsg->action))
    {
        //println("Message repeated and ignored");
        return;
    }
    this->lastActionReceived = EOIMsg->action;
    switch(EOIMsg->action)
    {
        case eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_ADD: 
        {
            debug("Message received, Type is ADD");
            switch(EOIMsg->type)
            {
                    case eu_nifti_env::ElementOfInterest::TYPE_CAR:
                    {
                        eu::nifti::env::CarObjectOfInterestPtr coi = eu::nifti::ConverterUtil_Mapping::convertCarOfInterestToCAST(EOIMsg->car);
                        addCarObjectOfInterest(coi);
                    }
                    break;
                    case eu_nifti_env::ElementOfInterest::TYPE_SIGN:
                    {
                        eu::nifti::env::SignObjectOfInterestPtr soi = eu::nifti::ConverterUtil_Mapping::convertSignOfInterestToCAST(EOIMsg->sign);
                        addSignObjectOfInterest(soi);
                    }
                    break;
                    case eu_nifti_env::ElementOfInterest::TYPE_VICTIM:
                    {
                        eu::nifti::env::VictimObjectOfInterestPtr voi = eu::nifti::ConverterUtil_Mapping::convertVictimOfInterestToCAST(EOIMsg->victim);
                        addVictimObjectOfInterest(voi);
                    }
                    break;
                    default:
                    log("Could not determine the type of the ADDED Object");
                    break;
            }
        }
        break;
        case eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_MODIFY:
        {
            debug("Message received, Type is MODIFY");
            switch(EOIMsg->type)
            {
                    case eu_nifti_env::ElementOfInterest::TYPE_CAR:
                    {
                        eu::nifti::env::CarObjectOfInterestPtr coi = eu::nifti::ConverterUtil_Mapping::convertCarOfInterestToCAST(EOIMsg->car);
                        modifyCarObjectOfInterest(coi);
                    }
                    break;
                    case eu_nifti_env::ElementOfInterest::TYPE_SIGN:
                    {
                        eu::nifti::env::SignObjectOfInterestPtr soi = eu::nifti::ConverterUtil_Mapping::convertSignOfInterestToCAST(EOIMsg->sign);
                        modifySignObjectOfInterest(soi);
                    }
                    break;
                    case eu_nifti_env::ElementOfInterest::TYPE_VICTIM:
                    {
                        eu::nifti::env::VictimObjectOfInterestPtr voi = eu::nifti::ConverterUtil_Mapping::convertVictimOfInterestToCAST(EOIMsg->victim);
			log("1 Victim of interest received modify status %d",EOIMsg->victim.object.element.status);
                        modifyVictimObjectOfInterest(voi);
                    }
                    break;
                    default:
                    log("Could not determine type of MODIFY Object");
                    break;
            }
        }
        break;
        case eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_REMOVE:
        {
            debug("Message received, Type is REMOVE");
            switch(EOIMsg->type)
            {
                    case eu_nifti_env::ElementOfInterest::TYPE_CAR:
                    {
                        eu::nifti::env::CarObjectOfInterestPtr coi = eu::nifti::ConverterUtil_Mapping::convertCarOfInterestToCAST(EOIMsg->car);
                        deleteCarObjectOfInterest(coi); 
                    }
                    break;
                    case eu_nifti_env::ElementOfInterest::TYPE_SIGN:
                    {
                        eu::nifti::env::SignObjectOfInterestPtr soi = eu::nifti::ConverterUtil_Mapping::convertSignOfInterestToCAST(EOIMsg->sign);
                        deleteSignObjectOfInterest(soi);
                    }
                    break;
                    case eu_nifti_env::ElementOfInterest::TYPE_VICTIM:
                    {
                        eu::nifti::env::VictimObjectOfInterestPtr voi = eu::nifti::ConverterUtil_Mapping::convertVictimOfInterestToCAST(EOIMsg->victim);
                        deleteVictimObjectOfInterest(voi);
                    }
                    break;
                    default:
                    log("Could not determine type of REMOVE Object");
                    break;
            }
        }
        break;
        default:
            log("Could not determine if object is to be ADDED, MODIFIED or REMOVED");
            break;
    }
    
    return;
}

void FEMAreasGenerator::addCarObjectOfInterest(eu::nifti::env::CarObjectOfInterestPtr coi)
{
    // Check whether this CarObjectOfInterest has been added yet, by checking in hashmap
    std::map<int, std::string>::iterator it; 
    it = this->mapUUIDToWMAForDetectedObjects.find(coi->uuid);
    if ((it != this->mapUUIDToWMAForDetectedObjects.end()) && (!this->mapUUIDToWMAForDetectedObjects.empty()))
    {
        log("Car Object being added has existing UUID = %d, will not add Object",coi->uuid);
        return;
    }
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(coi->uuid);

	//Artificially ground cars , TODO : This fix is due to the cars floating
	coi->pose.z = 0;

	// Create an address filter
    std::string tempDataID = newDataID();
    cast::MemberFunctionChangeReceiver<FEMAreasGenerator> *TempReceiver = new cast::MemberFunctionChangeReceiver<FEMAreasGenerator > (this, &FEMAreasGenerator::detectCOIModificationInWM);
    this->tempMap[tempDataID] = TempReceiver;
    addChangeFilter(cast::createAddressFilter(tempDataID,getSubarchitectureID(),cast::cdl::OVERWRITE), TempReceiver);
    addToWorkingMemory(tempDataID, coi);
    debug("We have added %s", coi->carClass.c_str());
    this->mapUUIDToWMAForDetectedObjects[coi->uuid] = tempDataID;

    // Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(coi->uuid);
    return;
}

void FEMAreasGenerator::addSignObjectOfInterest(eu::nifti::env::SignObjectOfInterestPtr soi)
{
    // Check whether this SignObjectOfInterest has been added yet, by checking in hashmap
    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(soi->uuid);
    if ((it != this->mapUUIDToWMAForDetectedObjects.end()) && (!this->mapUUIDToWMAForDetectedObjects.empty()))
    {
        log("Sign Object being added has existing UUID = %d, will not add Object",soi->uuid);
        return;
    }
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(soi->uuid);
    std::string tempDataID = newDataID();
    addToWorkingMemory(tempDataID, soi);
    this->mapUUIDToWMAForDetectedObjects[soi->uuid] = tempDataID;
    // Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(soi->uuid);
    return;
}

void FEMAreasGenerator::addVictimObjectOfInterest(eu::nifti::env::VictimObjectOfInterestPtr voi)
{
    // Check whether this VictimObjectOfInterest has been added yet, by checking in hashmap
    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(voi->uuid);
    if ((it != this->mapUUIDToWMAForDetectedObjects.end()) && (!this->mapUUIDToWMAForDetectedObjects.empty()))
    {
        log("Victim Object being added has existing UUID = %d, will not add Object",voi->uuid);
        return;
    }

    std::string tempDataID = newDataID();
    addToWorkingMemory(tempDataID, voi);
    this->mapUUIDToWMAForDetectedObjects[voi->uuid] = tempDataID;
    // Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(voi->uuid);
    return;
}

void FEMAreasGenerator::modifyCarObjectOfInterest(eu::nifti::env::CarObjectOfInterestPtr modifyCOI)
{
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(modifyCOI->uuid);

    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(modifyCOI->uuid);
    if (it == this->mapUUIDToWMAForDetectedObjects.end())
    {
        debug("Car Object of Interest for UUID %d does not exist, cannot modify", modifyCOI->uuid);
        FEMAreasGenerator::removeUUIDFromProcessingList(modifyCOI->uuid);
        return;
    }
    cast::cdl::WorkingMemoryAddress WMAOfModification;
    WMAOfModification.id = it->second;
    WMAOfModification.subarchitecture = getSubarchitectureID();
    eu::nifti::env::CarObjectOfInterestPtr origCar;
    try{
        origCar = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(WMAOfModification);
    }
    catch(char *str)
    {
        log("Could not get from WM during modification of car: %s",str);
        FEMAreasGenerator::removeUUIDFromProcessingList(modifyCOI->uuid);
        return;
    }

    origCar->carClass = modifyCOI->carClass;
    origCar->name = modifyCOI->name;
    origCar->sourceType = modifyCOI->sourceType;
    origCar->status = modifyCOI->status;
    log("******************************************************************@@@@@@@@@@@@@@ The UUID is %d",modifyCOI->uuid);
    log("******************************************************************@@@@@@@@@@@@@@ **The status of the modified car is %d", modifyCOI->status);
    log("******************************************************************!!!!! The confidence of the modified car is %4.3f", modifyCOI->confidence); 

    float changeTheta = modifyCOI->pose.theta - origCar->pose.theta;
    //Change the Functional Area positions
    for(unsigned int ctr = 0;ctr<origCar->functionalAreas.size();ctr++)
    {
        for(unsigned int ctr1 = 0;ctr1<origCar->functionalAreas[ctr]->area.points.size();ctr1++)
        {
            float xPointWRTCenterOfCar = origCar->functionalAreas[ctr]->area.points[ctr1].x - origCar->pose.x;
            float yPointWRTCenterOfCar = origCar->functionalAreas[ctr]->area.points[ctr1].y - origCar->pose.y;
            origCar->functionalAreas[ctr]->area.points[ctr1].x = modifyCOI->pose.x + xPointWRTCenterOfCar*cos(changeTheta) - yPointWRTCenterOfCar*sin(changeTheta);
            origCar->functionalAreas[ctr]->area.points[ctr1].y = modifyCOI->pose.y + xPointWRTCenterOfCar*sin(changeTheta) + yPointWRTCenterOfCar*cos(changeTheta);
        }
    }

   //Change the Bounding box position
    for(unsigned int ctr = 0;ctr<origCar->boundingBox.points.size();ctr++)
    {
            float xPointWRTCenterOfCar = origCar->boundingBox.points[ctr].x - origCar->pose.x;
            float yPointWRTCenterOfCar = origCar->boundingBox.points[ctr].y - origCar->pose.y;
            origCar->boundingBox.points[ctr].x = modifyCOI->pose.x + xPointWRTCenterOfCar*cos(changeTheta) - yPointWRTCenterOfCar*sin(changeTheta);
            origCar->boundingBox.points[ctr].y = modifyCOI->pose.y + xPointWRTCenterOfCar*sin(changeTheta) + yPointWRTCenterOfCar*cos(changeTheta);
    }

    //Change the vantage point poses
    for(unsigned int ctr = 0;ctr<origCar->vantagePoints.size();ctr++)
    {
            float xPointWRTCenterOfCar = origCar->vantagePoints[ctr]->pose.x - origCar->pose.x;
            float yPointWRTCenterOfCar = origCar->vantagePoints[ctr]->pose.y - origCar->pose.y;
            origCar->vantagePoints[ctr]->pose.x = modifyCOI->pose.x + xPointWRTCenterOfCar*cos(changeTheta) - yPointWRTCenterOfCar*sin(changeTheta);
            origCar->vantagePoints[ctr]->pose.y = modifyCOI->pose.y + xPointWRTCenterOfCar*sin(changeTheta) + yPointWRTCenterOfCar*cos(changeTheta);
            origCar->vantagePoints[ctr]->pose.theta = origCar->vantagePoints[ctr]->pose.theta + changeTheta;
    }

    //Change the X and Y Pos of the Car
    origCar->pose.x = modifyCOI->pose.x;
    origCar->pose.y = modifyCOI->pose.y;
    origCar->pose.theta = modifyCOI->pose.theta;

    //Replace the CAST entry
    try{
    overwriteWorkingMemory(WMAOfModification.id,origCar);
    }
    catch(char *str)
    {
        log("Could not overwrite Working Memory for COI with UUID:%d. Error: %s",modifyCOI->uuid,str);
    }

    //Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(modifyCOI->uuid);

        //Put new ROS entry in
    eu_nifti_env_msg_ros::ElementOfInterestMessage tempEOIMsg;
    tempEOIMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_MODIFY;
    tempEOIMsg.header.stamp = ros::Time::now();
    tempEOIMsg.header.frame_id = "/map";
    tempEOIMsg.type = eu_nifti_env::ElementOfInterest::TYPE_CAR;
    tempEOIMsg.car = eu::nifti::ConverterUtil_Mapping::convertCarOfInterestToROS(origCar);
    tempEOIMsg.castWorkingMemoryPointer.address.id = it->second;
    tempEOIMsg.castWorkingMemoryPointer.address.subarchitecture = getSubarchitectureID();
    tempEOIMsg.castWorkingMemoryPointer.type = "VictimSearch";
    this->eoiDrawCarFunctionalAreasPublisher.publish(tempEOIMsg);
    ros::spinOnce();
    return;
}

void FEMAreasGenerator::modifySignObjectOfInterest(eu::nifti::env::SignObjectOfInterestPtr modifySOI)
{
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(modifySOI->uuid);

    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(modifySOI->uuid);
    if (it == this->mapUUIDToWMAForDetectedObjects.end())
    {
        debug("Sign Object of Interest with UUID %d does not exist, cannot modify",modifySOI->uuid);
        FEMAreasGenerator::removeUUIDFromProcessingList(modifySOI->uuid);
        return;
    }

    try{
    overwriteWorkingMemory(it->second,modifySOI);
    }
    catch(char *str)
    {
        log("Could not overwrite Working Memory for SOI with UUID:%d. Error: %s",modifySOI->uuid,str);
    }

    //Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(modifySOI->uuid);

    //Put new ROS entry in
    eu_nifti_env_msg_ros::ElementOfInterestMessage tempEOIMsg;
    tempEOIMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_MODIFY;
    tempEOIMsg.header.stamp = ros::Time::now();
    tempEOIMsg.header.frame_id = "/map";
    tempEOIMsg.type = eu_nifti_env::ElementOfInterest::TYPE_SIGN;
    tempEOIMsg.sign = eu::nifti::ConverterUtil_Mapping::convertSignOfInterestToROS(modifySOI);
    tempEOIMsg.sign.object.element.status = modifySOI->status;
    tempEOIMsg.castWorkingMemoryPointer.address.id = it->second;
    tempEOIMsg.castWorkingMemoryPointer.address.subarchitecture = getSubarchitectureID();
    tempEOIMsg.castWorkingMemoryPointer.type = "VictimSearch";
    this->eoiDrawCarFunctionalAreasPublisher.publish(tempEOIMsg);
    ros::spinOnce();

    return;
}

void FEMAreasGenerator::modifyVictimObjectOfInterest(eu::nifti::env::VictimObjectOfInterestPtr modifyVOI)
{
    log("2 VOI modify status %d",modifyVOI->status);
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(modifyVOI->uuid);

    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(modifyVOI->uuid);
    if (it == this->mapUUIDToWMAForDetectedObjects.end())
    {
        debug("Victim Object of Interest with UUID %d does not exist, cannot modify",modifyVOI->uuid);
        FEMAreasGenerator::removeUUIDFromProcessingList(modifyVOI->uuid);
        return;
    }
    try{
    overwriteWorkingMemory(it->second,modifyVOI);
    }
    catch(char *str)
    {
        log("Could not overwrite Working Memory for VOI with UUID:%d. Error: %s",modifyVOI->uuid,str);
    }
    
    //Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(modifyVOI->uuid);

    //Put new ROS entry in
    eu_nifti_env_msg_ros::ElementOfInterestMessage tempEOIMsg;
    tempEOIMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_MODIFY;
    tempEOIMsg.header.stamp = ros::Time::now();
    tempEOIMsg.header.frame_id = "/map";
    tempEOIMsg.type = eu_nifti_env::ElementOfInterest::TYPE_VICTIM;
    tempEOIMsg.victim = eu::nifti::ConverterUtil_Mapping::convertVictimOfInterestToROS(modifyVOI);
    tempEOIMsg.victim.object.element.status = modifyVOI->status;
    tempEOIMsg.castWorkingMemoryPointer.address.id = it->second;
    tempEOIMsg.castWorkingMemoryPointer.address.subarchitecture = getSubarchitectureID();
    tempEOIMsg.castWorkingMemoryPointer.type = "VictimSearch";
    log("3 VOI modify status %d",tempEOIMsg.victim.object.element.status);
    this->eoiDrawCarFunctionalAreasPublisher.publish(tempEOIMsg);
    ros::spinOnce();

    return;
}

void FEMAreasGenerator::deleteCarObjectOfInterest(eu::nifti::env::CarObjectOfInterestPtr deleteCOI)
{
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(deleteCOI->uuid);

    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(deleteCOI->uuid);
    if (it == this->mapUUIDToWMAForDetectedObjects.end())
    {
        debug("Car Object of Interest with UUID %d does not exist, cannot delete",deleteCOI->uuid);
        FEMAreasGenerator::removeUUIDFromProcessingList(deleteCOI->uuid);
        return;
    }
    else
    { 
        try{        
        deleteFromWorkingMemory(it->second);
        }
        catch(char *str)
        {
            log("Could not delete Working Memory for COI with UUID:%d. Error: %s",deleteCOI->uuid,str);
        }
        this->mapUUIDToWMAForDetectedObjects.erase(it);
    }

    //Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(deleteCOI->uuid);

    // Send Remove message in ROS
    eu_nifti_env_msg_ros::ElementOfInterestMessage tempEOIMsg;
    tempEOIMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_REMOVE;
    tempEOIMsg.header.stamp = ros::Time::now();
    tempEOIMsg.header.frame_id = "/map";
    tempEOIMsg.type = eu_nifti_env::ElementOfInterest::TYPE_CAR;
    tempEOIMsg.car = eu::nifti::ConverterUtil_Mapping::convertCarOfInterestToROS(deleteCOI);
    tempEOIMsg.castWorkingMemoryPointer.address.id = it->second;
    tempEOIMsg.castWorkingMemoryPointer.address.subarchitecture = getSubarchitectureID();
    tempEOIMsg.castWorkingMemoryPointer.type = "VictimSearch";
    this->eoiDrawCarFunctionalAreasPublisher.publish(tempEOIMsg);
    ros::spinOnce();

    return;
}

void FEMAreasGenerator::deleteSignObjectOfInterest(eu::nifti::env::SignObjectOfInterestPtr deleteSOI)
{
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(deleteSOI->uuid);

    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(deleteSOI->uuid);
    if (it == this->mapUUIDToWMAForDetectedObjects.end())
    {
        debug("Sign Object of Interest with UUID %d does not exist, cannot delete",deleteSOI->uuid);
        FEMAreasGenerator::removeUUIDFromProcessingList(deleteSOI->uuid);
        return;
    }
    else
    {
        try{
            deleteFromWorkingMemory(it->second);
        }
        catch(char *str)
        {
            log("Could not delete Working Memory for SOI with UUID:%d. Error: %s",deleteSOI->uuid,str);
        }
        this->mapUUIDToWMAForDetectedObjects.erase(it);
    }

    //Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(deleteSOI->uuid);

    // Send Remove message in ROS
    eu_nifti_env_msg_ros::ElementOfInterestMessage tempEOIMsg;
    tempEOIMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_REMOVE;
    tempEOIMsg.header.stamp = ros::Time::now();
    tempEOIMsg.header.frame_id = "/map";
    tempEOIMsg.type = eu_nifti_env::ElementOfInterest::TYPE_SIGN;
    tempEOIMsg.sign = eu::nifti::ConverterUtil_Mapping::convertSignOfInterestToROS(deleteSOI);
    tempEOIMsg.castWorkingMemoryPointer.address.id = it->second;
    tempEOIMsg.castWorkingMemoryPointer.address.subarchitecture = getSubarchitectureID();
    tempEOIMsg.castWorkingMemoryPointer.type = "VictimSearch";
    this->eoiDrawCarFunctionalAreasPublisher.publish(tempEOIMsg);
    ros::spinOnce();

    return;

}

void FEMAreasGenerator::deleteVictimObjectOfInterest(eu::nifti::env::VictimObjectOfInterestPtr deleteVOI)
{
    //Add UUID to processing list
    FEMAreasGenerator::addUUIDToProcessingListWhenFree(deleteVOI->uuid);

    std::map<int, std::string>::iterator it;
    it = this->mapUUIDToWMAForDetectedObjects.find(deleteVOI->uuid);
    if (it == this->mapUUIDToWMAForDetectedObjects.end())
    {
        debug("Victim Object of Interest with UUID %d does not exist, cannot delete",deleteVOI->uuid);
        FEMAreasGenerator::removeUUIDFromProcessingList(deleteVOI->uuid);
        return;
    }
    else
    {
        try{
            deleteFromWorkingMemory(it->second);
        }
        catch(char *str)
        {
            log("Could not delete Working Memory for VOI with UUID:%d. Error: %s",deleteVOI->uuid,str);
        }
        this->mapUUIDToWMAForDetectedObjects.erase(it);
    }

    //Remove UUID from processing list
    FEMAreasGenerator::removeUUIDFromProcessingList(deleteVOI->uuid);

    // Send Remove message in ROS
    eu_nifti_env_msg_ros::ElementOfInterestMessage tempEOIMsg;
    tempEOIMsg.action = eu_nifti_env_msg_ros::ElementOfInterestMessage::TYPE_REMOVE;
    tempEOIMsg.header.stamp = ros::Time::now();
    tempEOIMsg.header.frame_id = "/map";
    tempEOIMsg.type = eu_nifti_env::ElementOfInterest::TYPE_VICTIM;
    tempEOIMsg.victim = eu::nifti::ConverterUtil_Mapping::convertVictimOfInterestToROS(deleteVOI);
    tempEOIMsg.castWorkingMemoryPointer.address.id = it->second;
    tempEOIMsg.castWorkingMemoryPointer.address.subarchitecture = getSubarchitectureID();
    tempEOIMsg.castWorkingMemoryPointer.type = "VictimSearch";
    this->eoiDrawCarFunctionalAreasPublisher.publish(tempEOIMsg);
    ros::spinOnce();

    return;
}

void FEMAreasGenerator::addLocationOfInterest(eu::nifti::env::LocationOfInterestPtr loi)
{
    addToWorkingMemory(newDataID(), loi);
    debug("We have added a location %d", loi->uuid);

    return;
}

//Checks if the message is repeated by comparing pose, orientation and uuid values. Assumes that if these are identical, the message is repeated.
bool FEMAreasGenerator::checkRepetitionOfOOI(const eu_nifti_env::ObjectOfInterest* OOIReceived)
{
    bool isMessageRepeated;
    if(this->firstMessageReceived==false)
    {
        // This is the first message, so just copy the values into lastOOIReceived and exit;
        this->lastOOIReceived.pose.position.x=OOIReceived->pose.position.x;
        this->lastOOIReceived.pose.position.y=OOIReceived->pose.position.y;
        this->lastOOIReceived.pose.position.z=OOIReceived->pose.position.z;
        this->lastOOIReceived.pose.orientation.x=OOIReceived->pose.orientation.x;
        this->lastOOIReceived.pose.orientation.y=OOIReceived->pose.orientation.y;
        this->lastOOIReceived.pose.orientation.z=OOIReceived->pose.orientation.z;
        this->lastOOIReceived.pose.orientation.w=OOIReceived->pose.orientation.w;
        this->lastOOIReceived.element.uuid=OOIReceived->element.uuid;
        this->firstMessageReceived = true;
        isMessageRepeated = false;
    }
    else
    {
         if((this->lastOOIReceived.pose.position.x==OOIReceived->pose.position.x)&&
            (this->lastOOIReceived.pose.position.y==OOIReceived->pose.position.y)&&
            (this->lastOOIReceived.pose.position.z==OOIReceived->pose.position.z)&&
            (this->lastOOIReceived.pose.orientation.x==OOIReceived->pose.orientation.x)&&
            (this->lastOOIReceived.pose.orientation.y==OOIReceived->pose.orientation.y)&&
            (this->lastOOIReceived.pose.orientation.z==OOIReceived->pose.orientation.z)&&
            (this->lastOOIReceived.pose.orientation.w==OOIReceived->pose.orientation.w)&&
            (this->lastOOIReceived.element.uuid==OOIReceived->element.uuid)
            )
         {
            isMessageRepeated = true;
         }
        else
        {
           isMessageRepeated = false;
        }
        this->lastOOIReceived.pose.position.x=OOIReceived->pose.position.x;
        this->lastOOIReceived.pose.position.y=OOIReceived->pose.position.y;
        this->lastOOIReceived.pose.position.z=OOIReceived->pose.position.z;
        this->lastOOIReceived.pose.orientation.x=OOIReceived->pose.orientation.x;
        this->lastOOIReceived.pose.orientation.y=OOIReceived->pose.orientation.y;
        this->lastOOIReceived.pose.orientation.z=OOIReceived->pose.orientation.z;
        this->lastOOIReceived.pose.orientation.w=OOIReceived->pose.orientation.w;
        this->lastOOIReceived.element.uuid=OOIReceived->element.uuid; // copying relevant fields for future checks
    }
    return(isMessageRepeated);
}

// CAST START FUNCTION
void FEMAreasGenerator::start()
{
    char* argv[] = {};
    int argc = sizeof (argv) / sizeof (char *);
    ros::init(argc, argv, "FEMAreasGenerator");

    //Establish connection to HFCServer
    try{
       this->_hfcserver = getIceServer<eu::nifti::mapping::HFCInterface>(this->_hfcservername);
    }
    catch(char *str)
    {
        log("Could not write to server: error %s",str);
    }

    //Register the change filter for planningControl
    std::string src = "ExecutionMonitoring"; //name of the component
    std::string id;
    std::string sa;
    addChangeFilter(cast::createChangeFilter<eu::nifti::Planning::slice::FunctionalMappingAction>(cast::cdl::OVERWRITE,src,id,sa,cast::cdl::ALLSA),new cast::MemberFunctionChangeReceiver<FEMAreasGenerator>(this,&FEMAreasGenerator::acceptPlanningTask)); 
}

// CAST CONFIGURE FUNCTION
void FEMAreasGenerator::configure(const std::map<std::string, std::string> &config)
{
    this->firstMessageReceived = false;
    this->lastActionReceived = -1;
    this->planningControlAction = new eu::nifti::Planning::slice::FunctionalMappingAction();
    this->planningControlAction->op = eu::nifti::Planning::slice::WAIT;
    
    // Getting parameters for HFC Server from CAST file
    std::map<std::string, std::string>::const_iterator it = config.find("--hfcserver-name");
     if (it!=config.end())
     {
       this->_hfcservername = it->second;
     }
     else
     {
         this->_hfcservername = "hfcserver";
     }
     println("We will attempt to connect to %s",this->_hfcservername.c_str());
     //Getting parameters for planningControl from CAST file
     it = config.find("--planningControl");
     if (it!=config.end())
     {
         if((it->second).compare("true")==0) // The case where the string returns "true"
         {
             this->planningControl = true;
         }
         else
         {
             this->planningControl = false;
         }
     }
     else
     {
         this->planningControl = false;
     }
}

// CAST RUN COMPONENT
void FEMAreasGenerator::runComponent()
{
    debug("Starting main loop...");

    ros::NodeHandle subscriberNode; //Node that is used to subscribe to the VisualCarInfo messages
    ros::NodeHandle publisherNode; //Node that is used to publish the Polygonal messages
    ros::Publisher testPolyPublisher;

    this->eoiDrawCarFunctionalAreasPublisher = publisherNode.advertise<eu_nifti_env_msg_ros::ElementOfInterestMessage> ("/eoi/draw", 30);

    ros::Rate loop_rate(1);
    //Checking to see if the visual car information is published yet. If not, the program will wait till it is published
    this->subscriberEOI = subscriberNode.subscribe("/eoi", 50, &FEMAreasGenerator::onElementOfInterestMessageReceived, this);
    while (this->subscriberEOI.getNumPublishers() == 0)
    {
        log("Waiting for EOI Publisher at topic /eoi");
        sleep(2);
        ros::spinOnce();
    }
    debug("Reading information");
    //Subscribing to Visual Object Information
    do
    {
        if(((this->planningControlAction->op==eu::nifti::Planning::slice::START)&&(this->planningControl==true))||(this->planningControl==false)) // Planner gives start command or the flag planningControlAction is false
        { 
            //log("Received a Start command from the planner");
            ros::spinOnce();
            //Publish the polygons in ROS for display
            for(unsigned int ctr=0;ctr<this->eoiDrawCarVisibilityAreasList.size();ctr++)
            {
                //debug("Publishing EOI List ctr = %d",ctr);
            }
            loop_rate.sleep();
        } 
        else if((this->planningControlAction->op==eu::nifti::Planning::slice::END)&&(this->planningControl==true)) // Planner gives End command
        {
            log("Received a Stop command from the planner, Ending operation");
            return;
        }
        else
        {
            //debug("Received a wait command from the planner");
            // Do nothing. The planner gave a wait command.
        } 
    } while (ros::ok());
    return;
}

extern "C"
{

    cast::CASTComponentPtr newComponent()
    {
        return new FEMAreasGenerator();
    }
}

