/*
 * PlanningUtils.cpp
 *
 *  Created on: Dec 29, 2011
 *      Author: alcor
 */

#include <string>
#include <fstream>
#include <sstream>
#include <PlanningUtils.hpp>
#include "tf/transform_listener.h"
#include <geometry_msgs/Quaternion.h>

PlanningUtils::PlanningUtils()
{
}

PlanningUtils::~PlanningUtils()
{
}

void PlanningUtils::storeArtefact(ArtefactPtr obj, std::string filename)
{
	std::ofstream myfile;
	myfile.open(filename.c_str(),std::ios_base::app);
	std::string s = "artefact(";
	if(myfile.is_open())
	{

		std::string label = obj->label;
		std::string x = to_string(obj->mapPose.pos.x);
		std::string y = to_string(obj->mapPose.pos.y);
		std::string z = to_string(obj->mapPose.pos.z);

		geometry_msgs::Quaternion q;
		q.x = obj->mapPose.orient.x;
		q.y = obj->mapPose.orient.y;
		q.z = obj->mapPose.orient.z;
		q.w = obj->mapPose.orient.w;
		double yaw = tf::getYaw(q);
		std::string theta = to_string(yaw);

		std::string bx = to_string(obj->bbox.x);
		std::string by = to_string(obj->bbox.y);
		std::string bz = to_string(obj->bbox.z);

		std::string doc = to_string(obj->confidenceDegree);
		std::string timestamp = to_string(obj->timestamp);

		std::string type;

		switch(obj->type)
		{
		case eu::nifti::env::are::CAR:
			type = "car";
			break;
		case eu::nifti::env::are::MALE:
			type = "male";
			break;
		case eu::nifti::env::are::FEMALE:
			type = "female";
			break;
		case eu::nifti::env::are::ROBOT:
			type = "robot";
			break;
		default:
			std::cout<<"****** ERROR - Artefact Type not found *********"<<std::endl;
			break;
		}

		std::string result = s + timestamp + "," + type + "," + label + "," + x + "," + y + "," + z + "," + theta + "," + bx + "," + by + "," + bz + "," + doc + ").";
		myfile << result << std::endl;
		myfile.close();
	}
	else
	{
		std:: cout << "storeArtefact: Unable to open file" << std::endl;
	}

}

/* Functions to write into detected_object.ecl file the detected car which has been red from the working memory */
void PlanningUtils::storeDetectedObject(CarObjectOfInterestPtr object, std::string filename)
{
    /* filename: detected_object.ecl
    *  format of the string: detected(car1,X,Y,Z,Theta)
    */
  	std::ofstream myfile;
	myfile.open(filename.c_str(),std::ios_base::app);
	std::string s = "detected(";
	if(myfile.is_open())
	{
	    std::string object_label = object->name + to_string(object->uuid);
	    std::string x = to_string(object->pose.x);
	    std::string y = to_string(object->pose.y);
	    std::string z = to_string(object->pose.z);
	    std::string theta = to_string(object->pose.theta);
	    std::string result = s + object_label + "," + x + "," + y + "," + z + "," + theta + ").";
	    myfile << result << std::endl;
	    myfile.close();
	}
	else
	{
	    std:: cout << "storeDetectedObject: Unable to open file" << std::endl;
	}
}

/* Functions to write into detected_object.ecl file the vantage points around the detected car which have been red from the working memory */
void PlanningUtils::storeVantagePoints(CarObjectOfInterestPtr object, std::string filename)
{
    /* filename: detected_object.ecl
    *  format of the string: vp(car1,vpnode(vp1,X,Y,Z,Theta,Gain,Flag))
    */
    std::ofstream myfile;
	myfile.open(filename.c_str(),std::ios_base::app);
	std::string s1 = "vp(";
	std::string object_label = object->name + to_string(object->uuid);
	std::string s2 = "vpnode(";
	if(myfile.is_open())
	{

        for(unsigned int i = 0; i < object->vantagePoints.size(); i++)
        {
            std::string temp = "vp" + to_string(i);
            std::string x = to_string(object->vantagePoints[i]->pose.x);
            std::string y = to_string(object->vantagePoints[i]->pose.y);
            std::string z = to_string(object->vantagePoints[i]->pose.z);
            std::string theta = to_string(object->vantagePoints[i]->pose.theta);
            std::string gain = to_string(object->vantagePoints[i]->gain);
            std::string flag = to_string(0);
            std::string result = s1 + object_label + "," + s2 + temp + "," + x + "," + y + "," + z + "," + theta + "," + gain + "," + flag + ")).";
            myfile << result << std::endl;
        }
	    myfile.close();
	}
	else
	{
	    std:: cout << "storeVantagePoints: Unable to open file" <<std::endl;
	}
}

/* Functions to write into nodes.ecl file the nodes of the topological segmentation which have been red from the working memory */
void PlanningUtils::storeNodes(NodePtr node, std::string filename)
{
    /* filename: nodes.ecl
    *  format of the string: graph(node(n1,X,Y,Flag))
    */
	std::ofstream myfile;
	myfile.open(filename.c_str(),std::ios_base::app);
	std::string _node = "graph(";

	if(myfile.is_open())
	{
		std::string s;
		s = _node + to_string(node) + ").";
		myfile << s << std::endl;
		myfile.close();

	}
	else
	{
		std:: cout << "storeNodes: Unable to open file" <<std::endl;
	}
}

/* Functions to write into edges.ecl file the edges of the topological segmentation which have been red from the working memory */
void PlanningUtils::storeEdges(EdgePtr edge, std::string filename)
{
    /* filename: edges.ecl
    *  format of the string: edge(edge(node(),node(),1))
    */
	std::ofstream myfile;
	myfile.open(filename.c_str(),std::ios_base::app);
	if(myfile.is_open())
	{
		myfile << to_string(edge).c_str() << std::endl;
		myfile.close();

	}
	else
	{
		std:: cout << "storeEdges: Unable to open file" <<std::endl;
	}
}

/* Functions to write into base_station.ecl file the starting region of the robot which has been red from the working memory */
void PlanningUtils::storeBaseStation(BasePosPtr pos,std::string filename)
{
    /* filename: base_station.ecl
    *  format of the string: base(navigation,node(),[])
    */
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if(myfile.is_open())
	{
        std::string position = "base(navigation," + to_string(pos->node) + ",[]).";
	    myfile << position << std::endl;
		myfile.close();

	}
	else
	{
		std:: cout << "storeBaseStation: Unable to open file" <<std::endl;
	}
}

void PlanningUtils::storeBatteryStatus(BatteryStatusPtr status, std::string filename)
{
	std::ofstream myfile;
    myfile.open(filename.c_str());
    if(myfile.is_open())
    {
    	std::string battery = "holds(diagnostic,battery(";
    	std::string level;

    	switch(status->level)
    	{
    	case eu::nifti::env::diagnostic::BHIGH:
    		level = "high),";
    		break;
    	case eu::nifti::env::diagnostic::BMEDIUM:
    		level = "medium),";
    		break;
    	case eu::nifti::env::diagnostic::BLOW:
    		level = "low),";
    		break;
    	default:
    		std::cout<<"****** ERROR - Battery status not found *********"<<std::endl;
    		break;
    	}
    	std::string result = battery + level + "[]).";
    	myfile << result << std::endl;
    	myfile.close();

    }
    else
    {
    	std:: cout << "storeBatteryStatus: Unable to open file" <<std::endl;
    }
}

void PlanningUtils::storeWifiStatus(WiFiStatusPtr status, std::string filename)
{
	std::ofstream myfile;
    myfile.open(filename.c_str());
    if(myfile.is_open())
    {
    	std::string wifi = "holds(diagnostic,wifi(";
    	std::string level;

    	switch(status->quality)
    	{
    	case eu::nifti::env::diagnostic::GOOD:
    		level = "good),";
    		break;
    	case eu::nifti::env::diagnostic::MODERATE:
    		level = "moderate),";    		break;
    	case eu::nifti::env::diagnostic::WEAK:
    		level = "weak),";    		break;
    	case eu::nifti::env::diagnostic::LOST:
    		level = "lost),";    		break;
    	default:
    		std::cout<<"****** ERROR - Wifi status not found *********"<<std::endl;
    		break;
    	}

    	std::string result = wifi + level + "[]).";
    	myfile << result << std::endl;
    	myfile.close();

    }
    else
    {
    	std:: cout << "storeWifiStatus: Unable to open file" <<std::endl;
    }
}

void PlanningUtils::storeNodeWifiStrenght(CurrentPosPtr position, WiFiStatusPtr status, std::string filename)
{
	std::ofstream myfile;
	myfile.open(filename.c_str());
	if(myfile.is_open())
	{
		std::string wifi_strength = "wifi_strenght(" + position->node->label + ",";

		std::string level;

		switch(status->quality)
		{
		case eu::nifti::env::diagnostic::GOOD:
			level = "good).";
			break;
		case eu::nifti::env::diagnostic::MODERATE:
			level = "moderate).";    		break;
		case eu::nifti::env::diagnostic::WEAK:
			level = "weak).";    		break;
		case eu::nifti::env::diagnostic::LOST:
			level = "lost).";    		break;
		default:
			std::cout<<"****** ERROR - Wifi status not found *********"<<std::endl;
			break;
		}

		std::string result = wifi_strength + level;
		myfile << result << std::endl;
		myfile.close();

	}
	else
	{
		std:: cout << "storeNodeWifiStrenght: Unable to open file" <<std::endl;
	}
}

std::string PlanningUtils::to_string(eu::nifti::env::position::PosePtr pose)
{
    std::string result;
    result = "holds(navigation,at(";
    result = result + to_string(pose->x) + ",";
    result = result + to_string(pose->y) + ",";
    result = result + to_string(pose->z) + ",";
    result = result + to_string(pose->theta) + "[])";
    return result;
}


std::string PlanningUtils::to_string(ActionPtr action)
{
    std::string result;

    if(GoToNodeActionPtr goto_action = GoToNodeActionPtr::dynamicCast(action))
	{
	    result = result + action->name + "(";
	    result = result + to_string(goto_action->node) + ",";
	    result = result + to_string(goto_action->theta) + ",";
	    result = result + to_string(action->time) + ")";
	}
	else if(TopoGraphWriterActionPtr topo_writer_action = TopoGraphWriterActionPtr::dynamicCast(action))
	{
	    if(topo_writer_action->op == START)
	    {
	        result = "start_read_topo(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(topo_writer_action->op == END)
	    {
	        result = "end_read_topo(";
	        result = result + to_string(action->time) + ")";
	    }
	    else
	    {
	        std::cout << "to_string-> Unknown action" << std::endl;
	    }
	}
	else if(TogoGraphBuilderActionPtr topo_builder_action = TogoGraphBuilderActionPtr::dynamicCast(action))
	{
	    if(topo_builder_action->op == START)
	    {
	        result = "start_build_topo(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(topo_builder_action->op == END)
	    {
	        result = "end_build_topo(";
	        result = result + to_string(action->time) + ")";
	    }
	    else
	    {
	        std::cout << "to_string-> Unknown action" << std::endl;
	    }
	}
	else if(FunctionalMappingActionPtr func_map_action = FunctionalMappingActionPtr::dynamicCast(action))
	{
	    if(func_map_action->op == START)
	    {
	        result = "start_func_map(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(func_map_action->op == END)
	    {
	        result = "end_func_map(";
	        result = result + to_string(action->time) + ")";
	    }
	    else
	    {
	        std::cout << "to_string-> Unknown action" << std::endl;
	    }
	}
	else if(RotatingLaserActionPtr rot_laser_action = RotatingLaserActionPtr::dynamicCast(action))
	{
	    result = "start_rotation(";
	    result = result + to_string(rot_laser_action->speed) + ",";
	    result = result + to_string(action->time) + ")";
	}
	else if(CenterLaserActionPtr cen_laser_action = CenterLaserActionPtr::dynamicCast(action))
	{
	    result = "end_rotation(";
	    result = result + to_string(action->time) + ")";
	}
	else if(DifferentialActionPtr diff_action = DifferentialActionPtr::dynamicCast(action))
	{
	    if(diff_action->flag == ON)
	    {
	        result = "lock(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(diff_action->flag == OFF)
	    {
	        result = "unlock(";
	        result = result + to_string(action->time) + ")";
	    }
	    else
	    {
	        std::cout << "to_string-> Unknown action" << std::endl;
	    }
	}
	else if(GapDetectionActionPtr gap_action = GapDetectionActionPtr::dynamicCast(action))
	{
	    if(gap_action->op == START)
	    {
	        result = "start_gap_detection(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(gap_action->op == END)
	    {
	        result = "end_gap_detection(";
	        result = result + to_string(action->time) + ")";
	    }
	    else
	    {
	        std::cout << "to_string-> Unknown action" << std::endl;
	    }
	}
	else if(FlipperActionPtr flipper_action = FlipperActionPtr::dynamicCast(action))
	{
	    result = "move(";
	    result = result + flipper_action->component + ",";
	    result = result + to_string(flipper_action->alfa) + ",";
	    result = result + to_string(action->time) + ")";
	}
	else if(MoveBaseActionPtr motion_action = MoveBaseActionPtr::dynamicCast(action))
	{
	    if(motion_action->command == MOVEFORWARD)
	    {
	        result = "move_forward(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(motion_action->command == MOVELEFT)
	    {
	        result = "move_left(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(motion_action->command == MOVERIGHT)
	    {
	        result = "move_right(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(motion_action->command == MOVEBACK)
	    {
	        result = "move_back(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(motion_action->command == TURNLEFT)
	    {
	        result = "turn_left(";
	        result = result + to_string(action->time) + ")";
	    }
	    else if(motion_action->command == TURNRIGHT)
	    {
	        result = "turn_right(";
	        result = result + to_string(action->time) + ")";
	    }
	    else
	    {
	        std::cout << "to_string-> Unknown action" << std::endl;
	    }
	}
	else
	{
        std::cout << "to_string-> Action Unknown" << std::endl;
	}

    return result;
}

std::string PlanningUtils::to_string(double value)
{
	std::stringstream ss;
	ss << value;
	return ss.str();
}

std::string PlanningUtils::to_string(NodePtr n)
{
	std::string node = "node(" + n->label + ",";
	std::stringstream _x;
	_x << n->x;
	node = node + _x.str() + ",";
	std::stringstream _y;
	_y << n->y;
	node = node + _y.str() + ",";
	std::stringstream _flag;
	_flag << n->flag;
	node = node + _flag.str() + ")";
	return node;
}

std::string PlanningUtils::to_string(EdgePtr e)
{
	std::string s;
	std::string _edge = "edge(";
	s = _edge + "edge(" + to_string(e->a) + "," + to_string(e->b) + "," + "1)" + ").";
	return s;
}



