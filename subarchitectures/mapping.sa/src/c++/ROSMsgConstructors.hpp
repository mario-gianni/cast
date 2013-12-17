/* 
 * File:   ROSMsgConstructors.hpp
 * Author: shanker
 *
 * Created on October 12, 2011, 4:37 PM
 */

#ifndef ROSMSGCONSTRUCTORS_HPP
#define	ROSMSGCONSTRUCTORS_HPP

#include "FEMAreasGenerator.hpp"

geometry_msgs::Point FEMAreasGenerator::constructPoint(float x, float y, float z)
{
    geometry_msgs::Point resPoint;
    resPoint.x = x;
    resPoint.y = y;
    resPoint.z = z;
    return(resPoint);
}

geometry_msgs::Vector3 FEMAreasGenerator::constructVector3(float x, float y, float z)
{
    geometry_msgs::Vector3 resVector3;
    resVector3.x = x;
    resVector3.y = y;
    resVector3.z = z;
    return(resVector3);
}



#endif	/* ROSMSGCONSTRUCTORS_HPP */

