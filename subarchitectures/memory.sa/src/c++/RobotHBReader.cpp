#include "RobotHBReader.hpp"

#include <iostream>
#include <sstream>

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
		return new RobotHBReader();
	}
}	                  
			                       
void RobotHBReader::start()
{
    char* argv[] = {};
    int argc = sizeof(argv)/sizeof(char *);
    ros::init(argc, argv, "nifti_memory_robothbreader");

    hb_wma.id = newDataID();
    hb_wma.subarchitecture = getSubarchitectureID();

    println("started ros-cast robot hb reader");
}

void RobotHBReader::runComponent()
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/Pose", 1, &RobotHBReader::rosPoseRead, this);
    ros::Rate r(0.2); // 1 hz
    while (true)
    {
        //println("going to spin once");
        ros::spinOnce();
        r.sleep();
    }
}

void RobotHBReader::rosPoseRead(const geometry_msgs::PosePtr& msg)
{
    //println("got heartbeat, going to store");

    hb = new de::dfki::lt::tr::memory::slice::ROSHeartBeat();
    hb->pose = new de::dfki::lt::tr::memory::slice::ROSPose();
    hb->pose->position = new de::dfki::lt::tr::memory::slice::ROSPointPosition();
    hb->pose->position->x = msg->position.x;
    hb->pose->position->y = msg->position.y;
    hb->pose->position->z = msg->position.z;
    hb->pose->orientation = new de::dfki::lt::tr::memory::slice::ROSQuaternionOrientation();
    hb->pose->orientation->x = msg->orientation.x;
    hb->pose->orientation->y = msg->orientation.y;
    hb->pose->orientation->z = msg->orientation.z;
    hb->pose->orientation->w = msg->orientation.w;

    ros::Time t = ros::Time::now();
    //hb->rosTimeSec = t.toSec();
    hb->rosTimeNSec = t.toNSec();

    std::stringstream ss (std::stringstream::in | std::stringstream::out);
    ss << "going to add heartbeat: (time: ((unsigned!)nsec=" << (unsigned long)hb->rosTimeNSec
        << "), pose: (" << "position: (x=" << hb->pose->position->x << ", y=" << hb->pose->position->y
        << ", z=" << hb->pose->position->z << "), orientation: (x=" << hb->pose->orientation->x
        << ", y=" << hb->pose->orientation->y << ", z=" << hb->pose->orientation->z
        << ", w=" << hb->pose->orientation->w << ")))";
    println(ss.str());
    
    if(!haveWrittenAlready)
    {
	//println("going to write to working memory: first time");
        addToWorkingMemory(hb_wma, hb);
        haveWrittenAlready = true;
        //println("heartbeat added");
    }
    else
    {
	//println("going to write to working memory");
        overwriteWorkingMemory(hb_wma, hb);
        //println("heartbeat overwritten");
    }
}


