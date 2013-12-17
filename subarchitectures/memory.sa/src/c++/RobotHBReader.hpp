#ifndef ROS_HB_READER_HPP_
#define ROS_HB_READER_HPP_
      
#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include "autogen/memory.hpp"
#include <geometry_msgs/Pose.h>
			                 
class RobotHBReader: public cast::ManagedComponent
{				 
	public:
        
	virtual void start();
                  
	protected:
	virtual void runComponent();
	virtual void rosPoseRead(const geometry_msgs::PosePtr&);
         
	private:
        bool haveWrittenAlready;
        cast::cdl::WorkingMemoryAddress hb_wma;
        de::dfki::lt::tr::memory::slice::ROSHeartBeatPtr hb;
};     
      
#endif
