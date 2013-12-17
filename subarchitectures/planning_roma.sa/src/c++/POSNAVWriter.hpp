#ifndef _POSNAV_WRITER_HPP_
#define _POSNAV_WRITER_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <voronoiseg/PoseAndTopologicalID.h>
#include <robot_position.hpp>

class POSNAVWriter : public cast::ManagedComponent
{
	public:
	    ros::Subscriber temp;
	    voronoiseg::PoseAndTopologicalID base_station;
		voronoiseg::PoseAndTopologicalID current_position;
		void listen_current_position(const voronoiseg::PoseAndTopologicalID&);
		void listen_base_position(const voronoiseg::PoseAndTopologicalID&);
		
	protected:
		virtual void start();
		virtual void runComponent();
	private:
		std::string to_string(double);
		std::string to_string(eu::nifti::env::topograph::NodePtr);
};

#endif
