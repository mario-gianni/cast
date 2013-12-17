#ifndef _TOPO_SEG_READER_COMPONENT_HPP_
#define _TOPO_SEG_READER_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <planning_roma.hpp>
#include <geometry_msgs/PolygonStamped.h>
#include <voronoiseg/PoseAndTopologicalID.h>
#include <robot_position.hpp>

using namespace eu::nifti::Planning::slice;
using namespace cast;
using namespace cast::cdl;
using namespace eu::nifti::env::topograph;
using namespace geometry_msgs;
using namespace std;
using namespace eu::nifti::env::position;

class TopoSegReaderComponent : public ManagedComponent
{
    public:

		TopoGraphWriterActionPtr current;
    
        PolygonStamped current_topo_centers;
		PolygonStamped current_conn_graph;
		
        ros::Subscriber sub_topo_centers;
        ros::Subscriber sub_conn_graph;
        ros::Subscriber temp_base;
        
        voronoiseg::PoseAndTopologicalID current_node;
        voronoiseg::PoseAndTopologicalID base_station;

        void listen_current_topo_centers(const PolygonStamped::ConstPtr&);
		void listen_current_conn_graph(const PolygonStamped::ConstPtr&);
		
		void listen_current_node(const voronoiseg::PoseAndTopologicalID&);
		void listen_base_position(const voronoiseg::PoseAndTopologicalID&);

		bool waitForSubscriber();
        
    protected:
        virtual void start();
        virtual void runComponent();
        virtual void readAction(const WorkingMemoryChange&);
        
    private:
        string to_string(double);
		string to_string(NodePtr);
		string to_string(EdgePtr);
		NodePtr getNode(int,GraphPtr);
};

#endif
