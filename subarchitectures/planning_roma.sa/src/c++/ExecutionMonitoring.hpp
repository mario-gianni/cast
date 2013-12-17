#ifndef _EXECUTION_MONITORING_HPP_
#define _EXECUTION_MONITORING_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <EmbeddedEclipse.hpp>
#include <planning_roma.hpp>
#include <topograph.hpp>
#include <robot_position.hpp>
#include <gap.hpp>
#include <eu/nifti/env/CarObjectOfInterest.hpp>
#include <module_control_action/ControlAction.h>
#include <boost/thread.hpp>
#include <PlanningUtils.hpp>
#include <Parser.hpp>
#include <are.hpp>
#include <diagnostic.hpp>

#include <pthread.h>


#define PI 3.14159265

using namespace eu::nifti::Planning::slice;
using namespace eu::nifti::env::topograph;
using namespace cast;
using namespace cast::cdl;
using namespace eu::nifti::env::terrain;
using namespace eu::nifti::env::are;
using namespace eu::nifti::env::diagnostic;


class ExecutionMonitoring : public ManagedComponent
{
    public:
		boost::thread m_Thread;
		boost::thread g_Thread;

		boost::thread t_Thread;

		//pthread_t plan_thread;

        EC_word myTask;
        std::string id_task;      
		TaskPtr task;
		
		ActionPtr current_action;
		
		//FailurePlanActionPtr interrupt;

		EC_word _plan;
		std::string id_plan;
		PlanPtr myPlan;
		
		EC_word bag;
		
		int mixed_status;
		bool task_aborted;
		EC_word mixedInitiative; //[2 task()]
		//EC_word update_status; //[list of properties]
		
		EC_word current_position;
		eu::nifti::env::position::CurrentPosPtr position;
		eu::nifti::env::position::BasePosPtr base;

		eu::nifti::env::position::PosePtr current_pose;
		
		eu::nifti::env::CarObjectOfInterestPtr object;

		GapPtr _gap;

		ArtefactPtr artefact;
		BatteryStatusPtr battery_status;
		WiFiStatusPtr wifi_status;

		EmbeddedEclipse engine;
		PlanningUtils utils;
		Parser parser;
		
		//bool exe_monitor;

		ros::Publisher actions_pub;
		
		GraphPtr topo_graph;
		
		void performAction(ActionPtr);

    protected:
        virtual void start();
        virtual void runComponent();
        
        void readTask(const WorkingMemoryChange&);
        void myreadTask(const WorkingMemoryChange&);
        
        void readPlan(const WorkingMemoryChange&);
        void myReadPlan(const WorkingMemoryChange&);
        
        //void readNodes(const WorkingMemoryChange&);
		//void readEdges(const WorkingMemoryChange&);
		void readGraph(const WorkingMemoryChange&);
		void myReadGraph(const WorkingMemoryChange&);
		
		void readCurrentPosition(const WorkingMemoryChange&);
		void readBaseStation(const WorkingMemoryChange&);
		void readCurrentPose(const WorkingMemoryChange&);
		
		void readCarObjectOfInterest(const WorkingMemoryChange&);
		
		void currentActionReceived(const WorkingMemoryChange&);

		void readDetectedGap(const WorkingMemoryChange&);

		void readArtefact(const WorkingMemoryChange&);
		void readBatteryStatus(const WorkingMemoryChange&);
		void readWifiStatus(const WorkingMemoryChange&);

		//void currentActionFailureNotificationReceived(const WorkingMemoryChange&);
        
   private:
        
        bool waitForActionExecution();
        //bool waitForNotificationActionFailure();
        
		void computeOrientation(TimelinePtr);
		TimelinePtr computeOrientation2(TimelinePtr);
		NodePtr neighborhood(NodePtr,GraphPtr);
		//double calculatePathCost(NodePtr,eu::nifti::env::VantagePointPtr);
		double calculatePathCost2(NodePtr,eu::nifti::env::VantagePointPtr);
		eu::nifti::env::VantagePointPtr bestVantagePoint(NodePtr,eu::nifti::env::ListOfVantagePoints);
		
	 	EC_word gotoNodeAction(GoToNodeActionPtr);

	 	EC_word updateRobotStatus();

	 	int getIdNode(double,double);

};

#endif
