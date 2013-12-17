#include "PlanningBridge.hpp"

using namespace std;
using namespace eu::nifti::planning::slice;

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
		return new PlanningBridge();
	}
}	                  
			                       
void PlanningBridge::start() 
{
	char* argv[] = {};
	int argc = sizeof(argv)/sizeof(char *);
	ros::init(argc, argv, "cast_planning_client");

	//ros::NodeHandle n;
	//client = n.serviceClient<eclipse_prolog_msgs::Task>("execution_monitoring");

	addChangeFilter(cast::createLocalTypeFilter<eu::nifti::planning::slice::PlanningTask>(cast::cdl::ADD),
		new cast::MemberFunctionChangeReceiver<PlanningBridge>(this, &PlanningBridge::planningTaskReceived));

	println("planning bridge (with cast_planning_client) started");
	sendPlan = false;
}

void PlanningBridge::planningTaskReceived(const cast::cdl::WorkingMemoryChange& wmc)
{
	println("new planning task received");

	//first send stop signal to move base client
	ActionPtr navaction = new Action(0, "stop", vector<ArgumentPtr>(), PENDING);
        addToWorkingMemory(newDataID(), "navigation", navaction);
	
	mut.lock();
	//get the plannning task from working memory
	currentPlanningTask = getMemoryEntry<PlanningTask>(wmc.address);
	if(currentPlanningTask->goal != "stop") 
	{
		sendPlan = true;
	} 
	else 
	{
		sendPlan = false;
	}
	mut.unlock();
}

void PlanningBridge::runComponent() 
{
	while(true)
	{
		if(sendPlan)
		{
			ros::NodeHandle n;
			client = n.serviceClient<eclipse_prolog_msgs::Task>("execution_monitoring");
			eclipse_prolog_msgs::Task srv;
			
			mut.lock();
			srv.request.id = currentPlanningTask->id;
			srv.request.task_name = currentPlanningTask->goal;
			sendPlan=false;
			mut.unlock();

			println("calling for " + srv.request.task_name);

			if(client.call(srv))
			{
				cout << srv.response.feedback << endl;
			}
			else
			{
				ROS_ERROR("failed to call service");
			}
		}
		else
		{
			sleep(1);
		}
	}
	return;
}

