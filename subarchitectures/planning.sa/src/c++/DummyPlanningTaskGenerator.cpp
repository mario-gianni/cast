#include "DummyPlanningTaskGenerator.hpp"

extern "C"
{
	cast::CASTComponentPtr newComponent() 
	{
		return new DummyPlanningTaskGenerator();
	}
}	                  
			                       
void DummyPlanningTaskGenerator::start() 
{
	println("planning bridge (with cast_planning_client) started");
}

void DummyPlanningTaskGenerator::runComponent() 
{
	println("running DummyPlanningTaskGenerator component ");
	
	println("going to publish task in 5 seconds");
	sleep(5);
	PlanningTaskPtr planningTask1 = new PlanningTask(0, "visit_graph", vector<ActionPtr>(), "", PENDING, 0, PENDING, 0);
        addToWorkingMemory(newDataID(), "planning", planningTask1);

	println("going to publish next task in 15 seconds");
	sleep(15);
	PlanningTaskPtr planningTask2 = new PlanningTask(0, "visit_graph", vector<ActionPtr>(), "", PENDING, 0, PENDING, 0);
        addToWorkingMemory(newDataID(), "planning", planningTask2);
	
	println("going to stop in 30 seconds");
	sleep(30);
	PlanningTaskPtr planningTask3 = new PlanningTask(0, "stop", vector<ActionPtr>(), "", PENDING, 0, PENDING, 0);
        addToWorkingMemory(newDataID(), "planning", planningTask3);

	return;
}

