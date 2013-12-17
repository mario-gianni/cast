#ifndef PLANNING_ICE
#define PLANNING_ICE

#include <cast/slice/CDL.ice>

module eu {
module nifti {
module planning
{
	module slice
	{
		sequence<string> stringSeq;

		enum Completion
		{
			PENDING,
			INPROGRESS,
			ABORTED,
			FAILED,
			SUCCEEDED
		};

		class Argument
		{
			string featureName;
			cast::cdl::WorkingMemoryPointer featureValueWMP;
		};

		sequence<Argument> ArgumentSeq;
		//sequence<beliefs::autogen::featurecontent::FeatureValue> ArgumentSeq;
		//sequence<de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief> BeliefSeq;

		//class StateChangeFilter 
		//{
			//BeliefSeq removeFilter;
			//stringSeq featureFilter;
		//};

		class Action
		{
			int taskID;
			string name;
			ArgumentSeq arguments;
			Completion status;
		};

		sequence<Action> ActionSeq;

		class PlanningTask
		{
			int id;
			string goal;
			ActionSeq plan;
			string firstActionID;
			//BeliefSeq state;
			Completion executionStatus;
			int executionRetries;
			Completion planningStatus;
			int planningRetries;
		};

		// this is for planning-internal use only and takes care of the communication between
		// (the c++ based) cast and the (python) components.

		//interface CppServer
		//{
		//  void deliverPlan(int id, ActionSeq plan);
		//  //void updateBeliefState(BeliefSeq beliefs);
		//  //void deliverPlan(PlanningTask task);
		//  void updateStatus(int id, Completion status);
		//  //void setChangeFilter(int id, StateChangeFilter filter);
		//};

		//interface PythonServer extends cast::interfaces::CASTComponent
		//{
		//  void registerTask(PlanningTask task);
		//  void updateTask(PlanningTask task);
		//};
	};
};
};
};

#endif
