#ifndef HWD_ICE
#define HWD_ICE

#include <cast/slice/CDL.ice>

module eu {
module nifti {
module navigation 
{
	module slice
	{
		class TwistMsg
		{
			float linx;
			float liny;
			float linz;
			float angx;
			float angy;
			float angz;
		};

		enum GoalState
		{
			PENDING,     // hasn't started yet
			INPROGRESS,  // started but no plan found yet
			ABORTED,     // aborted (if e.g a more important query occurs)???
			FAILED,      // no plan found
			SUCCEEDED    // plan found
		};
		

		class NavigationGoal
		{
			string goal;
			GoalState status;
		};
	};
};

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

module mapping { 
    
  // simple struct for a point in 3D
  struct Point3D {
    float x;
    float y;
    float z;
  };

  // class for representing any 2D pose
  class Pose {
    float x;
    float y;
    float theta;
  };

  // class that extends Pose to a 3D point
  class Pose3D extends Pose {
    float z;
    // could be enhanced by adding 3D rotation values later on!
  };

  // class for representing dimensions of simple rectangular shapes
  struct Dimensions {
    float width;
    float length;
  };
  
  // sequence of WMPs to WMEs that hold functional areas
  sequence <cast::cdl::WorkingMemoryPointer> FuncAreaWMPs; //Stores the memory addresses (in CAST) of the areas around the car
 
  // struct for holding window information: center pointand width of window opening
  struct Window {
    float xPos;
    float yPos;
    float theta;
    float width;
  };

  // sequence of Window objects
  sequence<Window> Windows;

  // superclass for all kinds of MapObjects
  // add more shared properties if needed
  class MapObject {
    Pose objectPose;
    string label;
  };

  // subclass of MapObject to represent cars	
  class CarObject extends MapObject {
    int          numberOfPolygons; // deprecated, should be avoided
    FuncAreaWMPs functionalAreas;
    Windows      carWindows;
    Dimensions   carDimensions;
    string       carClass;
  };

  // sequence of WMPs to WMEs that hold points
  sequence <Point3D> Points; //A sequence of objects of class PolygonPoint
  
  // individual polygon
  struct Polygon {
    int    numberOfPoints;
    Points polygonPoints;
  };
  
  // struct for functional areas, such as lookinsidability... ;-)
  class FunctionalArea {  // Changed from 'struct' to 'class' so that it could be written to WM
    string  functionType; // for now simply a string, could be refined later on!
    Polygon areaPolygon;
  };

// };
// };
// };

// module de {
// module dfki {
// module lt {
// module tr { 
// modul mapping {

  sequence<string> BindingRow;
  sequence<BindingRow> BindingTable;
  dictionary<string, int> StringToIntMap;

  sequence<string> Tuple;
    
  struct QueryResults {
    string query;
    StringToIntMap varPosMap;
    BindingTable bt;   
  };
    
  interface HFCInterface {
    QueryResults querySelect(string q);
    void addTuple(Tuple t);
    string ping();    	
  };


//    sequence<string> InstanceSet;
//    sequence<string> ConceptSet;
//    sequence<long> PlaceIdSet;
    
//    interface ComaReasonerInterface {
//        string testReverseString(string s);
//    	InstanceSet getAllInstances(string concept);
//    	InstanceSet getRelatedInstances(string instance);
//    	InstanceSet getRelatedInstancesByRelation(string instance, string relation);
//	InstanceSet getAllConcepts(string instance);
//	ConceptSet getAllSubconcepts(string concept);
//	bool addInstance(string instance, string concept);
//	bool addRelation(string instance1, string relation, string instance2);
//	bool deleteInstance(string instance);
//	bool deleteRelation(string instance1, string relation, string instance2);
//	bool isInstanceOf(string instance, string concept);
//	string executeSPARQL(string sparqlQuery);
//    };
    
     
};
};
};

#endif
