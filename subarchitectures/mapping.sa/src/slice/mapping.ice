#ifndef MAPPING_ICE
#define MAPPING_ICE

#include <cast/slice/CDL.ice>

module eu {
module nifti {
module mapping { 
  /*  
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



*/

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
// };
// };


#endif
