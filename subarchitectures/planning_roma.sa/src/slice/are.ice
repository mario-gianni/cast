#ifndef _ARE_ICE
#define _ARE_ICE

#include <cast/slice/CDL.ice>

module eu{
	module nifti {
		module env {
			module are {
			
				enum ArtefactType {CAR, MALE, FEMALE, ROBOT};
				
				struct Position
                {
                    double x;
                    double y;
		            double z;
                }; 
                
                struct Orientation
                {
                    double x;
                    double y;
                    double z;
                    double w;
                };
                
                struct Pose
                {
                    Position pos;
                    Orientation orient;
                };
                
                struct BoundingBox
                {
                	double x;
                	double y;
                	double z;
                };
                
                class Artefact
                {
                	ArtefactType type;
                	string label;
                	double confidenceDegree;
                	Pose mapPose;
                	BoundingBox bbox;
                	double timestamp;
                };
			};
		};
	};
};

#endif
