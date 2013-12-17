#ifndef _GAP_ICE
#define _GAP_ICE

#include <cast/slice/CDL.ice>

module eu {
    module nifti {
        module env {
            module terrain {
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
                
                struct Configuration
                {
                    double flipperAngleFL;
                    double flipperAngleFR;
                    double flipperAngleRL;
                    double flipperAngleRR;
                    double differential;
                };
                
                struct Pose
                {
                    Position pos;
                    Orientation orient;
                };
            
                class Gap{
                    string id;
                    bool traversability;
                    Pose init;
                    Pose final;
                    Configuration conf;
                };
            };            
        };
    };
};

#endif
