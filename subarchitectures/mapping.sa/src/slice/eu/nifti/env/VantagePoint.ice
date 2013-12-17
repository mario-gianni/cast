// Shanker 2011-09-29

#ifndef EU_NIFTI_ENV_VANTAGEPOINT_ICE
#define EU_NIFTI_ENV_VANTAGEPOINT_ICE

#include <eu/nifti/env/Pose.ice>

module eu 
{
    module nifti 
    {

        module env
        { 

            // Represents the poses of the Vantage Points
            class VantagePoint
            {
		float gain;
                eu::nifti::env::Pose pose;
		string function;
            };

            sequence<VantagePoint> ListOfVantagePoints;

        };
    };
};

#endif
