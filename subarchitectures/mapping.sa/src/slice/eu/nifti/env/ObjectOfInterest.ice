// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_OBJECT_OF_INTEREST_ICE
#define EU_NIFTI_ENV_OBJECT_OF_INTEREST_ICE

#include <eu/nifti/env/ElementOfInterest.ice>
#include <eu/nifti/env/Polygon.ice>
#include <eu/nifti/env/Pose.ice>


module eu 
{
    module nifti 
    {

        module env
        { 

            // Represents an object in the environment.
            class ObjectOfInterest extends ElementOfInterest
            {
                eu::nifti::env::Polygon boundingBox;
                eu::nifti::env::Pose pose;
            };

        };
    };
};

#endif
