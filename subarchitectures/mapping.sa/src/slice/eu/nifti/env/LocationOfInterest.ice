// Benoit 2011-04-14

#ifndef EU_NIFTI_ENV_LOCATION_OF_INTEREST_ICE
#define EU_NIFTI_ENV_LOCATION_OF_INTEREST_ICE

#include <eu/nifti/env/ElementOfInterest.ice>
#include <eu/nifti/env/Point3D.ice>

module eu 
{
    module nifti 
    {

        module env
        { 

            // Represents a location in the environment.
            class LocationOfInterest extends ElementOfInterest
            {
                eu::nifti::env::Point3D point;
            };

        };
    };
};

#endif
