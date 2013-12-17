// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_AREA_OF_INTEREST_ICE
#define EU_NIFTI_ENV_AREA_OF_INTEREST_ICE

#include <eu/nifti/env/Polygon.ice>
#include <eu/nifti/env/ElementOfInterest.ice>

module eu 
{
    module nifti 
    {

        module env
        { 

            // Represents an area in the environment.
            class AreaOfInterest extends ElementOfInterest
            {
                eu::nifti::env::Polygon polygon;
            };

        };
    };
};

#endif
