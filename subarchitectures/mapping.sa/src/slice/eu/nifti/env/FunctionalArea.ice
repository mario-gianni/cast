// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_FUNCTIONAL_AREA_ICE
#define EU_NIFTI_ENV_FUNCTIONAL_AREA_ICE

#include <eu/nifti/env/Polygon.ice>

module eu 
{
    module nifti 
    {

        module env
        { 

            // Represents a functional area, which is a polygon with a certain meaning
            class FunctionalArea
            {
                string function;
                eu::nifti::env::Polygon area;
            };

            sequence<FunctionalArea> ListOfFunctionalAreas;

        };
    };
};

#endif
