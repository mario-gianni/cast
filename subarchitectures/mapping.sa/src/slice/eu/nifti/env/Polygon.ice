// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_POLYGON_ICE
#define EU_NIFTI_ENV_POLYGON_ICE

#include <eu/nifti/env/Point3D.ice>

module eu 
{
    module nifti 
    {

        module env
        { 

            // Represents a simple polygon, with no special meaning.
            struct Polygon
            {
                ListOfPoints3D points;
            };

        };
    };
};

#endif
