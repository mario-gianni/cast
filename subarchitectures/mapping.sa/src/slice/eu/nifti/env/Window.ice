// Benoit 2011-04-27

#ifndef EU_NIFTI_ENV_WINDOW_ICE
#define EU_NIFTI_ENV_WINDOW_ICE

#include <eu/nifti/env/Pose.ice>

module eu 
{
    module nifti 
    {

        module env
        { 

            // Represents the position of a window of a car
            struct Window
            {
                double width;
		double height;
                eu::nifti::env::Pose pose;
            };

            sequence<Window> ListOfWindows;

        };
    };
};

#endif
