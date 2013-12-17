// Benoit 2011-04-13

#include <eu/nifti/ros/Time.ice>

module eu 
{
    module nifti 
    {
        module ros
        {

            class Header
            {
                long seq;
                Time stamp;
                string frameID;

            };
        };

    };
};

