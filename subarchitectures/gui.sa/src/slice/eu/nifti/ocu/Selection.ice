// Benoit 2011-04-12

#ifndef EU_NIFTI_OCU_SELECTION_ICE
#define EU_NIFTI_OCU_SELECTION_ICE

#include <cast/slice/CDL.ice>

#include <eu/nifti/ros/Time.ice>

module eu 
{
    module nifti 
    {

        module ocu 
        { 
    
                    // Represents the selection of an element
                    class Selection
                    {
                        string userID;
                        // int elementUUID; Probably not necessary

                        eu::nifti::ros::Time start;
                        eu::nifti::ros::Time end;

                        // In lieu of the UUID
                        cast::cdl::WorkingMemoryPointer element;
                    };


        };
    };
};

#endif
