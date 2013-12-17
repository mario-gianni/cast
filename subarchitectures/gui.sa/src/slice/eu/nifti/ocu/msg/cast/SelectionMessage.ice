// Benoit 2011-04-12

#ifndef EU_NIFTI_OCU_MSG_CAST_SELECTION_MESSAGE_ICE
#define EU_NIFTI_OCU_MSG_CAST_SELECTION_MESSAGE_ICE

#include <cast/slice/CDL.ice>

#include <eu/nifti/ros/Header.ice>

module eu 
{
    module nifti 
    {

        module ocu 
        { 
    
            module msg
            {

                module cast
                {

                    enum SelectionMessageAction {SELECT, DESELECT};

                    // This message is sent from the OCU to CAST
                    // Tells when an element is selected
                    class SelectionMessage
                    {
                        eu::nifti::ros::Header header;
                        string userID;
                        int elementUUID;
                        SelectionMessageAction action;

                        // This field is there just for convenience, because the uuid is sufficient
                        cast::cdl::WorkingMemoryPointer wmp;
                    };

                };

            };

        };
    };
};

#endif
