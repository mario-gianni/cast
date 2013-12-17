// Benoit 2011-06-01

#ifndef EU_NIFTI_OCU_MSG_CAST_DIALOGUE_UTTERANCE_MESSAGE_ICE
#define EU_NIFTI_OCU_MSG_CAST_DIALOGUE_UTTERANCE_MESSAGE_ICE

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

                    // DialogueUtteranceMessage: tells when a user or a robot says something as part of a dialogue
                    class DialogueUtteranceMessage
                    {
                        eu::nifti::ros::Header header;
                        string userID;
                        string utterance;
                    };

                };

            };

        };
    };
};

#endif
