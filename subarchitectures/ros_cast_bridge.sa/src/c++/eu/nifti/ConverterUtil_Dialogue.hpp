// Benoit & Shanker 2011-07-12

#ifndef EU_NIFTI_CONVERTER_UTIL_DIALOGUE_HPP_
#define EU_NIFTI_CONVERTER_UTIL_DIALOGUE_HPP_

#include <eu_nifti_ocu_msg_ros/DialogueUtteranceMessage.h>
#include <eu/nifti/ocu/msg/cast/DialogueUtteranceMessage.hpp> 

#include "eu/nifti/ConverterUtil.hpp"


namespace eu
{
    namespace nifti
    {

        class ConverterUtil_Dialogue
        {
        public:

            static inline eu::nifti::ocu::msg::cast::DialogueUtteranceMessagePtr convertDialogueUtteranceMessageToCAST(const eu_nifti_ocu_msg_ros::DialogueUtteranceMessage::ConstPtr& msg)
            {
                return new eu::nifti::ocu::msg::cast::DialogueUtteranceMessage(
                        ConverterUtil::convertHeaderToCAST(msg->header),
                        msg->userID,
                        msg->utterance
                        );
            }

            static inline eu_nifti_ocu_msg_ros::DialogueUtteranceMessagePtr convertDialogueUtteranceMessageToROS(const eu::nifti::ocu::msg::cast::DialogueUtteranceMessagePtr msg)
            {
                eu_nifti_ocu_msg_ros::DialogueUtteranceMessage* newMsg = new eu_nifti_ocu_msg_ros::DialogueUtteranceMessage();
                newMsg->header = ConverterUtil::convertHeaderToROS(msg->header);
                newMsg->userID = msg->userID;
                newMsg->utterance = msg->utterance;
                return eu_nifti_ocu_msg_ros::DialogueUtteranceMessagePtr(newMsg);
            }
            
        };

    };

};

#endif // EU_NIFTI_CONVERTER_UTIL_DIALOGUE_HPP_

