// Benoit & Shanker 2011-07-12

#ifndef EU_NIFTI_CONVERTER_UTIL_GUI_HPP_
#define EU_NIFTI_CONVERTER_UTIL_GUI_HPP_

#include <eu_nifti_ocu_msg_ros/SelectionMessage.h>
#include <eu/nifti/ocu/msg/cast/SelectionMessage.hpp> 

#include "eu/nifti/ConverterUtil.hpp"

namespace eu
{
    namespace nifti
    {

        class ConverterUtil_GUI
        {
        public:

            static inline eu::nifti::ocu::msg::cast::SelectionMessagePtr convertSelectionMessageToCAST(const eu_nifti_ocu_msg_ros::SelectionMessage::ConstPtr& msg)
            {
                return new eu::nifti::ocu::msg::cast::SelectionMessage(
                        ConverterUtil::convertHeaderToCAST(msg->header),
                        msg->userID,
                        msg->elementUUID,
                        eu::nifti::ocu::msg::cast::SelectionMessageAction(msg->action),
                        ConverterUtil::convertWorkingMemoryPointerToCAST(msg->castWorkingMemoryPointer)
                        );
            }

        };
      
    };

};

#endif // EU_NIFTI_CONVERTER_UTIL_GUI_HPP_

