// Benoit 2011-01-12

#ifndef ROS_CAST_CONVERTER_HPP_
#define ROS_CAST_CONVERTER_HPP_

#include <cast/architecture.hpp>

#include <eu_nifti_ocu_msg_ros/DialogueUtteranceMessage.h>
#include <eu_nifti_ocu_msg_ros/SelectionMessage.h>

namespace eu
{
    namespace nifti
    {

        // This class is an active component that can listen or publish to ROS
        // and CAST, and uses the Util files to do the actual conversions
        class ROS_CAST_Converter : public cast::ManagedComponent
        {
        protected:
            virtual void configure(const cast::cdl::StringMap & _config, const Ice::Current & _current);
            virtual void start();
            virtual void runComponent();
            virtual void stop();

            void onSelectionMessageReceived(const eu_nifti_ocu_msg_ros::SelectionMessage::ConstPtr& msg);
            void onDialogueUtteranceMessageReceivedFromROS(const eu_nifti_ocu_msg_ros::DialogueUtteranceMessage::ConstPtr& msg);
            void onDialogueUtteranceMessageReceivedFromCAST(const cast::cdl::WorkingMemoryChange & _wmc);

            ::ros::NodeHandle nodeHandle;
            ::ros::Publisher publisherDialogueUtterance;

            static const char* DIALOGUE_TOPIC;

            static const char* DIALOGUE_UTTERANCE_MESSAGE_PROVIDER;
            static const char* DIALOGUE_SA_NAME;
        };

    };

};

#endif // ROS_CAST_CONVERTER_HPP_

