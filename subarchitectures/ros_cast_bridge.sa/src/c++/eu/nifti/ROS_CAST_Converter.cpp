// Benoit 2011-01-12

#include <ros/ros.h>
#include <cast/slice/CDL.hpp>

#include "eu/nifti/ConverterUtil_Dialogue.hpp"
#include "eu/nifti/ConverterUtil_GUI.hpp"

#include "ROS_CAST_Converter.hpp"

namespace eu
{
    namespace nifti
    {
        const char* ROS_CAST_Converter::DIALOGUE_TOPIC = "ocu/dialogue";
        
        // Todo Put in parameter
        const char* ROS_CAST_Converter::DIALOGUE_UTTERANCE_MESSAGE_PROVIDER = "gui-nifti";
        const char* ROS_CAST_Converter::DIALOGUE_SA_NAME = "dialogue";
        
        void ROS_CAST_Converter::onDialogueUtteranceMessageReceivedFromROS(const eu_nifti_ocu_msg_ros::DialogueUtteranceMessage::ConstPtr& msg)
        {
            ROS_INFO("Received a dialogue utterance message from ROS: %s", msg->utterance.c_str());

            // Todo Put the name of the other SA in a parameter
            try
            {
                addToWorkingMemory(newDataID(), "dialogue", ConverterUtil_Dialogue::convertDialogueUtteranceMessageToCAST(msg));

            }
            catch (cast::AlreadyExistsOnWMException ex)
            {
                std::cerr << "Could not write this dialogue utterance to working memory: " << ex.message << std::endl;
            }
            catch (cast::UnknownSubarchitectureException ex)
            {
                std::cerr << "Could not write this dialogue utterance to working memory: " << ex.message << std::endl;
            }

        }
        
        void ROS_CAST_Converter::onDialogueUtteranceMessageReceivedFromCAST(const cast::cdl::WorkingMemoryChange & _wmc)
        {
            std::cout << "IN onDialogueUtteranceMessageReceivedFromCAST with: " << _wmc.address.subarchitecture <<  _wmc.address.id << std::endl;
            
            eu::nifti::ocu::msg::cast::DialogueUtteranceMessagePtr msg = getMemoryEntry<eu::nifti::ocu::msg::cast::DialogueUtteranceMessage>(_wmc.address);            
            
            ROS_INFO("Received a dialogue utterance message from CAST: %s", msg->utterance.c_str());
            
            eu_nifti_ocu_msg_ros::DialogueUtteranceMessagePtr rosMsg = ConverterUtil_Dialogue::convertDialogueUtteranceMessageToROS(msg);
            
            publisherDialogueUtterance.publish(rosMsg);
            
            try
            {
                deleteFromWorkingMemory(_wmc.address);
            }
            catch (cast::DoesNotExistOnWMException ex)
            {
                std::cerr << "Could not delete this dialogue utterance message from working memory: " << ex.message << std::endl;
            }
            catch (cast::PermissionException ex)
            {
                std::cerr << "Could not delete this dialogue utterance message from working memory: " << ex.message << std::endl;
            }

            //std::cout << "OUT onDialogueUtteranceMessageReceivedFromCAST with: " << _wmc.address.subarchitecture <<  _wmc.address.id << std::endl;
        }

        void ROS_CAST_Converter::onSelectionMessageReceived(const eu_nifti_ocu_msg_ros::SelectionMessage::ConstPtr& msg)
        {


            if (msg->action == eu_nifti_ocu_msg_ros::SelectionMessage::SELECT)
            {
                ROS_INFO("Received SELECT: [%s]", msg->userID.c_str());
                //castProxy->onSelected(msg->id, msg->header.stamp.toSec());
            }
            else
            {
                ROS_INFO("Received DESELECT: [%s]", msg->userID.c_str());
                //castProxy->onDeselected(msg->id, msg->header.stamp.toSec());
            }



            // Todo Put the name of the other SA in a parameter
            try
            {
                addToWorkingMemory(newDataID(), "gui", ConverterUtil_GUI::convertSelectionMessageToCAST(msg));

            }
            catch (cast::AlreadyExistsOnWMException ex)
            {
                std::cerr << "Could not write this selection to working memory: " << ex.message << std::endl;
            }
            catch (cast::UnknownSubarchitectureException ex)
            {
                std::cerr << "Could not write this selection to working memory: " << ex.message << std::endl;
            }

        }

        void ROS_CAST_Converter::configure(const cast::cdl::StringMap & _config, const Ice::Current & _current)
        {
            cast::ManagedComponent::configure(_config, _current);
            //println("IN configure");

            // TODO: Pass the real arguments
            int argc = 0;
            char **argv = NULL;
            ::ros::init(argc, argv, "ros_cast_converter");

            //println("OUT configure");
        }

        void ROS_CAST_Converter::start()
        {
            //std::cout << "IN start " << this->subarchitectureID() << std::endl;

            publisherDialogueUtterance = nodeHandle.advertise<eu_nifti_ocu_msg_ros::DialogueUtteranceMessage > (DIALOGUE_TOPIC, 1);
            
            addChangeFilter
                    (
                    cast::createChangeFilter<eu::nifti::ocu::msg::cast::DialogueUtteranceMessage > (cast::cdl::ADD, DIALOGUE_UTTERANCE_MESSAGE_PROVIDER, "", "", cast::cdl::ALLSA),
                    new cast::MemberFunctionChangeReceiver<ROS_CAST_Converter > (this, &ROS_CAST_Converter::onDialogueUtteranceMessageReceivedFromCAST)
                    );      

            //std::cout << "OUT start " << this->subarchitectureID() << std::endl;
        }

        void ROS_CAST_Converter::runComponent()
        {
            //println("IN runComponent");


            ::ros::Subscriber subSelection = nodeHandle.subscribe("/ocu/selection", 10, &ROS_CAST_Converter::onSelectionMessageReceived, this);
            ::ros::Subscriber subDialogue = nodeHandle.subscribe("/ocu/dialogue", 10, &ROS_CAST_Converter::onDialogueUtteranceMessageReceivedFromROS, this);


            ::ros::spin();


            //println("OUT runComponent");
        }

        void ROS_CAST_Converter::stop()
        {
            //println("IN stop");

            ::ros::shutdown();

            //println("OUT stop");
        }



        extern "C"
        {

            cast::CASTComponentPtr
            newComponent()
            {
                return new ROS_CAST_Converter();
            }
        }

    };

};
