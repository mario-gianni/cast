// Benoit 2011-04-12

#ifndef NIFTI_CENTRAL_SERVER_HPP_
#define NIFTI_CENTRAL_SERVER_HPP_

#include <cast/architecture.hpp>

#include <eu_nifti_env_msg_ros/RequestForUUIDs.h>

#include "eu/nifti/ros/Time.hpp"

#include "eu/nifti/ConverterUtil.hpp"
#include "eu/nifti/RequestForROSTime.hpp"

namespace eu
{
    namespace nifti
    {

        class NIFTiCentralServer : public cast::ManagedComponent
        {
        public:
            
            // Returns the ROS time in CAST format
            inline eu::nifti::ros::TimePtr getROSTime()
            {
                return eu::nifti::ConverterUtil::convertTimeToCAST(::ros::Time::now());
            }

            inline int getUUID()
            {
                // START MUTEX

                return uuid++;

                // END MUTEX
            }
            
        protected:
            virtual void configure(const cast::cdl::StringMap & _config, const Ice::Current & _current);
            virtual void start();
            virtual void runComponent();
            virtual void stop();

            inline void onRequestForROSTime(const cast::cdl::WorkingMemoryChange & _wmc)
            {
                eu::nifti::RequestForROSTimePtr req = getMemoryEntry<eu::nifti::RequestForROSTime>(_wmc.address);
                req->time = getROSTime();
                overwriteWorkingMemory<eu::nifti::RequestForROSTime>(_wmc.address, req);
            }
            
            inline bool onRequestForUUIDs(eu_nifti_env_msg_ros::RequestForUUIDs::Request &req, eu_nifti_env_msg_ros::RequestForUUIDs::Response &res )
            {
                res.uuids.resize(req.numRequested);
                for(uint i = 0; i < res.uuids.size(); i++)
                {
                    res.uuids.at(i) = getUUID(); // Todo: this will enter and leave the mutex zone very quickly. Maybe acquire the re-entrant mutex here.
                }
                return true;
            }

        private:
            static int uuid;

            ::ros::NodeHandle* nodeHandle;
            ::ros::ServiceServer service;
        };

    }

}

#endif // NIFTI_CENTRAL_SERVER_HPP_

