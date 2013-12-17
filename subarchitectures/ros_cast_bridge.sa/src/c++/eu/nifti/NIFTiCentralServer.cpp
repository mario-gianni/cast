// Benoit 2011-04-12

#include <ros/ros.h>

#include "eu/nifti/NIFTiCentralServer.hpp"
#include "eu/nifti/ROS_CAST_Converter.hpp"

namespace eu
{
    namespace nifti
    {
        int NIFTiCentralServer::uuid = 1;
        
        void NIFTiCentralServer::configure(const cast::cdl::StringMap & _config, const Ice::Current & _current)
        {
            cast::ManagedComponent::configure(_config, _current);
            //println("IN configure");

            // TODO: Pass the real arguments
            int argc = 0;
            char **argv = NULL;
            ::ros::init(argc, argv, "NIFTiCentralServer");

            //println("OUT configure");
        }
        
        void NIFTiCentralServer::start()
        {
            //println("IN start");

            nodeHandle = new ::ros::NodeHandle();
            
	    // Todo: Check if this service already exists. If it does, than crash completely and avoid possible ambiguities
            // package interface testing
	    // bool InterfaceTester::checkForService 	( 	std::string  	serviceName 	 )

// https://trac.dfki.de/nifti/ticket/131
  // Avoid advertising the service twice.
  if (::ros::service::waitForService("/eoi/RequestForUUIDs", ::ros::Duration(1.0))) {
    ROS_ERROR("UUID manager already running. Crashing this node...");
    assert(false);
  }

            // Starts a service that receives requests for UUIDs
            service = nodeHandle->advertiseService("/eoi/RequestForUUIDs", &NIFTiCentralServer::onRequestForUUIDs, this);


            // Subscribes to receive requests for the ROS time
            addChangeFilter
                    (
                    cast::createLocalTypeFilter<eu::nifti::RequestForROSTime > (cast::cdl::ADD),
                    new cast::MemberFunctionChangeReceiver<NIFTiCentralServer > (this, &NIFTiCentralServer::onRequestForROSTime)
                    );

            
            //::ros::start();

            //println("OUT start");
        }

        void NIFTiCentralServer::runComponent()
        {
            //println("IN runComponent");
            
            //::ros::spin(); // Makes this thread listen for requests for UUIDs
            ::ros::AsyncSpinner spinner(0);
            spinner.start(); 

            //println("OUT runComponent");
        }

        void NIFTiCentralServer::stop()
        {
            //println("IN stop");
            
            ::ros::shutdown();
            delete nodeHandle;
            //::ros::shutdown();

            //println("OUT stop");
        }

        


        extern "C"
        {

            cast::CASTComponentPtr newComponent()
            {
                return new NIFTiCentralServer();
            }
        }


    }
}







