#ifndef _ARE_INTERFACE_COMPONENT_HPP_
#define _ARE_INTERFACE_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <nifti_ar_world_msgs/ObjectArtifact.h>

class AREInterfaceComponent : public cast::ManagedComponent
{
public:
	ros::Subscriber artefact_subscriber;
	void listen_current_artefact(const nifti_ar_world_msgs::ObjectArtifactConstPtr&);
protected:
	virtual void start();
	virtual void runComponent();
};

#endif
