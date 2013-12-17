#include <AREInterfaceComponent.hpp>
#include <are.hpp>

void AREInterfaceComponent::listen_current_artefact(const nifti_ar_world_msgs::ObjectArtifactConstPtr& msg)
{
	nifti_ar_world_msgs::ObjectArtifact obj = *msg;

	eu::nifti::env::are::ArtefactPtr artefact = new eu::nifti::env::are::Artefact();

	switch(obj.type)
	{
	case nifti_ar_world_msgs::ObjectArtifact::TYPE_CAR:
		artefact->type = eu::nifti::env::are::CAR;
		break;
	case nifti_ar_world_msgs::ObjectArtifact::TYPE_MALE:
		artefact->type = eu::nifti::env::are::MALE;
		break;
	case nifti_ar_world_msgs::ObjectArtifact::TYPE_FEMALE:
		artefact->type = eu::nifti::env::are::FEMALE;
		break;
	case nifti_ar_world_msgs::ObjectArtifact::TYPE_ROBOT:
		artefact->type = eu::nifti::env::are::ROBOT;
		break;
	default:
		println("************** ERROR - Artefact Type not found ****************");
		break;
	}

	artefact->label = obj.label;
	artefact->confidenceDegree = obj.degree_of_confidence;
	artefact->mapPose.pos.x = obj.pose.position.x;
	artefact->mapPose.pos.y = obj.pose.position.y;
	artefact->mapPose.pos.z = obj.pose.position.z;
	artefact->mapPose.orient.x = obj.pose.orientation.x;
	artefact->mapPose.orient.y = obj.pose.orientation.y;
	artefact->mapPose.orient.z = obj.pose.orientation.z;
	artefact->mapPose.orient.w = obj.pose.orientation.w;
	artefact->bbox.x = obj.bounding_box.x;
	artefact->bbox.y = obj.bounding_box.y;
	artefact->bbox.z = obj.bounding_box.z;
	artefact->timestamp = obj.header.stamp.sec;

	println("AREInterfaceComponent: Artefact %s in WM",artefact->label.c_str());
	addToWorkingMemory(newDataID(),artefact);

}

void AREInterfaceComponent::start()
{
	println("************************************************************");
	println("***** AREInterfaceComponent ROS CAST Component *************");
	println("******************* Status: starting ***********************");
	println("************************************************************");
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);

	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "AREInterfaceComponent");

}

void AREInterfaceComponent::runComponent()
{
	println("************************************************************");
	println("********* AREInterfaceComponent ROS CAST Component *********");
	println("******************** Status: running ***********************");
	println("************************************************************");

	ros::NodeHandle node;
	artefact_subscriber = node.subscribe("/artefacts",10,&AREInterfaceComponent::listen_current_artefact,this);

    ros::Rate r(20);
    while(ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();
    }

}

extern "C"
{
cast::CASTComponentPtr newComponent()
{
	return new AREInterfaceComponent();
}
}
