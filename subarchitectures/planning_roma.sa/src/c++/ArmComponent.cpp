#include <ArmComponent.hpp>
#include <nifti_arm_msgs/msg_height.h>

void ArmComponent::setHeight(const cast::cdl::WorkingMemoryChange& _wmc)
{
	eu::nifti::Planning::slice::ChangeArmHeightPtr current = getMemoryEntry<eu::nifti::Planning::slice::ChangeArmHeight>(_wmc.address);
    std::string id_action = _wmc.address.id;


}

void ArmComponent::start()
{
	println("************************************************************");
	println("************* ArmComponent ROS CAST Component **************");
	println("******************* Status: starting ***********************");
	println("************************************************************");
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);

	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "ArmComponent");
}

void ArmComponent::runComponent()
{
	println("************************************************************");
	println("************* ArmComponent ROS CAST Component **************");
	println("******************** Status: running ***********************");
	println("************************************************************");
}

extern "C"
{
cast::CASTComponentPtr newComponent()
{
	return new ArmComponent();
}
}
