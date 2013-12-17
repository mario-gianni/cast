#include <ActionFailureManager.hpp>
#include <geometry_msgs/Twist.h>

void ActionFailureManager::start()
{
	println("************************************************************");
	println("********* ActionFailureManager ROS CAST Component **********");
	println("******************* Status: starting ***********************");
	println("************************************************************");
    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	
	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "ActionFailureManager");

	ros::NodeHandle node;
	this->exe_pub = node.advertise<geometry_msgs::Twist>("private/nav2/cmd_vel",1);
	
	 addChangeFilter(cast::createLocalTypeFilter<eu::nifti::Planning::slice::FailurePlanAction>(cast::cdl::ADD),new cast::MemberFunctionChangeReceiver<ActionFailureManager>(this,&ActionFailureManager::actionFailure),cast::HIGH);
}

void ActionFailureManager::runComponent()
{
	println("************************************************************");
	println("********* ActionFailureManager ROS CAST Component **********");
	println("******************* Status: running ************************");
	println("************************************************************");
}

void ActionFailureManager::actionFailure(const cast::cdl::WorkingMemoryChange& _wmc)
{
    println("Filter callback: the USER has decided to abort the previous plan");
    eu::nifti::Planning::slice::FailurePlanActionPtr interrupt = getMemoryEntry<eu::nifti::Planning::slice::FailurePlanAction>(_wmc.address);
    std::string id_action = _wmc.address.id;
    
    MoveBaseClient ac1("move_base",true);
	MoveBaseClient ac2("trp_as",true);
	
	ac1.waitForServer(ros::Duration(5.0));
	ac2.waitForServer(ros::Duration(5.0));

    ac1.cancelGoalsAtAndBeforeTime(ros::Time::now());
    ac2.cancelGoalsAtAndBeforeTime(ros::Time::now());
    ac1.cancelAllGoals();
    ac2.cancelAllGoals();

    //overwriteWorkingMemory(id_action,interrupt);

    //println("The USER has interrupted the current plan");
	    
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.orientation.w = 1.0;
	
	ac1.sendGoal(goal);
	ac1.waitForResult();
	
	ac2.sendGoal(goal);
	ac2.waitForResult();
	
	geometry_msgs::Twist cmdvel;
	cmdvel.linear.x = cmdvel.linear.y = 0.0;
	cmdvel.angular.z = 0.0;
	this->exe_pub.publish(cmdvel);
	
	println("The USER has interrupted the current plan");

}

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
    	return new ActionFailureManager();
  	}
}
