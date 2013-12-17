#include "NiftiNavigationGoals.hpp"

using namespace eu::nifti::planning::slice;

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
		return new NiftiNavigationGoals();
	}
}	                  
			                       
void NiftiNavigationGoals::start() 
{
	char* argv[] = {};
	int argc = sizeof(argv)/sizeof(char *);
	ros::init(argc, argv, "nifti_navigation_goals");

        //tell the action client that we want to spin a theread by default
	ac = new MoveBaseClient("move_base", true);

	addChangeFilter(cast::createLocalTypeFilter<Action>(cast::cdl::ADD),
		new cast::MemberFunctionChangeReceiver<NiftiNavigationGoals>(this, &NiftiNavigationGoals::ExecuteAction));

	println("started nifti_navigation_goals node with NiftiNavigationGoals component");
}

void NiftiNavigationGoals::ExecuteAction(const cast::cdl::WorkingMemoryChange & _wmc)
{
	//ActionPtr ActionMsg = getMemoryEntry<Action>(_wmc.address);
	//println("got a navigation action (" + ActionMsg->name + ")");

	//TODO: this is just a fix for stop action
	println("got stop action");
	ActionPtr ActionMsg = new Action(0, "stop", std::vector<ArgumentPtr>(), PENDING);

	//wait for the action server to come up
	if(!ac->waitForServer(ros::Duration(10.0)))
	{
		println("move_baser_server is not connected, navigation action will be discarded!");
		return;
	}
	
	//if there is a goal cancel it
	if(ac->getState() != actionlib::SimpleClientGoalState::LOST)
	{
		ac->cancelGoal();
	}

	//now generating move_base_goal
	move_base_msgs::MoveBaseGoal goal;

	//setting default frame to move and time for tf
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	if(ActionMsg->name == "move-forward")
	{
		//move forward 0.5m
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.orientation.w = 1.0;
	}
	else if(ActionMsg->name == "move-left")
	{
		//move to position 0.5m to the left and turn by 45 degrees
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.position.y = -0.5;
		goal.target_pose.pose.orientation.z = sin(PI/8);
		goal.target_pose.pose.orientation.w = cos(PI/8);
	}
	else if(ActionMsg->name == "move-right")
	{
		//move to position 0.5m to the right and turn by 45 degree
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.position.y = 0.5;
		goal.target_pose.pose.orientation.z = -sin(PI/8);
		goal.target_pose.pose.orientation.w = cos(PI/8);
	}
	else if(ActionMsg->name == "move-back")
	{
		//move backward by 0.5m
		goal.target_pose.pose.position.x = -0.5;
		goal.target_pose.pose.orientation.w = 1.0;
	}
	else if(ActionMsg->name == "turn-left")
	{
		//turn left by 30 degrees
		goal.target_pose.pose.orientation.z = sin(PI/12);
		goal.target_pose.pose.orientation.w = cos(PI/12);
	}
	else if(ActionMsg->name == "turn-right")
	{
		//turn right by 30 degrees
		goal.target_pose.pose.orientation.z = -sin(PI/12);
		goal.target_pose.pose.orientation.w = cos(PI/12);
	}
	else if(ActionMsg->name == "turn-back")
	{
		//turn 180 degrees
		goal.target_pose.pose.orientation.z = 1.0;
	}
	else if(ActionMsg->name == "stop")
	{
		//stop
		goal.target_pose.pose.orientation.w = 1.0;
	}
	else if(ActionMsg->name == "go-to-position")
	{
		//TODO: first check that arguments is not empty!
		eu::nifti::mapping::Pose3DPtr Pose3DMsg = getMemoryEntry<eu::nifti::mapping::Pose3D>(ActionMsg->arguments[0]->featureValueWMP->address);

		goal.target_pose.pose.position.x = Pose3DMsg->x;
		goal.target_pose.pose.position.y = Pose3DMsg->y;
		goal.target_pose.pose.position.z = Pose3DMsg->z;
		goal.target_pose.pose.orientation.w = 1.0;

		//this position is in map frame
		goal.target_pose.header.frame_id = "map";
	}
	else
	{
		println("unrecognized action: " + ActionMsg->name);
	}
	
	ROS_INFO("setting navigation goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)",
		goal.target_pose.header.frame_id.c_str(),
		goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z,
		goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
		goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);
	ac->sendGoal(goal);

	//TODO: uncomment when above fix is solved
	//ActionMsg->status = eu::nifti::planning::slice::INPROGRESS;
	//overwriteWorkingMemory(_wmc.address, ActionMsg);

	/*ac->waitForResult();

	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("goal succeeded");
		ActionMsg->status = eu::nifti::planning::slice::SUCCEEDED;
    		overwriteWorkingMemory(_wmc.address, ActionMsg);
	}
	else
	{
		ROS_INFO("goal failed");
		ActionMsg->status = eu::nifti::planning::slice::FAILED;
    		overwriteWorkingMemory(_wmc.address, ActionMsg);
	}*/
}

void NiftiNavigationGoals::runComponent() 
{
	return;
}
