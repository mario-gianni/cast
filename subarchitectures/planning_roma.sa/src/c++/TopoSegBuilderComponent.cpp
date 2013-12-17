#include <TopoSegBuilderComponent.hpp>

void TopoSegBuilderComponent::topoSegActionReceived(const WorkingMemoryChange& _wmc)
{
    TogoGraphBuilderActionPtr current = getMemoryEntry<TogoGraphBuilderAction>(_wmc.address);
    std::string id_action = _wmc.address.id;
    
    module_control_action::ControlActionGoal goal;
    
    while(!client->waitForServer(ros::Duration(5.0)))
	{
        println("Waiting for the toposeg_control action server to come up");
	}
	
	if(current->op == eu::nifti::Planning::slice::START)
    {
        goal.goal.control = module_control_action::ControlGoal::PLAY_MODULE;
    }
    else if(current->op == eu::nifti::Planning::slice::END)
    {
        goal.goal.control = module_control_action::ControlGoal::STOP_MODULE;
    }
    else
    {
        println("Request unknown");
    }
    
    println("Sending goal");
        
    client->sendGoal(goal.goal);
        
   	client->waitForResult();
   	
   	if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	    println("TogoGraphBuilderAction Status: EXECUTED");
	    current->status = EXECUTED;
	    overwriteWorkingMemory(id_action,current);
	}
	else
    {
        println("TogoGraphBuilderAction Status: FAILED");
        current->status = FAILED;
        overwriteWorkingMemory(id_action,current);
    }
}

void TopoSegBuilderComponent::start()
{
    println("************************************************************");
    println("******* TopoSegBuilderComponent ROS CAST Component *********");
    println("******************* Status: starting ***********************");
    println("************************************************************");
    
    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	
	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "TopoSegBuilderComponent");
	
	client = new TopoSegClient("toposeg_control",true);
	
	println("***************** Initializing filters *********************");
	addChangeFilter(createLocalTypeFilter<TogoGraphBuilderAction>(ADD),new MemberFunctionChangeReceiver<TopoSegBuilderComponent>(this,&TopoSegBuilderComponent::topoSegActionReceived));	

}

void TopoSegBuilderComponent::runComponent()
{
    println("************************************************************");
    println("******** TopoSegBuilderComponent ROS CAST Component ********");
    println("******************* Status: running ************************");
    println("************************************************************");
}

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
    	return new TopoSegBuilderComponent();
  	}
}
