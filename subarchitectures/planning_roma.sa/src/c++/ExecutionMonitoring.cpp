#include <string>
#include <fstream>
#include <sstream>
#include "tf/transform_listener.h"
#include <ExecutionMonitoring.hpp>
#include <voronoiseg/topological_mapping_control.h>
#include <eclipse_prolog_msgs/ActionScheduled.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nifti_robot_driver_msgs/FlipperCommand.h>
#include <gap_traversal_msgs/gap_service.h>
#include <gap_traversal_msgs/gap_message.h>

#include <nav_msgs/GetPlan.h>
#include <voronoiseg/PoseToID.h>


void ExecutionMonitoring::myReadPlan(const WorkingMemoryChange& _wmc)
{
	m_Thread = boost::thread(&ExecutionMonitoring::readPlan,this,_wmc);
}
void ExecutionMonitoring::myReadGraph(const WorkingMemoryChange& _wmc)
{
	g_Thread = boost::thread(&ExecutionMonitoring::readGraph,this,_wmc);
}

void ExecutionMonitoring::myreadTask(const WorkingMemoryChange& _wmc)
{
	t_Thread = boost::thread(&ExecutionMonitoring::readTask,this,_wmc);
}

EC_word ExecutionMonitoring::gotoNodeAction(GoToNodeActionPtr action)
{
	EC_word result;
	EC_functor _action = EC_functor((char*)action->name.c_str(),3);
	EC_word _node = parser.node(action->node);
	
	EC_word _theta;
	
	if(action->theta == 0)
	{
	    //action->theta = atan2((action->node->y - this->position->node->y),(action->node->x - this->position->node->x));
	    action->theta = atan2((action->node->y - this->current_pose->y),(action->node->x - this->current_pose->x));
	    _theta = EC_word(action->theta);
	}
	else
	{
	    _theta = EC_word(action->theta);
	}
	EC_word _time = EC_word(action->time);
	result = term(_action,_node,_theta,_time);
	return result;
}

void ExecutionMonitoring::computeOrientation(TimelinePtr timeline)
{	
	int n = timeline->actions.size();
	
	for(int i = 0; i < n-1; i++)
	{
		double theta;
		if(GoToNodeActionPtr next = GoToNodeActionPtr::dynamicCast(timeline->actions[i]))
		{
			if(GoToNodeActionPtr current = GoToNodeActionPtr::dynamicCast(timeline->actions[i+1]))
			{
				theta = atan2((next->node->y - current->node->y),(next->node->x - current->node->x));
				next->theta = theta;
			}
		}
	}
}

TimelinePtr ExecutionMonitoring::computeOrientation2(TimelinePtr timeline)
{
    double theta;
    int index = 0;
    GoToNodeActionPtr first = new GoToNodeAction();
    TimelinePtr new_timeline = new Timeline();
    int n = timeline->actions.size();
    
    for(int i = 0; i < n; i++)
    {
        if(GoToNodeActionPtr temp = GoToNodeActionPtr::dynamicCast(timeline->actions[i]))
        {
            first = temp;
            new_timeline->actions.push_back(first);
            index = i;
            break;
        }
        else
        {
            new_timeline->actions.push_back(timeline->actions[i]);
            index = i;
            println("this action is not a GoToNodeAction");
        }
    }
    
    index++;
    
    if(index == n)
    {
        return new_timeline;
    }
    
    for(int i = index; i < n; i++)
    {
        if(GoToNodeActionPtr temp = GoToNodeActionPtr::dynamicCast(timeline->actions[i]))
        {
            if(temp->theta == 0)
            {
                theta = atan2((temp->node->y - first->node->y),(temp->node->x - first->node->x));
                temp->theta = theta;
		println("orientation: %f",temp->theta);
                first = temp;
                new_timeline->actions.push_back(temp);
            }
            else
            {
                first = temp;
                new_timeline->actions.push_back(temp);
            }

        }
        else
        {
            new_timeline->actions.push_back(timeline->actions[i]);
        }
    }  

    for(int i = 0 ; i < n ; i++)
    {
	println(this->utils.to_string(new_timeline->actions[i]));
    }
    
    return new_timeline;
}

void ExecutionMonitoring::start()
{
	println("************************************************************");
	println("********** ExecutionMonitoring ROS CAST Component **********");
	println("******************* Status: starting ***********************");
	println("************************************************************");

    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	
	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "ExecutionMonitoring");
	
	ros::NodeHandle node;
	this->actions_pub = node.advertise<eclipse_prolog_msgs::ActionScheduled>("planner/task",1);
	
	println("Compiling the knowledge base...");
	this->engine.compile("prolog/main.ecl");
	this->engine.resume();
	this->engine.message_output(1);
	this->engine.message_output(2);
	
	this->mixed_status = -1;
	this->mixedInitiative = nil();
	this->task_aborted = false;
	
    this->topo_graph = new Graph();
    this->task = new Task();
    this->task->status = COMPLETED;

    println("******************** Initializing filters *****************************");
   
    // Filters for TopoSeg
	//addChangeFilter(createLocalTypeFilter<Graph>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readGraph),LOW);
	addChangeFilter(createLocalTypeFilter<Graph>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::myReadGraph),LOW);
	
	//Filters for Base station and current region
	addChangeFilter(createLocalTypeFilter<eu::nifti::env::position::CurrentPos>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readCurrentPosition));
    addChangeFilter(createLocalTypeFilter<eu::nifti::env::position::BasePos>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readBaseStation));
    
    //Filters for the current pose of the robot
    addChangeFilter(createLocalTypeFilter<eu::nifti::env::position::Pose>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readCurrentPose));
    
    // Filters for CarObjectOfInterest
    std::string src = "FEMAreasGenerator"; //name of the component
    std::string id;
    std::string sa;
    addChangeFilter(createChangeFilter<eu::nifti::env::CarObjectOfInterest>(ADD,src,id,sa,ALLSA),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readCarObjectOfInterest));

    // Filter for the TASK
    //std::string src1 = "TaskWriter"; //name of the component
    //std::string id1;
    //std::string sa1;
	//addChangeFilter(createLocalTypeFilter<Task>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readTask));
	addChangeFilter(createLocalTypeFilter<Task>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::myreadTask));
	//addChangeFilter(createChangeFilter<Task>(ADD,src1,id1,sa1,ALLSA),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::myreadTask));

	//Filter for the PLAN
	//addChangeFilter(createLocalTypeFilter<Plan>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readPlan));
	addChangeFilter(createLocalTypeFilter<Plan>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::myReadPlan));
	
	//Filter for the ACTION
	addChangeFilter(createLocalTypeFilter<Action>(OVERWRITE),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::currentActionReceived),HIGH);

	//Filter for Action Notification Failure
	//addChangeFilter(createLocalTypeFilter<Action>(OVERWRITE),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::currentActionFailureNotificationReceived),HIGH);
	//addChangeFilter(createLocalTypeFilter<Action>(OVERWRITE),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::mycurrentActionFailureNotificationReceived),HIGH);

	//Filter for the Gap
	addChangeFilter(createLocalTypeFilter<Gap>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readDetectedGap));

	addChangeFilter(createLocalTypeFilter<Artefact>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readArtefact));

	addChangeFilter(createLocalTypeFilter<BatteryStatus>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readBatteryStatus));
	addChangeFilter(createLocalTypeFilter<WiFiStatus>(ADD),new MemberFunctionChangeReceiver<ExecutionMonitoring>(this,&ExecutionMonitoring::readWifiStatus));
}

bool ExecutionMonitoring::waitForActionExecution()
{
    if(this->current_action->status != PENDING)
    {
        return true; 
    }
    return false;
}

/*bool ExecutionMonitoring::waitForNotificationActionFailure()
{
    if(this->interrupt->status != PENDING)
    {
        return true;
    }
    return false;
}*/

void ExecutionMonitoring::currentActionReceived(const WorkingMemoryChange& _wmc)
{
    println("The status of the current action %s",this->utils.to_string(this->current_action).c_str());
    println("has been changed from the component %s",_wmc.src.c_str());
    this->current_action = getMemoryEntry<Action>(_wmc.address);
}

/*void ExecutionMonitoring::currentActionFailureNotificationReceived(const WorkingMemoryChange& _wmc)
{
    println("The status of the current action %s",this->utils.to_string(this->current_action).c_str());
    println("has been changed from the component %s",_wmc.src.c_str());
    this->interrupt = getMemoryEntry<FailurePlanAction>(_wmc.address);
}*/

void ExecutionMonitoring::readBatteryStatus(const WorkingMemoryChange& _wmc)
{
	println("Filter callback: a new battery status has been red from the WM");
	this->battery_status = getMemoryEntry<BatteryStatus>(_wmc.address);
}

void ExecutionMonitoring::readWifiStatus(const WorkingMemoryChange& _wmc)
{
	println("Filter callback: a new wifi status has been red from the WM");
	this->wifi_status = getMemoryEntry<WiFiStatus>(_wmc.address);
}

void ExecutionMonitoring::readArtefact(const WorkingMemoryChange& _wmc)
{
	println("Filter callback: a new artefact has been red from the WM");
	this->artefact = getMemoryEntry<Artefact>(_wmc.address);
}

void ExecutionMonitoring::readDetectedGap(const WorkingMemoryChange& _wmc)
{
	println("Filter callback: a new gap has been red from the WM");
	this->_gap = getMemoryEntry<Gap>(_wmc.address);
	/*	
	println("Gap Parameters");

	std::string label = "gap" + this->_gap->id;
	EC_functor name = EC_functor((char*)"detected_gap",5);
	EC_atom obj = EC_atom((char*)label.c_str());
	EC_word x = newvar();
	EC_word y = newvar();
	EC_word z = newvar();
	EC_word theta = newvar();
	EC_word detected = term(name,obj,x,y,z,theta);
	this->engine.my_assert(detected);
	println("The detected gap has been asserted in the knowledge base with its pose");

	if(this->_gap->traversability == true)
	{
		EC_functor p1 = EC_functor((char*)"traversable",1);
		EC_word result1 = term(p1,obj);
		this->engine.my_assert(result1);
		println("The gap is traversable");

		EC_functor p2 = EC_functor((char*)"starting_pose",5);
		EC_word x1 = EC_word(this->_gap->init.pos.x);
		EC_word y1 = EC_word(this->_gap->init.pos.y);
		EC_word z1 = EC_word(this->_gap->init.pos.z);
		// da continuare
		tf::Quaternion quat1 = tf::Quaternion(this->_gap->init.orient.x,this->_gap->init.orient.y,this->_gap->init.orient.z,this->_gap->init.orient.w);
		EC_word theta1 = EC_word(tf::getYaw(quat1));
		EC_word result2 = term(p2,obj,x1,y1,z1,theta1);
		this->engine.my_assert(result2);
		println("The starting pose of the robot has been asserted in the KB for the overcome_gap task");

		EC_functor p3 = EC_functor((char*)"ending_pose",5);
		EC_word x2 = EC_word(this->_gap->final.pos.x);
		EC_word y2 = EC_word(this->_gap->final.pos.y);
		EC_word z2 = EC_word(this->_gap->final.pos.z);
		tf::Quaternion quat2 = tf::Quaternion(this->_gap->final.orient.x,this->_gap->final.orient.y,this->_gap->final.orient.z,this->_gap->final.orient.w);
		EC_word theta2 = EC_word(tf::getYaw(quat2));
		EC_word result3 = term(p3,obj,x2,y2,z2,theta2);
		this->engine.my_assert(result3);
		println("The ending pose of the robot has been asserted in the KB for the overcome_gap task");

		EC_functor p4 = EC_functor((char*)"configuration",5);
		EC_word a1 = EC_word(this->_gap->conf.flipperAngleFL);
		EC_word a2 = EC_word(this->_gap->conf.flipperAngleFR);
		EC_word a3 = EC_word(this->_gap->conf.flipperAngleRR);
		EC_word a4 = EC_word(this->_gap->conf.flipperAngleRL);
		EC_word result4 = term(p4,obj,a1,a2,a3,a4);
		this->engine.my_assert(result4);
		println("The configuration of the flippers of the robot have been asserted in the KB for the overcome_gap task");

	}
	else
	{
		EC_functor p1 = EC_functor((char*)"not_traversable",1);
		EC_word result = term(p1,obj);
		this->engine.my_assert(result);
        println("The gap is not traversable");
	}
	*/
}

void ExecutionMonitoring::readCurrentPose(const WorkingMemoryChange& _wmc)
{
    println("A new pose of the robot has been red from the WM");
    this->current_pose = getMemoryEntry<eu::nifti::env::position::Pose>(_wmc.address);
}

void ExecutionMonitoring::readCarObjectOfInterest(const WorkingMemoryChange& _wmc)
{
    println("Filter callback: a new object has been red from the WM");
    this->object = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(_wmc.address);
    this->utils.storeDetectedObject(this->object,"prolog/detected_objects.ecl");
    println("The detected object has been stored in a file with its pose");
    this->utils.storeVantagePoints(this->object,"prolog/detected_objects.ecl");
    println("The vantage points around the detected object have been stored in a file with their pose");
}

void ExecutionMonitoring::readCurrentPosition(const WorkingMemoryChange& _wmc)
{
    //println("Filter callback: the region in which the robot is has been red from the WM");
    this->position = getMemoryEntry<eu::nifti::env::position::CurrentPos>(_wmc.address);
}

void ExecutionMonitoring::readBaseStation(const WorkingMemoryChange& _wmc)
{
    this->base = getMemoryEntry<eu::nifti::env::position::BasePos>(_wmc.address);
    this->utils.storeBaseStation(this->base,"prolog/base_station.ecl");
    println("The base station of the robot has been stored in a file");
    this->current_position = parser.origin(this->base);
    this->engine.my_assert(this->current_position);
    println("The base position of the robot has been asserted in the knowledge base");
    
}

void ExecutionMonitoring::readGraph(const WorkingMemoryChange& _wmc)
{
	println("A new graph has been red from the WM");
	this->topo_graph = getMemoryEntry<Graph>(_wmc.address);
}

int ExecutionMonitoring::getIdNode(double x, double y)
{
	int id = -2;

	ros::NodeHandle n;
	ros::ServiceClient getID = n.serviceClient<voronoiseg::PoseToID>("toposeg/convert_xy_to_id");
	voronoiseg::PoseToID srv;
	srv.request.x = x;
	srv.request.y = y;
	if(getID.exists())
	{
		if(getID.call(srv))
		{
			id = srv.response.id;
		}
		else
		{
			println("Failure to call convert_xy_to_id service");
		}
	}
	else
	{
		println("convert_xy_to_id service not available");
	}

	return id;
}

NodePtr ExecutionMonitoring::neighborhood(NodePtr node, GraphPtr graph)
{
    NodePtr min = new Node();
    
    min = graph->nodes[0];
    
    double distance;
    
    distance = sqrt((min->x - node->x)*(min->x - node->x) + (min->y - node->y)*(min->y - node->y));
    
    for(unsigned int i = 1; i < graph->nodes.size(); i++)
    {
        double temp;
        temp = sqrt((graph->nodes[i]->x - node->x)*(graph->nodes[i]->x - node->x) + (graph->nodes[i]->y - node->y)*(graph->nodes[i]->y - node->y));
        if(temp < distance)
        {
            distance = temp;
            min = graph->nodes[i];
        }
    }
    
    return min;
}

/*
double ExecutionMonitoring::calculatePathCost(NodePtr node, eu::nifti::env::VantagePointPtr vp)
{
    double cost = INFINITY;
    
    ros::NodeHandle n;
    ros::ServiceClient makePlanClient = n.serviceClient<nav_msgs::GetPlan>("make_plan");
    
    geometry_msgs::PoseStamped start;
    start.pose.position.x = node->x;
    start.pose.position.y = node->y;
    start.pose.position.z = 0;
    start.pose.orientation.w = 1;
    
    geometry_msgs::PoseStamped end;
    end.pose.position.x = vp->pose.x;
    end.pose.position.y = vp->pose.y;
    end.pose.position.z = vp->pose.z;
    btVector3 axisOfRotation(0,0,1);
    btQuaternion quat(axisOfRotation, vp->pose.theta);
    end.pose.orientation.x = quat.x();
	end.pose.orientation.y = quat.y();
	end.pose.orientation.z = quat.z();
	end.pose.orientation.w = quat.w();
    
    nav_msgs::GetPlan srv;
    
    srv.request.start = start;
    srv.request.goal = end;
    srv.request.tolerance = 0.1;
    
    if (makePlanClient.exists()) 
    {

        if(makePlanClient.call(srv))
        {
            if(srv.response.plan.poses.empty())
            {
                println("make_plan service returns that plan is empty");
            }
            else
            {
                cost = srv.response.cost;
            }
        }
        else
        {
            println("failed to call make_plan service");
        }
    }
    else
    {
        println("make_plan service not available");
    }

    
    return cost;
}
*/

double ExecutionMonitoring::calculatePathCost2(NodePtr node, eu::nifti::env::VantagePointPtr vp)
{
    double cost;
    
    cost = sqrt((node->x - vp->pose.x)*(node->x - vp->pose.x) + (node->y - vp->pose.y)*(node->y - vp->pose.y));
    
    return cost;
}

eu::nifti::env::VantagePointPtr ExecutionMonitoring::bestVantagePoint(NodePtr node, eu::nifti::env::ListOfVantagePoints vps)
{
    eu::nifti::env::VantagePointPtr best = new eu::nifti::env::VantagePoint();
    
    best = vps[0];
    
    //double cost = calculatePathCost(node,best);
    double cost = calculatePathCost2(node,best);
    
    for(unsigned int i = 1; i < vps.size(); i++)
    {
        //double temp = calculatePathCost(node,vps[i]);
        double temp = calculatePathCost2(node,vps[i]);
        if(temp < cost)
        {
            cost = temp;
            best = vps[i];
        }
    }
    return best;
}

EC_word ExecutionMonitoring::updateRobotStatus()
{
	EC_word internal_status = nil();

	/* Current Position of the robot in the metric map */
	if(this->current_pose != 0)
	{
		EC_word pose = parser.at(this->current_pose);
		internal_status = list(pose,internal_status);
	}

	/* Current Position of the robot in the topological map */
	if(this->position != 0)
	{
		EC_word region = parser.in(this->position);
		internal_status = list(region,internal_status);
	}


	/* CarObjectOfInterest */
	if(this->object != 0)
	{
		std::string object_label = this->object->name + this->utils.to_string(this->object->uuid);
		EC_functor name = EC_functor((char*)"detected",5);
		EC_atom obj = EC_atom((char*)object_label.c_str());
		EC_word x = EC_word(this->object->pose.x);
		EC_word y = EC_word(this->object->pose.y);
		EC_word z = EC_word(this->object->pose.z);
		EC_word theta = EC_word(this->object->pose.theta);
		EC_word result = term(name,obj,x,y,z,theta);
		internal_status = list(result,internal_status);
		for(unsigned int i = 0; i < this->object->vantagePoints.size(); i++)
		{
			EC_functor vp_node = EC_functor((char*)"vpnode",7);
			std::string temp = "vp" + this->utils.to_string(i);
			EC_atom l = EC_atom((char*)temp.c_str());
			EC_word x = EC_word(this->object->vantagePoints[i]->pose.x);
			EC_word y = EC_word(this->object->vantagePoints[i]->pose.y);
			EC_word z = EC_word(this->object->vantagePoints[i]->pose.z);
			EC_word theta = EC_word(this->object->vantagePoints[i]->pose.theta);
			EC_word gain = EC_word(this->object->vantagePoints[i]->gain);
			EC_word flag = EC_word(0);
			EC_word vp_node_term = term(vp_node,l,x,y,z,theta,gain,flag);
			EC_functor vp = EC_functor((char*)"vp",2);
			EC_word vp_term = term(vp,obj,vp_node_term);
			internal_status = list(vp_term,internal_status);
		}
	}

	 /* Current Topological Graph */
	if(this->topo_graph != 0)
	{
		for(unsigned int i = 0; i < this->topo_graph->nodes.size() ; i++)
		{
			this->utils.storeNodes(this->topo_graph->nodes[i],"prolog/nodes.ecl");
			//println("The %s has been stored in a file",this->utils.to_string(this->topo_graph->nodes[i]).c_str());
			EC_word result;
			EC_functor graph = EC_functor((char*)"graph",1);
			EC_word arg = parser.node(this->topo_graph->nodes[i]);
			//println("%s",this->topo_graph->nodes[i]->label.c_str());
			result = term(graph,arg);
			internal_status = list(result,internal_status);
		}
		for(unsigned int i = 0; i < this->topo_graph->edges.size(); i++)
		{
			this->utils.storeEdges(this->topo_graph->edges[i],"prolog/edges.ecl");
			//println("The connection between segmented regions %s has been stored in a file",this->utils.to_string(this->topo_graph->edges[i]).c_str());
			EC_word result;
			EC_functor pred = EC_functor((char*)"edge",1);
			EC_word arg = parser.edge(this->topo_graph->edges[i]);
			//println("a: %s",this->topo_graph->edges[i]->a->label.c_str());
			//println("b: %s",this->topo_graph->edges[i]->b->label.c_str());
			result = term(pred,arg);
			internal_status = list(result,internal_status);
		}
	}
	/* Gap */
	if(this->_gap != 0)
	{
		std::string label = "gap" + this->_gap->id;
		EC_functor name = EC_functor((char*)"detected_gap",5);
		EC_atom obj = EC_atom((char*)label.c_str());
		EC_word x = newvar();
		EC_word y = newvar();
		EC_word z = newvar();
		EC_word theta = newvar();
		EC_word detected = term(name,obj,x,y,z,theta);
		internal_status = list(detected,internal_status);
		println("The detected gap has been asserted in the knowledge base with its pose");

		if(this->_gap->traversability == true)
		{
			EC_functor p1 = EC_functor((char*)"traversable",1);
			EC_word result1 = term(p1,obj);
			internal_status = list(result1,internal_status);
			println("The gap is traversable");

			EC_functor p2 = EC_functor((char*)"starting_pose",5);
			EC_word x1 = EC_word(this->_gap->init.pos.x);
			EC_word y1 = EC_word(this->_gap->init.pos.y);
			EC_word z1 = EC_word(this->_gap->init.pos.z);
			// da continuare
			tf::Quaternion quat1 = tf::Quaternion(this->_gap->init.orient.x,this->_gap->init.orient.y,this->_gap->init.orient.z,this->_gap->init.orient.w);
			EC_word theta1 = EC_word(tf::getYaw(quat1));
			EC_word result2 = term(p2,obj,x1,y1,z1,theta1);
			internal_status = list(result2,internal_status);
			println("The starting pose of the robot has been asserted in the KB for the overcome_gap task");

			EC_functor p3 = EC_functor((char*)"ending_pose",5);
			EC_word x2 = EC_word(this->_gap->final.pos.x);
			EC_word y2 = EC_word(this->_gap->final.pos.y);
			EC_word z2 = EC_word(this->_gap->final.pos.z);
			tf::Quaternion quat2 = tf::Quaternion(this->_gap->final.orient.x,this->_gap->final.orient.y,this->_gap->final.orient.z,this->_gap->final.orient.w);
			EC_word theta2 = EC_word(tf::getYaw(quat2));
			EC_word result3 = term(p3,obj,x2,y2,z2,theta2);
			internal_status = list(result3,internal_status);
			println("The ending pose of the robot has been asserted in the KB for the overcome_gap task");

			EC_functor p4 = EC_functor((char*)"configuration",5);
			EC_word a1 = EC_word(this->_gap->conf.flipperAngleFL);
			EC_word a2 = EC_word(this->_gap->conf.flipperAngleFR);
			EC_word a3 = EC_word(this->_gap->conf.flipperAngleRR);
			EC_word a4 = EC_word(this->_gap->conf.flipperAngleRL);
			EC_word result4 = term(p4,obj,a1,a2,a3,a4);
			internal_status = list(result4,internal_status);
			println("The configuration of the flippers of the robot have been asserted in the KB for the overcome_gap task");	
			this->_gap = 0;

		}
		else
		{
			EC_functor p1 = EC_functor((char*)"not_traversable",1);
			EC_word result = term(p1,obj);
			internal_status = list(result,internal_status);
        		println("The gap is not traversable");
			this->_gap = 0;
		}
	}
	/* Artefact */
	if(this->artefact != 0)
	{
		this->utils.storeArtefact(this->artefact,"prolog/artefacts.ecl");
		EC_word result = parser.artefact(this->artefact);
		internal_status = list(result,internal_status);
		println("An artefact has been asserted in the KB");
	}
	/* Battery Status */
	if(this->battery_status != 0)
	{
		this->utils.storeBatteryStatus(this->battery_status,"prolog/battery.ecl");
		EC_word result = parser.battery(this->battery_status);
		internal_status = list(result,internal_status);
		println("The power level of the battery has been asserted in the KB");
	}
	/* Wifi status and wifi signal per position node*/
	if(this->wifi_status != 0)
	{
		this->utils.storeWifiStatus(this->wifi_status,"prolog/wifi.ecl");
		EC_word result = parser.wifi(this->wifi_status);
		internal_status = list(result,internal_status);
		println("the level of wifi signal has been asserted in the KB");

		this->utils.storeNodeWifiStrenght(this->position,this->wifi_status,"prolog/node_properties.ecl");
		EC_word wifinode = parser.wifiStrength(this->position,this->wifi_status);
		internal_status = list(wifinode,internal_status);
		println("the level of wifi signal associated with the current position node has been asserted in the KB");
	}

	return internal_status;
}

void ExecutionMonitoring::readTask(const WorkingMemoryChange & _wmc)
{   
    println("A new request for a task has been received");
    TaskPtr received_task = getMemoryEntry<Task>(_wmc.address);
	println("The src of the task is %s",_wmc.src.c_str());
	println("The type of the task is %s",_wmc.type.c_str());
    
    
    // received_task->status == NEW and this->task->status == CONFIRMED -> this->task->status = ABORTED and this->task = received_task
    if(received_task->status == NEW && this->task->status == eu::nifti::Planning::slice::CONFIRMED)
    {
        // è arrivata la richiesta di un nuovo task mentre un altro task era ancora in esecuzione
        // devo interrompere il task precedente
        println("******************** Human Task Intention ***********************");
        FailurePlanActionPtr interrupt = new FailurePlanAction();
        interrupt->name = "";
        interrupt->component = "";
        interrupt->time = 0;
        interrupt->status = eu::nifti::Planning::slice::PENDING;
        println("A new task has been requested while the previous one has still not been completed");
        this->task_aborted = true;
        // invio l'azione al componente che gestisce l'interruzione del piano

        addToWorkingMemory(newDataID(),interrupt);

//        bool flag = true;
//        while(flag)
//        {
//        	if(this->em_flag == 1)
//        	{
//        		flag = false;
//        	}
//        }
//        int count = 0;
//        while(!waitForNotificationActionFailure())
//        {
//        	if(count % 100000 == 0)
//        	{
//        		//println("**************** Action Failure Execution: WAITING *******************");
//        	}
//        	count++;
//        }


        // devo aggiornare lo stato del task precedentemente in esecuzione in ABORTED
        this->task->status = ABORTED;

        //sleep(5);


        overwriteWorkingMemory(this->id_task,this->task);
        println("The previous Task has been aborted");
        //riallocazione del nuovo task da eseguire
        this->id_task = _wmc.address.id;
        this->task = received_task;

        //t_Thread.join();
        m_Thread.join();

        println("******************** Task Interpretation ***********************");
        if(NavTaskPtr task = NavTaskPtr::dynamicCast(this->task))
        {
            println("Navigation Task has been received");
        	EC_functor visit = EC_functor((char*)"visit",1);
        	EC_functor sink_node = EC_functor((char*)"node",4);
        	EC_atom label = EC_atom((char*)task->node->label.c_str());
        	EC_word x = newvar();
        	EC_word y = newvar();
        	EC_word flag = newvar();
        	EC_word node = term(sink_node,label,x,y,flag);
        	this->myTask = term(visit,node);
        	println("the request of the Navigation task has been converted in a prolog term");
        }
        else if(MoveBaseTaskPtr task = MoveBaseTaskPtr::dynamicCast(this->task))
        {
            println("MoveBase Task has been received");
            if(task->command == MOVEFORWARD)
            {
                EC_atom motion = EC_atom((char*)"move_forward");
                this->myTask = motion;
            }
            else if(task->command == MOVELEFT)
            {
                EC_atom motion = EC_atom((char*)"move_left");
                this->myTask = motion;
            }
            else if(task->command == MOVERIGHT)
            {
                EC_atom motion = EC_atom((char*)"move_right");
                this->myTask = motion;
            }
            else if(task->command == MOVEBACK)
            {
                EC_atom motion = EC_atom((char*)"move_back");
                this->myTask = motion;
            }
            else if(task->command == TURNLEFT)
            {
                EC_atom motion = EC_atom((char*)"turn_left");
                this->myTask = motion;
            }
            else if(task->command == TURNRIGHT)
            {
                EC_atom motion = EC_atom((char*)"turn_right");
                this->myTask = motion;
            }
            else
            {
                println("Task type is unknown");
            }

        	println("the request of the MoveBase task has been converted in a prolog term");
        }
        else if(GUITaskPtr task = GUITaskPtr::dynamicCast(this->task))
        {
            println("GUI Task has been received");
            int xyID = getIdNode(task->x,task->y);

            std::string currentID = this->position->node->label;
            int cID;
            char c;
            sscanf(currentID.c_str(),"%c%d",&c,&cID);

            if(xyID == cID)
            {
            	EC_functor _goto = EC_functor((char*)"goto",2);
            	EC_word _x = EC_word(task->x);
            	EC_word _y = EC_word(task->y);
            	this->myTask = term(_goto,_x,_y);
            	println("<x,y> point is in the same region of the robot");
            }
            else if(xyID == -1)
            {	// Trovo il nodo più vicino ad <x,y> rispetto alla distanza euclidea
            	NodePtr temp = new Node();
            	temp->x = task->x;
            	temp->y = task->y;
            	NodePtr closest = neighborhood(temp,this->topo_graph);
            	EC_functor reach = EC_functor((char*)"reach",3);
            	EC_word node = parser.node(closest);
            	EC_word _x = EC_word(task->x);
            	EC_word _y = EC_word(task->y);
            	this->myTask = term(reach,_x,_y,node);
            	println("<x,y> point is in unknown space");
            }
            else if(xyID == -2)
            {
            	println("Error in calling the service convert_xy_to_id");
            	EC_functor _goto = EC_functor((char*)"goto",2);
            	EC_word _x = EC_word(task->x);
            	EC_word _y = EC_word(task->y);
            	this->myTask = term(_goto,_x,_y);
            }
            else
            {
            	NodePtr temp = new Node();
            	temp->x = task->x;
            	temp->y = task->y;
            	NodePtr closest = neighborhood(temp,this->topo_graph);
            	EC_functor reach = EC_functor((char*)"reach",3);
            	EC_word node = parser.node(closest);
            	EC_word _x = EC_word(task->x);
            	EC_word _y = EC_word(task->y);
            	this->myTask = term(reach,_x,_y,node);
            	println("<x,y> point is in known space");
            }
            println("the request of the GUI task has been converted in a prolog term");
        }
        else if(TopoGraphTaskPtr task = TopoGraphTaskPtr::dynamicCast(this->task))
        {
            println("TopoGraph Task has been received");
            EC_atom read_map = EC_atom((char*)task->name.c_str());
            this->myTask = read_map;
            println("the request of the TopoGraph Reader has been converted in a prolog term");
        }
        else if(VantagePointTaskPtr task = VantagePointTaskPtr::dynamicCast(this->task))
        {
            println("VantagePoint Task has been received");
            eu::nifti::env::CarObjectOfInterestPtr object = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(task->carWMA);
            NodePtr car_node = new Node();
            car_node->x = object->pose.x;
            car_node->y = object->pose.y;
            NodePtr closer = neighborhood(car_node,this->topo_graph);
            println("The closest topological node to the detected object is: %s",this->utils.to_string(closer).c_str());
            eu::nifti::env::VantagePointPtr best = bestVantagePoint(closer,object->vantagePoints);

            EC_functor approach = EC_functor((char*)"approach",2);
            EC_atom topo_node = EC_atom((char*)closer->label.c_str());

            EC_functor vp_node = EC_functor((char*)"vpnode",7);
            EC_atom l = EC_atom((char*)"vp");
            EC_word x = EC_word(best->pose.x);
            EC_word y = EC_word(best->pose.y);
            EC_word z = EC_word(best->pose.z);
            EC_word theta = EC_word(best->pose.theta);
            EC_word gain = EC_word(best->gain);
            EC_word flag = EC_word(0);
            EC_word vp_node_term = term(vp_node,l,x,y,z,theta,gain,flag);

            this->myTask = term(approach,topo_node,vp_node_term);
            println("the request of the VantagePoint task has been converted in a prolog term");

        }
        else if(GoHomeTaskPtr task = GoHomeTaskPtr::dynamicCast(this->task))
        {
            println("GoHome Task has been received");
            //EC_atom come_back = EC_atom((char*)task->name.c_str()); // "come_back"
            //int xyID = getIdNode(this->base->node->x,this->base->node->y);
            //std::string label = "n" + this->utils.to_string(xyID);
            //EC_functor come_back = EC_functor((char*)"come_back",1);

            //EC_atom label_node = EC_atom((char*)label.c_str());
            //this->myTask = term(come_back,label_node);
            EC_atom come_back = EC_atom((char*)"come_back");
            this->myTask = come_back;
            println("the request of the GoHome task has been converted in a prolog term");
        }
        else if(DetectGapTraversableTaskPtr task = DetectGapTraversableTaskPtr::dynamicCast(this->task))
        {
            println("DetectGapTraversable Task has been received");
            //EC_atom is_trav = EC_atom((char*)task->name.c_str()); // "detect_gap"
            EC_atom is_trav = EC_atom((char*)"detect_gap");
            this->myTask = is_trav;
            println("the request of the DetectGapTraversable task has been converted in a prolog term");
        }
        else if(TraverseGapTaskPtr task = TraverseGapTaskPtr::dynamicCast(this->task))
        {
            println("TraverseGap Task has been received");
            //EC_atom trav = EC_atom((char*)task->name.c_str()); // "traverse_gap"
            EC_atom trav = EC_atom((char*)"traverse_gap");
            this->myTask = trav;
            println("the request of the TraverseGap task has been converted in a prolog term");
        }
        else
        {
            println("Task type is unknown");
        }

        println("***************** Plan Generation ***********************************");
        EC_functor genPlan = EC_functor((char*)"task",2);
        EC_ref plan;
        EC_word query = term(genPlan,this->myTask,plan);
        this->engine.posta_goal(query);
        println("The plan generator has been called");
        this->engine.message_output(1);
        this->engine.message_output(2);
        myPlan = new Plan();
        id_plan = newDataID();

        int res = this->engine.resume();
        this->engine.message_output(1);
        this->engine.message_output(2);

        println("res values %d",res);

        if(res == EC_succeed)
        {
            //if the plan is empty
            if(((EC_word)plan).is_nil() == 0)
            {
                println("Empty Plan?: %d",((EC_word)plan).is_nil());
                this->task->status = eu::nifti::Planning::slice::REJECTED;
                overwriteWorkingMemory(this->id_task,this->task);
                println("The plan is empty and the task has been REJECTED");
            }
            else
            {
                this->task->status = eu::nifti::Planning::slice::CONFIRMED;
                overwriteWorkingMemory(this->id_task,this->task);
                println("The task has been CONFIRMED");


        	    TimelinePtr timeline = new Timeline();

        	    EC_word head, tail;

        	    ((EC_word)plan).is_list(head,tail);

        	    println("From eclipse prolog 2 ICE format: starting");
       	        EC_functor action_name;
       	        head.functor(&action_name);
       	        std::string component = action_name.name();
       	        println("Action type: %s",component.c_str());

       	        if(component.compare("start_goto_node") == 0)
       	        {
       	        	GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("start_read_topo") == 0)
       	        {
       	        	TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("end_read_topo") == 0)
       	        {
       	        	TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("start_build_topo") == 0)
       	        {
       	        	TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("end_build_topo") == 0)
       	        {
       	        	TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("start_func_map") == 0)
       	        {
       	        	FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("end_func_map") == 0)
       	        {
       	        	FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("start_rotation") == 0)
       	        {
       	        	RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("end_rotation") == 0)
       	        {
       	        	CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("move") == 0)
       	        {
       	        	FlipperActionPtr action = parser.eclipse2IceFlipperAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("lock") == 0)
       	        {
       	        	DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("unlock") == 0)
       	        {
       	        	DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("start_gap_detection") == 0)
       	        {
       	        	GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("end_gap_detection") == 0)
       	        {
       	        	GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("move_forward") == 0)
       	        {
       	        	MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("move_left") == 0)
       	        {
       	        	MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("move_right") == 0)
       	        {
       	        	MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("move_back") == 0)
       	        {
       	        	MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("turn_left") == 0)
       	        {
       	        	MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else if(component.compare("turn_right") == 0)
       	        {
       	        	MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
       	        	action->status = PENDING;
       	        	timeline->actions.push_back(action);
       	        	println("From eclipse prolog 2 ICE format: done");
       	        }
       	        else
       	        {
       	        	println("Convertion from eclipse 2 ICE is not possible: action type unknown");
       	        }

       	        while(tail.is_nil() != 0)
       	        {
       	        	EC_word new_head, new_tail;
       	        	tail.is_list(new_head,new_tail);

       	        	EC_functor action_name;
       	        	new_head.functor(&action_name);
       	        	std::string component = action_name.name();
       	        	println("Action type: %s",component.c_str());

       	        	if(component.compare("start_goto_node") == 0)
       	        	{
       	        		GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("start_read_topo") == 0)
       	        	{
       	        		TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("end_read_topo") == 0)
       	        	{
       	        		TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("start_build_topo") == 0)
       	        	{
       	        		TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("end_build_topo") == 0)
       	        	{
       	        		TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("start_func_map") == 0)
       	        	{
       	        		FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("end_func_map") == 0)
       	        	{
       	        		FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("start_rotation") == 0)
       	        	{
       	        		RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("end_rotation") == 0)
       	        	{
       	        		CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("move") == 0)
       	        	{
       	        		FlipperActionPtr action = parser.eclipse2IceFlipperAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("lock") == 0)
       	        	{
       	        		DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("unlock") == 0)
       	        	{
       	        		DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("start_gap_detection") == 0)
       	        	{
       	        		GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("end_gap_detection") == 0)
       	        	{
       	        		GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("move_forward") == 0)
       	        	{
       	        		MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("move_left") == 0)
       	        	{
       	        		MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("move_right") == 0)
       	        	{
       	        		MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("move_back") == 0)
       	        	{
       	        		MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("turn_left") == 0)
       	        	{
       	        		MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else if(component.compare("turn_right") == 0)
       	        	{
       	        		MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
       	        		action->status = PENDING;
       	        		timeline->actions.push_back(action);
       	        		println("From eclipse prolog 2 ICE format: done");
       	        	}
       	        	else
       	        	{
       	        		println("Convertion from eclipse 2 ICE is not possible: action type unknown");
       	        	}

       	        	tail = new_tail;
       	        }

       	        //computeOrientation(timeline);
       	        TimelinePtr new_timeline = computeOrientation2(timeline);

       	        myPlan->id = this->task->id;
       	        myPlan->goal = this->task;
       	        myPlan->bag.push_back(new_timeline);
       	        //myPlan->status = "PENDING";
       	        myPlan->status = PENDING;
       	        addToWorkingMemory(id_plan,myPlan);
       	        println("The plan has been stored in the WM");
            }
        }
        else
        {
        	println("*************** NEW-CONFIRMED Plan generation ERROR ***************************");
        }
    }
    else if(received_task->status == NEW && this->task->status == COMPLETED)
    {
        //il precedente task è ormai terminato con esito positivo
        // posso eseguire un altro task
        println("A new task has been requested and the previous one has already been completed");
        this->id_task = _wmc.address.id;
        this->task = received_task;

        println("******************** Task Interpretation ***********************");
        if(NavTaskPtr task = NavTaskPtr::dynamicCast(this->task))
        {
        	println("Navigation Task has been received");
        	EC_functor visit = EC_functor((char*)"visit",1);
        	EC_functor sink_node = EC_functor((char*)"node",4);
        	EC_atom label = EC_atom((char*)task->node->label.c_str());
        	EC_word x = newvar();
        	EC_word y = newvar();
        	EC_word flag = newvar();
        	EC_word node = term(sink_node,label,x,y,flag);
        	this->myTask = term(visit,node);
        	println("the request of the Navigation task has been converted in a prolog term");
        }
        else if(MoveBaseTaskPtr task = MoveBaseTaskPtr::dynamicCast(this->task))
        {
        	println("MoveBase Task has been received");
        	if(task->command == MOVEFORWARD)
        	{
        		EC_atom motion = EC_atom((char*)"move_forward");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVELEFT)
        	{
        		EC_atom motion = EC_atom((char*)"move_left");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVERIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"move_right");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVEBACK)
        	{
        		EC_atom motion = EC_atom((char*)"move_back");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNLEFT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_left");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNRIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_right");
        		this->myTask = motion;
        	}
        	else
        	{
        		println("Task type is unknown");
        	}

        	println("the request of the MoveBase task has been converted in a prolog term");
        }
        else if(GUITaskPtr task = GUITaskPtr::dynamicCast(this->task))
        {
        	println("GUI Task has been received");
        	int xyID = getIdNode(task->x,task->y);

        	std::string currentID = this->position->node->label;
        	int cID;
        	char c;
        	sscanf(currentID.c_str(),"%c%d",&c,&cID);

        	if(xyID == cID)
        	{
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        		println("<x,y> point is in the same region of the robot");
        	}
        	else if(xyID == -1)
        	{	// Trovo il nodo più vicino ad <x,y> rispetto alla distanza euclidea
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in unknown space");
        	}
        	else if(xyID == -2)
        	{
        		println("Error in calling the service convert_xy_to_id");
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        	}
        	else
        	{
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in known space");
        	}
        	println("the request of the GUI task has been converted in a prolog term");
        }
        else if(TopoGraphTaskPtr task = TopoGraphTaskPtr::dynamicCast(this->task))
        {
        	println("TopoGraph Task has been received");
        	EC_atom read_map = EC_atom((char*)task->name.c_str());
        	this->myTask = read_map;
        	println("the request of the TopoGraph Reader has been converted in a prolog term");
        }
        else if(VantagePointTaskPtr task = VantagePointTaskPtr::dynamicCast(this->task))
        {
        	println("VantagePoint Task has been received");
        	eu::nifti::env::CarObjectOfInterestPtr object = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(task->carWMA);
        	NodePtr car_node = new Node();
        	car_node->x = object->pose.x;
        	car_node->y = object->pose.y;
        	NodePtr closer = neighborhood(car_node,this->topo_graph);
        	println("The closest topological node to the detected object is: %s",this->utils.to_string(closer).c_str());
        	eu::nifti::env::VantagePointPtr best = bestVantagePoint(closer,object->vantagePoints);

        	EC_functor approach = EC_functor((char*)"approach",2);
        	EC_atom topo_node = EC_atom((char*)closer->label.c_str());

        	EC_functor vp_node = EC_functor((char*)"vpnode",7);
        	EC_atom l = EC_atom((char*)"vp");
        	EC_word x = EC_word(best->pose.x);
        	EC_word y = EC_word(best->pose.y);
        	EC_word z = EC_word(best->pose.z);
        	EC_word theta = EC_word(best->pose.theta);
        	EC_word gain = EC_word(best->gain);
        	EC_word flag = EC_word(0);
        	EC_word vp_node_term = term(vp_node,l,x,y,z,theta,gain,flag);

        	this->myTask = term(approach,topo_node,vp_node_term);
        	println("the request of the VantagePoint task has been converted in a prolog term");

        }
        else if(GoHomeTaskPtr task = GoHomeTaskPtr::dynamicCast(this->task))
        {
        	println("GoHome Task has been received");
        	//EC_atom come_back = EC_atom((char*)task->name.c_str()); // "come_back"
        	//int xyID = getIdNode(this->base->node->x,this->base->node->y);
	    	//std::string label = "n" + this->utils.to_string(xyID);
	    	//EC_functor come_back = EC_functor((char*)"come_back",1);

            //EC_atom label_node = EC_atom((char*)label.c_str());
            //this->myTask = term(come_back,label_node);
        	EC_atom come_back = EC_atom((char*)"come_back");
        	this->myTask = come_back;
        	println("the request of the GoHome task has been converted in a prolog term");
        }
        else if(DetectGapTraversableTaskPtr task = DetectGapTraversableTaskPtr::dynamicCast(this->task))
        {
        	println("DetectGapTraversable Task has been received");
        	//EC_atom is_trav = EC_atom((char*)task->name.c_str()); // "detect_gap"
        	EC_atom is_trav = EC_atom((char*)"detect_gap");
        	this->myTask = is_trav;
        	println("the request of the DetectGapTraversable task has been converted in a prolog term");
        }
        else if(TraverseGapTaskPtr task = TraverseGapTaskPtr::dynamicCast(this->task))
        {
        	println("TraverseGap Task has been received");
        	//EC_atom trav = EC_atom((char*)task->name.c_str()); // "traverse_gap"
        	EC_atom trav = EC_atom((char*)"traverse_gap");
        	this->myTask = trav;
        	println("the request of the TraverseGap task has been converted in a prolog term");
        }
        else
        {
        	println("Task type is unknown");
        }

        println("***************** Plan Generation ***********************************");
        EC_functor genPlan = EC_functor((char*)"task",2);
        EC_ref plan;
        EC_word query = term(genPlan,this->myTask,plan);
        this->engine.posta_goal(query);
        println("The plan generator has been called");
        this->engine.message_output(1);
        this->engine.message_output(2);
        myPlan = new Plan();
        id_plan = newDataID();

        int res = this->engine.resume();
        this->engine.message_output(1);
        this->engine.message_output(2);

        if(res == EC_succeed)
        {
        	//if the plan is empty
        	if(((EC_word)plan).is_nil() == 0)
        	{
        		println("Empty Plan?: %d",((EC_word)plan).is_nil());
        		this->task->status = eu::nifti::Planning::slice::REJECTED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The plan is empty and the task has been REJECTED");
        	}
        	else
        	{
        		this->task->status = eu::nifti::Planning::slice::CONFIRMED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The task has been CONFIRMED");


        		TimelinePtr timeline = new Timeline();

        		EC_word head, tail;

        		((EC_word)plan).is_list(head,tail);

        		println("From eclipse prolog 2 ICE format: starting");

        		EC_functor action_name;
        		head.functor(&action_name);
        		std::string component = action_name.name();
        		println("Action type: %s",component.c_str());

        		if(component.compare("start_goto_node") == 0)
        		{
        			GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_rotation") == 0)
        		{
        			RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_rotation") == 0)
        		{
        			CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move") == 0)
        		{
        			FlipperActionPtr action = parser.eclipse2IceFlipperAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("lock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("unlock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_forward") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_back") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else
        		{
        			println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        		}

        		while(tail.is_nil() != 0)
        		{
        			EC_word new_head, new_tail;
        			tail.is_list(new_head,new_tail);

        			EC_functor action_name;
        			new_head.functor(&action_name);
        			std::string component = action_name.name();
        			println("Action type: %s",component.c_str());

        			if(component.compare("start_goto_node") == 0)
        			{
        				GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_rotation") == 0)
        			{
        				RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_rotation") == 0)
        			{
        				CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move") == 0)
        			{
        				FlipperActionPtr action = parser.eclipse2IceFlipperAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("lock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("unlock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_forward") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_back") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else
        			{
        				println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        			}

        			tail = new_tail;
        		}

        		//computeOrientation(timeline);
        		TimelinePtr new_timeline = computeOrientation2(timeline);

        		myPlan->id = this->task->id;
        		myPlan->goal = this->task;
        		myPlan->bag.push_back(new_timeline);
        		//myPlan->status = "PENDING";
        		myPlan->status = PENDING;
        		addToWorkingMemory(id_plan,myPlan);
        		println("The plan has been stored in the WM");
        	}
        }
        else
        {
        	println("*************** NEW-COMPLETED Plan generation ERROR ***************************");
        }

    }
    else if(received_task->status == NEW && this->task->status == FAILED)
    {
        //il precedente task è terminato con un fallimento
        // posso eseguire un altro task
        println("A new task has been requested and the previous one has failed");
        this->id_task = _wmc.address.id;
        this->task = received_task;

        println("******************** Task Interpretation ***********************");
        if(NavTaskPtr task = NavTaskPtr::dynamicCast(this->task))
        {
        	println("Navigation Task has been received");
        	EC_functor visit = EC_functor((char*)"visit",1);
        	EC_functor sink_node = EC_functor((char*)"node",4);
        	EC_atom label = EC_atom((char*)task->node->label.c_str());
        	EC_word x = newvar();
        	EC_word y = newvar();
        	EC_word flag = newvar();
        	EC_word node = term(sink_node,label,x,y,flag);
        	this->myTask = term(visit,node);
        	println("the request of the Navigation task has been converted in a prolog term");
        }
        else if(MoveBaseTaskPtr task = MoveBaseTaskPtr::dynamicCast(this->task))
        {
        	println("MoveBase Task has been received");
        	if(task->command == MOVEFORWARD)
        	{
        		EC_atom motion = EC_atom((char*)"move_forward");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVELEFT)
        	{
        		EC_atom motion = EC_atom((char*)"move_left");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVERIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"move_right");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVEBACK)
        	{
        		EC_atom motion = EC_atom((char*)"move_back");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNLEFT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_left");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNRIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_right");
        		this->myTask = motion;
        	}
        	else
        	{
        		println("Task type is unknown");
        	}

        	println("the request of the MoveBase task has been converted in a prolog term");
        }
        else if(GUITaskPtr task = GUITaskPtr::dynamicCast(this->task))
        {
        	println("GUI Task has been received");
        	int xyID = getIdNode(task->x,task->y);

        	std::string currentID = this->position->node->label;
        	int cID;
        	char c;
        	sscanf(currentID.c_str(),"%c%d",&c,&cID);

        	if(xyID == cID)
        	{
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        		println("<x,y> point is in the same region of the robot");
        	}
        	else if(xyID == -1)
        	{	// Trovo il nodo più vicino ad <x,y> rispetto alla distanza euclidea
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in unknown space");
        	}
        	else if(xyID == -2)
        	{
        		println("Error in calling the service convert_xy_to_id");
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        	}
        	else
        	{
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in known space");
        	}
        	println("the request of the GUI task has been converted in a prolog term");
        }
        else if(TopoGraphTaskPtr task = TopoGraphTaskPtr::dynamicCast(this->task))
        {
        	println("TopoGraph Task has been received");
        	EC_atom read_map = EC_atom((char*)task->name.c_str());
        	this->myTask = read_map;
        	println("the request of the TopoGraph Reader has been converted in a prolog term");
        }
        else if(VantagePointTaskPtr task = VantagePointTaskPtr::dynamicCast(this->task))
        {
        	println("VantagePoint Task has been received");
        	eu::nifti::env::CarObjectOfInterestPtr object = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(task->carWMA);
        	NodePtr car_node = new Node();
        	car_node->x = object->pose.x;
        	car_node->y = object->pose.y;
        	NodePtr closer = neighborhood(car_node,this->topo_graph);
        	println("The closest topological node to the detected object is: %s",this->utils.to_string(closer).c_str());
        	eu::nifti::env::VantagePointPtr best = bestVantagePoint(closer,object->vantagePoints);

        	EC_functor approach = EC_functor((char*)"approach",2);
        	EC_atom topo_node = EC_atom((char*)closer->label.c_str());

        	EC_functor vp_node = EC_functor((char*)"vpnode",7);
        	EC_atom l = EC_atom((char*)"vp");
        	EC_word x = EC_word(best->pose.x);
        	EC_word y = EC_word(best->pose.y);
        	EC_word z = EC_word(best->pose.z);
        	EC_word theta = EC_word(best->pose.theta);
        	EC_word gain = EC_word(best->gain);
        	EC_word flag = EC_word(0);
        	EC_word vp_node_term = term(vp_node,l,x,y,z,theta,gain,flag);

        	this->myTask = term(approach,topo_node,vp_node_term);
        	println("the request of the VantagePoint task has been converted in a prolog term");

        }
        else if(GoHomeTaskPtr task = GoHomeTaskPtr::dynamicCast(this->task))
        {
        	println("GoHome Task has been received");
        	//EC_atom come_back = EC_atom((char*)task->name.c_str()); // "come_back"
        	//int xyID = getIdNode(this->base->node->x,this->base->node->y);
	    	//std::string label = "n" + this->utils.to_string(xyID);
	    	//EC_functor come_back = EC_functor((char*)"come_back",1);

            //EC_atom label_node = EC_atom((char*)label.c_str());
            //this->myTask = term(come_back,label_node);
        	EC_atom come_back = EC_atom((char*)"come_back");
        	this->myTask = come_back;
        	println("the request of the GoHome task has been converted in a prolog term");
        }
        else if(DetectGapTraversableTaskPtr task = DetectGapTraversableTaskPtr::dynamicCast(this->task))
        {
        	println("DetectGapTraversable Task has been received");
        	//EC_atom is_trav = EC_atom((char*)task->name.c_str()); // "detect_gap"
        	EC_atom is_trav = EC_atom((char*)"detect_gap");
        	this->myTask = is_trav;
        	println("the request of the DetectGapTraversable task has been converted in a prolog term");
        }
        else if(TraverseGapTaskPtr task = TraverseGapTaskPtr::dynamicCast(this->task))
        {
        	println("TraverseGap Task has been received");
        	//EC_atom trav = EC_atom((char*)task->name.c_str()); // "traverse_gap"
        	EC_atom trav = EC_atom((char*)"traverse_gap");
        	this->myTask = trav;
        	println("the request of the TraverseGap task has been converted in a prolog term");
        }
        else
        {
        	println("Task type is unknown");
        }

        println("***************** Plan Generation ***********************************");
        EC_functor genPlan = EC_functor((char*)"task",2);
        EC_ref plan;
        EC_word query = term(genPlan,this->myTask,plan);
        this->engine.posta_goal(query);
        println("The plan generator has been called");
        this->engine.message_output(1);
        this->engine.message_output(2);
        myPlan = new Plan();
        id_plan = newDataID();

        int res = this->engine.resume();
        this->engine.message_output(1);
        this->engine.message_output(2);

        if(res == EC_succeed)
        {
        	//if the plan is empty
        	if(((EC_word)plan).is_nil() == 0)
        	{
        		println("Empty Plan?: %d",((EC_word)plan).is_nil());
        		this->task->status = eu::nifti::Planning::slice::REJECTED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The plan is empty and the task has been REJECTED");
        	}
        	else
        	{
        		this->task->status = eu::nifti::Planning::slice::CONFIRMED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The task has been CONFIRMED");


        		TimelinePtr timeline = new Timeline();

        		EC_word head, tail;

        		((EC_word)plan).is_list(head,tail);

        		println("From eclipse prolog 2 ICE format: starting");

        		EC_functor action_name;
        		head.functor(&action_name);
        		std::string component = action_name.name();
        		println("Action type: %s",component.c_str());

        		if(component.compare("start_goto_node") == 0)
        		{
        			GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_rotation") == 0)
        		{
        			RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_rotation") == 0)
        		{
        			CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move") == 0)
        		{
        			FlipperActionPtr action = parser.eclipse2IceFlipperAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("lock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("unlock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_forward") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_back") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else
        		{
        			println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        		}

        		while(tail.is_nil() != 0)
        		{
        			EC_word new_head, new_tail;
        			tail.is_list(new_head,new_tail);

        			EC_functor action_name;
        			new_head.functor(&action_name);
        			std::string component = action_name.name();
        			println("Action type: %s",component.c_str());

        			if(component.compare("start_goto_node") == 0)
        			{
        				GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_rotation") == 0)
        			{
        				RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_rotation") == 0)
        			{
        				CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move") == 0)
        			{
        				FlipperActionPtr action = parser.eclipse2IceFlipperAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("lock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("unlock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_forward") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_back") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else
        			{
        				println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        			}

        			tail = new_tail;
        		}

        		//computeOrientation(timeline);
        		TimelinePtr new_timeline = computeOrientation2(timeline);

        		myPlan->id = this->task->id;
        		myPlan->goal = this->task;
        		myPlan->bag.push_back(new_timeline);
        		//myPlan->status = "PENDING";
        		myPlan->status = PENDING;
        		addToWorkingMemory(id_plan,myPlan);
        		println("The plan has been stored in the WM");
        	}
        }
        else
        {
        	println("*************** NEW-FAILED Plan generation ERROR ***************************");
        }

    }
    else if(received_task->status == NEW && this->task->status == eu::nifti::Planning::slice::REJECTED)
    {
        //il task precedente termina perchè non vi sono azioni da eseguire
        //posso eseguire un altro task
        println("A new task has been requested and the previous one has been rejected");
        this->id_task = _wmc.address.id;
        this->task = received_task;

        println("******************** Task Interpretation ***********************");
        if(NavTaskPtr task = NavTaskPtr::dynamicCast(this->task))
        {
        	println("Navigation Task has been received");
        	EC_functor visit = EC_functor((char*)"visit",1);
        	EC_functor sink_node = EC_functor((char*)"node",4);
        	EC_atom label = EC_atom((char*)task->node->label.c_str());
        	EC_word x = newvar();
        	EC_word y = newvar();
        	EC_word flag = newvar();
        	EC_word node = term(sink_node,label,x,y,flag);
        	this->myTask = term(visit,node);
        	println("the request of the Navigation task has been converted in a prolog term");
        }
        else if(MoveBaseTaskPtr task = MoveBaseTaskPtr::dynamicCast(this->task))
        {
        	println("MoveBase Task has been received");
        	if(task->command == MOVEFORWARD)
        	{
        		EC_atom motion = EC_atom((char*)"move_forward");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVELEFT)
        	{
        		EC_atom motion = EC_atom((char*)"move_left");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVERIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"move_right");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVEBACK)
        	{
        		EC_atom motion = EC_atom((char*)"move_back");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNLEFT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_left");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNRIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_right");
        		this->myTask = motion;
        	}
        	else
        	{
        		println("Task type is unknown");
        	}

        	println("the request of the MoveBase task has been converted in a prolog term");
        }
        else if(GUITaskPtr task = GUITaskPtr::dynamicCast(this->task))
        {
        	println("GUI Task has been received");
        	int xyID = getIdNode(task->x,task->y);

        	std::string currentID = this->position->node->label;
        	int cID;
        	char c;
        	sscanf(currentID.c_str(),"%c%d",&c,&cID);

        	if(xyID == cID)
        	{
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        		println("<x,y> point is in the same region of the robot");
        	}
        	else if(xyID == -1)
        	{	// Trovo il nodo più vicino ad <x,y> rispetto alla distanza euclidea
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in unknown space");
        	}
        	else if(xyID == -2)
        	{
        		println("Error in calling the service convert_xy_to_id");
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        	}
        	else
        	{
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in known space");
        	}
        	println("the request of the GUI task has been converted in a prolog term");
        }
        else if(TopoGraphTaskPtr task = TopoGraphTaskPtr::dynamicCast(this->task))
        {
        	println("TopoGraph Task has been received");
        	EC_atom read_map = EC_atom((char*)task->name.c_str());
        	this->myTask = read_map;
        	println("the request of the TopoGraph Reader has been converted in a prolog term");
        }
        else if(VantagePointTaskPtr task = VantagePointTaskPtr::dynamicCast(this->task))
        {
        	println("VantagePoint Task has been received");
        	eu::nifti::env::CarObjectOfInterestPtr object = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(task->carWMA);
        	NodePtr car_node = new Node();
        	car_node->x = object->pose.x;
        	car_node->y = object->pose.y;
        	NodePtr closer = neighborhood(car_node,this->topo_graph);
        	println("The closest topological node to the detected object is: %s",this->utils.to_string(closer).c_str());
        	eu::nifti::env::VantagePointPtr best = bestVantagePoint(closer,object->vantagePoints);

        	EC_functor approach = EC_functor((char*)"approach",2);
        	EC_atom topo_node = EC_atom((char*)closer->label.c_str());

        	EC_functor vp_node = EC_functor((char*)"vpnode",7);
        	EC_atom l = EC_atom((char*)"vp");
        	EC_word x = EC_word(best->pose.x);
        	EC_word y = EC_word(best->pose.y);
        	EC_word z = EC_word(best->pose.z);
        	EC_word theta = EC_word(best->pose.theta);
        	EC_word gain = EC_word(best->gain);
        	EC_word flag = EC_word(0);
        	EC_word vp_node_term = term(vp_node,l,x,y,z,theta,gain,flag);

        	this->myTask = term(approach,topo_node,vp_node_term);
        	println("the request of the VantagePoint task has been converted in a prolog term");

        }
        else if(GoHomeTaskPtr task = GoHomeTaskPtr::dynamicCast(this->task))
        {
        	println("GoHome Task has been received");
        	//EC_atom come_back = EC_atom((char*)task->name.c_str()); // "come_back"
        	//int xyID = getIdNode(this->base->node->x,this->base->node->y);
	    	//std::string label = "n" + this->utils.to_string(xyID);
	    	//EC_functor come_back = EC_functor((char*)"come_back",1);

            //EC_atom label_node = EC_atom((char*)label.c_str());
            //this->myTask = term(come_back,label_node);
        	EC_atom come_back = EC_atom((char*)"come_back");
        	this->myTask = come_back;
        	println("the request of the GoHome task has been converted in a prolog term");
        }
        else if(DetectGapTraversableTaskPtr task = DetectGapTraversableTaskPtr::dynamicCast(this->task))
        {
        	println("DetectGapTraversable Task has been received");
        	//EC_atom is_trav = EC_atom((char*)task->name.c_str()); // "detect_gap"
        	EC_atom is_trav = EC_atom((char*)"detect_gap");
        	this->myTask = is_trav;
        	println("the request of the DetectGapTraversable task has been converted in a prolog term");
        }
        else if(TraverseGapTaskPtr task = TraverseGapTaskPtr::dynamicCast(this->task))
        {
        	println("TraverseGap Task has been received");
        	//EC_atom trav = EC_atom((char*)task->name.c_str()); // "traverse_gap"
        	EC_atom trav = EC_atom((char*)"traverse_gap");
        	this->myTask = trav;
        	println("the request of the TraverseGap task has been converted in a prolog term");
        }
        else
        {
        	println("Task type is unknown");
        }

        println("***************** Plan Generation ***********************************");
        EC_functor genPlan = EC_functor((char*)"task",2);
        EC_ref plan;
        EC_word query = term(genPlan,this->myTask,plan);
        this->engine.posta_goal(query);
        println("The plan generator has been called");
        this->engine.message_output(1);
        this->engine.message_output(2);
        myPlan = new Plan();
        id_plan = newDataID();

        int res = this->engine.resume();
        this->engine.message_output(1);
        this->engine.message_output(2);

        if(res == EC_succeed)
        {
        	//if the plan is empty
        	if(((EC_word)plan).is_nil() == 0)
        	{
        		println("Empty Plan?: %d",((EC_word)plan).is_nil());
        		this->task->status = eu::nifti::Planning::slice::REJECTED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The plan is empty and the task has been REJECTED");
        	}
        	else
        	{
        		this->task->status = eu::nifti::Planning::slice::CONFIRMED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The task has been CONFIRMED");


        		TimelinePtr timeline = new Timeline();

        		EC_word head, tail;

        		((EC_word)plan).is_list(head,tail);

        		println("From eclipse prolog 2 ICE format: starting");

        		EC_functor action_name;
        		head.functor(&action_name);
        		std::string component = action_name.name();
        		println("Action type: %s",component.c_str());

        		if(component.compare("start_goto_node") == 0)
        		{
        			GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_rotation") == 0)
        		{
        			RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_rotation") == 0)
        		{
        			CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move") == 0)
        		{
        			FlipperActionPtr action = parser.eclipse2IceFlipperAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("lock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("unlock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_forward") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_back") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else
        		{
        			println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        		}

        		while(tail.is_nil() != 0)
        		{
        			EC_word new_head, new_tail;
        			tail.is_list(new_head,new_tail);

        			EC_functor action_name;
        			new_head.functor(&action_name);
        			std::string component = action_name.name();
        			println("Action type: %s",component.c_str());

        			if(component.compare("start_goto_node") == 0)
        			{
        				GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_rotation") == 0)
        			{
        				RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_rotation") == 0)
        			{
        				CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move") == 0)
        			{
        				FlipperActionPtr action = parser.eclipse2IceFlipperAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("lock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("unlock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_forward") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_back") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else
        			{
        				println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        			}

        			tail = new_tail;
        		}

        		//computeOrientation(timeline);
        		TimelinePtr new_timeline = computeOrientation2(timeline);

        		myPlan->id = this->task->id;
        		myPlan->goal = this->task;
        		myPlan->bag.push_back(new_timeline);
        		//myPlan->status = "PENDING";
        		myPlan->status = PENDING;
        		addToWorkingMemory(id_plan,myPlan);
        		println("The plan has been stored in the WM");
        	}
        }
        else
        {
        	println("*************** NEW-REJECTED Plan generation ERROR ***************************");
        }

    }
    else if(received_task->status == ADDED && this->task->status == eu::nifti::Planning::slice::CONFIRMED)
    {
        println("Mixed Initiative Modality: starting");
        this->mixed_status = 2;
        
        if(NavTaskPtr task = NavTaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: Navigation Task has been received");
		    EC_functor visit = EC_functor((char*)"visit",1);
		    EC_functor sink_node = EC_functor((char*)"node",4);
		    EC_atom label = EC_atom((char*)task->node->label.c_str());
		    EC_word x = newvar();
		    EC_word y = newvar();
		    EC_word flag = newvar();
		    EC_word node = term(sink_node,label,x,y,flag);
		    this->mixedInitiative = term(visit,node);
		    println("Mixed Initiative Modality: the request of the Navigation task has been converted in a prolog term");
	    }
	    else if(MoveBaseTaskPtr task = MoveBaseTaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: MoveBase Task has been received");
	        if(task->command == MOVEFORWARD)
	        {
	            EC_atom motion = EC_atom((char*)"move_forward");
	            this->mixedInitiative = motion;
	        }
	        else if(task->command == MOVELEFT)
	        {
	            EC_atom motion = EC_atom((char*)"move_left");
	            this->mixedInitiative = motion;
	        }
	        else if(task->command == MOVERIGHT)
	        {
	            EC_atom motion = EC_atom((char*)"move_right");
	            this->mixedInitiative = motion;
	        }
	        else if(task->command == MOVEBACK)
	        {
	            EC_atom motion = EC_atom((char*)"move_back");
	            this->mixedInitiative = motion;
	        }
	        else if(task->command == TURNLEFT)
	        {
	            EC_atom motion = EC_atom((char*)"turn_left");
	            this->mixedInitiative = motion;
	        }
	        else if(task->command == TURNRIGHT)
	        {
	            EC_atom motion = EC_atom((char*)"turn_right");
	            this->mixedInitiative = motion;
	        }
	        else
	        {
	            println("Mixed Initiative Modality: Task type is unknown");
	        } 
	        		
		    println("Mixed Initiative Modality: the request of the MoveBase task has been converted in a prolog term");
	    }
	    else if(GUITaskPtr task = GUITaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: GUI Task has been received");
	        println("GUI Task has been received");
	        int xyID = getIdNode(task->x,task->y);

	        std::string currentID = this->position->node->label;
	        int cID;
	        char c;
	        sscanf(currentID.c_str(),"%c%d",&c,&cID);

	        if(xyID == cID)
	        {
	        	EC_functor _goto = EC_functor((char*)"goto",2);
	        	EC_word _x = EC_word(task->x);
	        	EC_word _y = EC_word(task->y);
	        	this->mixedInitiative = term(_goto,_x,_y);
	        	println("<x,y> point is in the same region of the robot");
	        }
	        else if(xyID == -1)
	        {	// Trovo il nodo più vicino ad <x,y> rispetto alla distanza euclidea
	        	NodePtr temp = new Node();
	        	temp->x = task->x;
	        	temp->y = task->y;
	        	NodePtr closest = neighborhood(temp,this->topo_graph);
	        	EC_functor reach = EC_functor((char*)"reach",3);
	        	EC_word node = parser.node(closest);
	        	EC_word _x = EC_word(task->x);
	        	EC_word _y = EC_word(task->y);
	        	this->mixedInitiative = term(reach,_x,_y,node);
	        	println("<x,y> point is in unknown space");
	        }
	        else if(xyID == -2)
	        {
	        	println("Error in calling the service convert_xy_to_id");
	        	EC_functor _goto = EC_functor((char*)"goto",2);
	        	EC_word _x = EC_word(task->x);
	        	EC_word _y = EC_word(task->y);
	        	this->mixedInitiative = term(_goto,_x,_y);
	        }
	        else
	        {
	        	NodePtr temp = new Node();
	        	temp->x = task->x;
	        	temp->y = task->y;
	        	NodePtr closest = neighborhood(temp,this->topo_graph);
	        	EC_functor reach = EC_functor((char*)"reach",3);
	        	EC_word node = parser.node(closest);
	        	EC_word _x = EC_word(task->x);
	        	EC_word _y = EC_word(task->y);
	        	this->mixedInitiative = term(reach,_x,_y,node);
	        	println("<x,y> point is in known space");
	        }
		    println("Mixed Initiative Modality: the request of the GUI task has been converted in a prolog term");
	    }
	    else if(TopoGraphTaskPtr task = TopoGraphTaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: TopoGraph Task has been received");
	        EC_atom read_map = EC_atom((char*)task->name.c_str());
	        this->mixedInitiative = read_map;
	        println("Mixed Initiative Modality: the request of the TopoGraph Reader has been converted in a prolog term");
	    }
	    else if(VantagePointTaskPtr task = VantagePointTaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: VantagePoint Task has been received");
	        eu::nifti::env::CarObjectOfInterestPtr object = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(task->carWMA);
	        NodePtr car_node = new Node();
	        car_node->x = object->pose.x;
	        car_node->y = object->pose.y;
	        NodePtr closer = neighborhood(car_node,this->topo_graph);
	        println("Mixed Initiative Modality: The closest topological node to the detected object is: %s",this->utils.to_string(closer).c_str());
	        eu::nifti::env::VantagePointPtr best = bestVantagePoint(closer,object->vantagePoints);
	        
	        EC_functor approach = EC_functor((char*)"approach",2);
	        EC_atom topo_node = EC_atom((char*)closer->label.c_str());
	        
	        EC_functor vp_node = EC_functor((char*)"vpnode",7);
	        EC_atom l = EC_atom((char*)"vp"); 
            EC_word x = EC_word(best->pose.x);
            EC_word y = EC_word(best->pose.y);
            EC_word z = EC_word(best->pose.z);
            EC_word theta = EC_word(best->pose.theta);
            EC_word gain = EC_word(best->gain);
            EC_word flag = EC_word(0);
            EC_word vp_node_term = term(vp_node,l,x,y,z,theta,gain,flag);
            
            this->mixedInitiative = term(approach,topo_node,vp_node_term);
            println("Mixed Initiative Modality: the request of the VantagePoint task has been converted in a prolog term");
            
	    }
	    else if(GoHomeTaskPtr task = GoHomeTaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: GoHome Task has been received");
	        //EC_atom come_back = EC_atom((char*)task->name.c_str()); // "come_back"
	        //int xyID = getIdNode(this->base->node->x,this->base->node->y);
	    	//std::string label = "n" + this->utils.to_string(xyID);
	    	//EC_functor come_back = EC_functor((char*)"come_back",1);

            //EC_atom label_node = EC_atom((char*)label.c_str());
            //this->mixedInitiative = term(come_back,label_node);
	        EC_atom come_back = EC_atom((char*)"come_back");
	        this->myTask = come_back;
	        println("Mixed Initiative Modality: the request of the GoHome task has been converted in a prolog term");
	    }
	    else if(DetectGapTraversableTaskPtr task = DetectGapTraversableTaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: DetectGapTraversable Task has been received");
	        //EC_atom is_trav = EC_atom((char*)task->name.c_str()); // "detect_gap"
	        EC_atom is_trav = EC_atom((char*)"detect_gap");
	        this->mixedInitiative = is_trav;
	        println("Mixed Initiative Modality: the request of the DetectGapTraversable task has been converted in a prolog term");
	    }
	    else if(TraverseGapTaskPtr task = TraverseGapTaskPtr::dynamicCast(received_task))
	    {
	        println("Mixed Initiative Modality: TraverseGap Task has been received");
	        //EC_atom trav = EC_atom((char*)task->name.c_str()); // "traverse_gap"
	        EC_atom trav = EC_atom((char*)"traverse_gap");
	        this->mixedInitiative = trav;
	        println("Mixed Initiative Modality: the request of the TraverseGap task has been converted in a prolog term");
	    }
	    else
	    {
	        println("Mixed Initiative Modality: Task type is unknown");
	    }
	    
	    println("Mixed Initiative Modality: Ended");
    }
    else
    {
        this->id_task = _wmc.address.id;
        this->task = received_task;

        println("******************** Task Interpretation ***********************");
        if(NavTaskPtr task = NavTaskPtr::dynamicCast(this->task))
        {
        	println("Navigation Task has been received");
        	EC_functor visit = EC_functor((char*)"visit",1);
        	EC_functor sink_node = EC_functor((char*)"node",4);
        	EC_atom label = EC_atom((char*)task->node->label.c_str());
        	EC_word x = newvar();
        	EC_word y = newvar();
        	EC_word flag = newvar();
        	EC_word node = term(sink_node,label,x,y,flag);
        	this->myTask = term(visit,node);
        	println("the request of the Navigation task has been converted in a prolog term");
        }
        else if(MoveBaseTaskPtr task = MoveBaseTaskPtr::dynamicCast(this->task))
        {
        	println("MoveBase Task has been received");
        	if(task->command == MOVEFORWARD)
        	{
        		EC_atom motion = EC_atom((char*)"move_forward");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVELEFT)
        	{
        		EC_atom motion = EC_atom((char*)"move_left");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVERIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"move_right");
        		this->myTask = motion;
        	}
        	else if(task->command == MOVEBACK)
        	{
        		EC_atom motion = EC_atom((char*)"move_back");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNLEFT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_left");
        		this->myTask = motion;
        	}
        	else if(task->command == TURNRIGHT)
        	{
        		EC_atom motion = EC_atom((char*)"turn_right");
        		this->myTask = motion;
        	}
        	else
        	{
        		println("Task type is unknown");
        	}

        	println("the request of the MoveBase task has been converted in a prolog term");
        }
        else if(GUITaskPtr task = GUITaskPtr::dynamicCast(this->task))
        {
        	println("GUI Task has been received");
        	int xyID = getIdNode(task->x,task->y);

        	std::string currentID = this->position->node->label;
        	int cID;
        	char c;
        	sscanf(currentID.c_str(),"%c%d",&c,&cID);

        	if(xyID == cID)
        	{
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        		println("<x,y> point is in the same region of the robot");
        	}
        	else if(xyID == -1)
        	{	// Trovo il nodo più vicino ad <x,y> rispetto alla distanza euclidea
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in unknown space");
        	}
        	else if(xyID == -2)
        	{
        		println("Error in calling the service convert_xy_to_id");
        		EC_functor _goto = EC_functor((char*)"goto",2);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(_goto,_x,_y);
        	}
        	else
        	{
        		NodePtr temp = new Node();
        		temp->x = task->x;
        		temp->y = task->y;
        		NodePtr closest = neighborhood(temp,this->topo_graph);
        		EC_functor reach = EC_functor((char*)"reach",3);
        		EC_word node = parser.node(closest);
        		EC_word _x = EC_word(task->x);
        		EC_word _y = EC_word(task->y);
        		this->myTask = term(reach,_x,_y,node);
        		println("<x,y> point is in known space");
        	}
        	println("the request of the GUI task has been converted in a prolog term");
        }
        else if(TopoGraphTaskPtr task = TopoGraphTaskPtr::dynamicCast(this->task))
        {
        	println("TopoGraph Task has been received");
        	EC_atom read_map = EC_atom((char*)task->name.c_str());
        	this->myTask = read_map;
        	println("the request of the TopoGraph Reader has been converted in a prolog term");
        }
        else if(VantagePointTaskPtr task = VantagePointTaskPtr::dynamicCast(this->task))
        {
        	println("VantagePoint Task has been received");
        	eu::nifti::env::CarObjectOfInterestPtr object = getMemoryEntry<eu::nifti::env::CarObjectOfInterest>(task->carWMA);
        	NodePtr car_node = new Node();
        	car_node->x = object->pose.x;
        	car_node->y = object->pose.y;
        	NodePtr closer = neighborhood(car_node,this->topo_graph);
        	println("The closest topological node to the detected object is: %s",this->utils.to_string(closer).c_str());
        	eu::nifti::env::VantagePointPtr best = bestVantagePoint(closer,object->vantagePoints);

        	EC_functor approach = EC_functor((char*)"approach",2);
        	EC_atom topo_node = EC_atom((char*)closer->label.c_str());

        	EC_functor vp_node = EC_functor((char*)"vpnode",7);
        	EC_atom l = EC_atom((char*)"vp");
        	EC_word x = EC_word(best->pose.x);
        	EC_word y = EC_word(best->pose.y);
        	EC_word z = EC_word(best->pose.z);
        	EC_word theta = EC_word(best->pose.theta);
        	EC_word gain = EC_word(best->gain);
        	EC_word flag = EC_word(0);
        	EC_word vp_node_term = term(vp_node,l,x,y,z,theta,gain,flag);

        	this->myTask = term(approach,topo_node,vp_node_term);
        	println("the request of the VantagePoint task has been converted in a prolog term");

        }
        else if(GoHomeTaskPtr task = GoHomeTaskPtr::dynamicCast(this->task))
        {
        	println("GoHome Task has been received");
        	//EC_atom come_back = EC_atom((char*)task->name.c_str()); // "come_back"
        	//int xyID = getIdNode(this->base->node->x,this->base->node->y);
	    	//std::string label = "n" + this->utils.to_string(xyID);
	    	//EC_functor come_back = EC_functor((char*)"come_back",1);

            //EC_atom label_node = EC_atom((char*)label.c_str());
            //this->myTask = term(come_back,label_node);
        	EC_atom come_back = EC_atom((char*)"come_back");
        	this->myTask = come_back;
        	println("the request of the GoHome task has been converted in a prolog term");
        }
        else if(DetectGapTraversableTaskPtr task = DetectGapTraversableTaskPtr::dynamicCast(this->task))
        {
        	println("DetectGapTraversable Task has been received");
        	//EC_atom is_trav = EC_atom((char*)task->name.c_str()); // "detect_gap"
        	EC_atom is_trav = EC_atom((char*)"detect_gap");
        	this->myTask = is_trav;
        	println("the request of the DetectGapTraversable task has been converted in a prolog term");
        }
        else if(TraverseGapTaskPtr task = TraverseGapTaskPtr::dynamicCast(this->task))
        {
        	println("TraverseGap Task has been received");
        	//EC_atom trav = EC_atom((char*)task->name.c_str()); // "traverse_gap"
        	EC_atom trav = EC_atom((char*)"traverse_gap");
        	this->myTask = trav;
        	println("the request of the TraverseGap task has been converted in a prolog term");
        }
        else
        {
        	println("Task type is unknown");
        }

        println("***************** Plan Generation ***********************************");
        EC_functor genPlan = EC_functor((char*)"task",2);
        EC_ref plan;
        EC_word query = term(genPlan,this->myTask,plan);
        this->engine.posta_goal(query);
        println("The plan generator has been called");
        this->engine.message_output(1);
        this->engine.message_output(2);
        myPlan = new Plan();
        id_plan = newDataID();

        int res = this->engine.resume();
        this->engine.message_output(1);
        this->engine.message_output(2);

        if(res == EC_succeed)
        {
        	//if the plan is empty
        	if(((EC_word)plan).is_nil() == 0)
        	{
        		println("Empty Plan?: %d",((EC_word)plan).is_nil());
        		this->task->status = eu::nifti::Planning::slice::REJECTED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The plan is empty and the task has been REJECTED");
        	}
        	else
        	{
        		this->task->status = eu::nifti::Planning::slice::CONFIRMED;
        		overwriteWorkingMemory(this->id_task,this->task);
        		println("The task has been CONFIRMED");


        		TimelinePtr timeline = new Timeline();

        		EC_word head, tail;

        		((EC_word)plan).is_list(head,tail);

        		println("From eclipse prolog 2 ICE format: starting");

        		EC_functor action_name;
        		head.functor(&action_name);
        		std::string component = action_name.name();
        		println("Action type: %s",component.c_str());

        		if(component.compare("start_goto_node") == 0)
        		{
        			GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_read_topo") == 0)
        		{
        			TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_build_topo") == 0)
        		{
        			TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_func_map") == 0)
        		{
        			FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_rotation") == 0)
        		{
        			RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_rotation") == 0)
        		{
        			CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move") == 0)
        		{
        			FlipperActionPtr action = parser.eclipse2IceFlipperAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("lock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("unlock") == 0)
        		{
        			DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("start_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("end_gap_detection") == 0)
        		{
        			GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_forward") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("move_back") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_left") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else if(component.compare("turn_right") == 0)
        		{
        			MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(head);
        			action->status = PENDING;
        			timeline->actions.push_back(action);
        			println("From eclipse prolog 2 ICE format: done");
        		}
        		else
        		{
        			println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        		}

        		while(tail.is_nil() != 0)
        		{
        			EC_word new_head, new_tail;
        			tail.is_list(new_head,new_tail);

        			EC_functor action_name;
        			new_head.functor(&action_name);
        			std::string component = action_name.name();
        			println("Action type: %s",component.c_str());

        			if(component.compare("start_goto_node") == 0)
        			{
        				GoToNodeActionPtr action = parser.eclipse2IceGoToNodeAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_read_topo") == 0)
        			{
        				TopoGraphWriterActionPtr action = parser.eclipse2IceTopoGraphWriterAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_build_topo") == 0)
        			{
        				TogoGraphBuilderActionPtr action = parser.eclipse2IceTopoGraphBuilderAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_func_map") == 0)
        			{
        				FunctionalMappingActionPtr action = parser.eclipse2IceFuncMappingAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_rotation") == 0)
        			{
        				RotatingLaserActionPtr action = parser.eclipse2IceRotatingLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_rotation") == 0)
        			{
        				CenterLaserActionPtr action = parser.eclipse2IceCenterLaserAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move") == 0)
        			{
        				FlipperActionPtr action = parser.eclipse2IceFlipperAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("lock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("unlock") == 0)
        			{
        				DifferentialActionPtr action = parser.eclipse2IceDifferentialAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("start_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("end_gap_detection") == 0)
        			{
        				GapDetectionActionPtr action = parser.eclipse2IceGapDetectionAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_forward") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("move_back") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_left") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else if(component.compare("turn_right") == 0)
        			{
        				MoveBaseActionPtr action = parser.eclipse2IceMoveBaseAction(new_head);
        				action->status = PENDING;
        				timeline->actions.push_back(action);
        				println("From eclipse prolog 2 ICE format: done");
        			}
        			else
        			{
        				println("Convertion from eclipse 2 ICE is not possible: action type unknown");
        			}

        			tail = new_tail;
        		}

        		//computeOrientation(timeline);
        		TimelinePtr new_timeline = computeOrientation2(timeline);

        		myPlan->id = this->task->id;
        		myPlan->goal = this->task;
        		myPlan->bag.push_back(new_timeline);
        		//myPlan->status = "PENDING";
        		myPlan->status = PENDING;
        		addToWorkingMemory(id_plan,myPlan);
        		println("The plan has been stored in the WM");
        	}
        }
        else
        {
        	println("*************** ADDED-CONFIRMED Plan generation ERROR ***************************");
        }

    }
}

void ExecutionMonitoring::readPlan(const WorkingMemoryChange& _wmc)
{
    println("Filter callback: a new plan has been stored in WM");

    PlanPtr plan = getMemoryEntry<Plan>(_wmc.address);
    
    //this->exe_monitor = false;

    int n_timelines = plan->bag.size();
    
    this->bag = nil();
    
    for(int i = 0; i < n_timelines; i++)
	{
			TimelinePtr timeline = new Timeline();
			timeline = plan->bag[i];
			
			this->_plan = nil();
	
			int n_actions = timeline->actions.size();

			for(int i = n_actions -1; i >= 0; i--)
			{
				ActionPtr temp = new Action();
				temp = timeline->actions[i];
				if(GoToNodeActionPtr goto_action = GoToNodeActionPtr::dynamicCast(temp))
				{
					EC_word _temp = gotoNodeAction(goto_action);
					this->_plan = list(_temp,this->_plan);
				}
				else if(TopoGraphWriterActionPtr topo_writer_action = TopoGraphWriterActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseTopoGraphWriterAction(topo_writer_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(TogoGraphBuilderActionPtr topo_builder_action = TogoGraphBuilderActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseTopoGraphBuilderAction(topo_builder_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(FunctionalMappingActionPtr func_map_action = FunctionalMappingActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseFunctMappingAction(func_map_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(RotatingLaserActionPtr rot_laser_action = RotatingLaserActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseRotatingLaserAction(rot_laser_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(CenterLaserActionPtr cent_laser_action = CenterLaserActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseCenterLaserAction(cent_laser_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(FlipperActionPtr flipper_action = FlipperActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseFlipperAction(flipper_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(DifferentialActionPtr diff_action = DifferentialActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseDifferentialAction(diff_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(GapDetectionActionPtr gap_action = GapDetectionActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseGapDetectionAction(gap_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(MoveBaseActionPtr motion_action = MoveBaseActionPtr::dynamicCast(temp))
				{
				    EC_word _temp = parser.ice2EclipseMoveBaseAction(motion_action);
				    this->_plan = list(_temp,this->_plan);
				}
				else if(AutoModeActionPtr auto_action = AutoModeActionPtr::dynamicCast(temp))
				{
					EC_word _temp = parser.ice2EclipseAutoModeAction(auto_action);
					this->_plan = list(_temp,this->_plan);
				}
				else
				{
				    println("Read Plan: Action Unknown");
				}
			}
			
			this->bag = list(this->_plan,this->bag);
		
	}
		
	println("The plan has been converted in a prolog term"); 

	println("******************* Plan Execution START ****************************");
	EC_functor em = EC_functor((char*)"em3",2);
	EC_ref result;
	EC_word task = term(em,this->_plan,result);
	this->engine.posta_goal(task);
	
	EC_ref FromEclipse;
	EC_resume(FromEclipse);
	println("executing the first action");
	this->engine.message_output(1);
	this->engine.message_output(2);
	
	int status_flag = -1;
	
    EC_functor action_name;
    ((EC_word)FromEclipse).functor(&action_name);
    std::string component = action_name.name();
    
    eclipse_prolog_msgs::ActionScheduled as;
    
    if(component.compare("start_goto_node") == 0)
    {
        this->current_action = parser.eclipse2IceGoToNodeAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("start_read_topo") == 0)
    {
        this->current_action = parser.eclipse2IceTopoGraphWriterAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("end_read_topo") == 0)
    {
        this->current_action = parser.eclipse2IceTopoGraphWriterAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("start_build_topo") == 0)
    {
        this->current_action = parser.eclipse2IceTopoGraphBuilderAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("end_build_topo") == 0)
    {
        this->current_action = parser.eclipse2IceTopoGraphBuilderAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("start_func_map") == 0)
    {
        this->current_action = parser.eclipse2IceFuncMappingAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("end_func_map") == 0)
    {
        this->current_action = parser.eclipse2IceFuncMappingAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("start_rotation") == 0)
    {
        this->current_action = parser.eclipse2IceRotatingLaserAction(FromEclipse);
        println("The first action has been parsed: start_rotation");
    }
    else if(component.compare("end_rotation") == 0)
    {
        this->current_action = parser.eclipse2IceCenterLaserAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("move") == 0)
    {
        this->current_action = parser.eclipse2IceFlipperAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("lock") == 0)
    {
        this->current_action = parser.eclipse2IceDifferentialAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("unlock") == 0)
    {
        this->current_action = parser.eclipse2IceDifferentialAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("start_gap_detection") == 0)
    {
        this->current_action = parser.eclipse2IceGapDetectionAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("end_gap_detection") == 0)
    {
        this->current_action = parser.eclipse2IceGapDetectionAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("move_forward") == 0)
    {
        this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("move_left") == 0)
    {
        this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("move_right") == 0)
    {
        this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("move_back") == 0)
    {
        this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("turn_left") == 0)
    {
        this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
        println("The first action has been parsed");
    }
    else if(component.compare("turn_right") == 0)
    {
        this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
        println("The first action has been parsed");
    }
    else
    {
        println("Unknown Action");
    }
    
    this->current_action->status = PENDING;
    
    //std::string id_action = newDataID();
    //addToWorkingMemory(id_action,current_action); //Not in my WM
	//println("adding the first action in the WM");
	
	as.action = this->utils.to_string(this->current_action);
	as.status = "pending";
	this->actions_pub.publish(as);
	println("publishing the first action in the topic");

	performAction(this->current_action);

	int count = 0;
	while(!waitForActionExecution())
	{
		if(count % 100000 == 0)
		{
			//println("**************** Action Execution: WAITING *******************");
		}
		count++;
	}

	//println("**************** Action Execution: DONE *******************");
	if(this->current_action->status == EXECUTED)
	{
	    status_flag = 0;
	    as.status = "executed";
	    this->actions_pub.publish(as);
	    println("**************** Action Execution: EXECUTED *******************");
	}
	else if(this->current_action->status == FAILED)
	{
	    status_flag = 1;
	    as.status = "failed";
	    this->actions_pub.publish(as);
	    println("**************** Action Execution: FAILED *******************");
	}
	else
	{
	    println("**************** Action Execution: ERROR *******************");
	}
	
	if(status_flag == 0 && this->mixed_status == 2)
	{
	    println("The user is requesting to switch in the mixed initiative modality");
	    status_flag = this->mixed_status;
	    this->mixed_status = -1;
	}
	else
	{
	    println("No mixed initiative modality has been requested");
	}

	println("status_flag %s",this->utils.to_string(status_flag).c_str());

	println("********************* EC_YIELD CYCLE ****************************");	
	
	//int res = EC_resume(list(EC_word(status_flag),list(updateRobotStatus(),this->mixedInitiative)),FromEclipse);

	//println("EC Yield: res %d",res);

	while(EC_resume(list(EC_word(status_flag),list(updateRobotStatus(),this->mixedInitiative)),FromEclipse) == EC_yield)
	{
	    this->mixedInitiative = nil();
	    
	    EC_functor action_name;
	    ((EC_word)FromEclipse).functor(&action_name);
	    std::string component = action_name.name();
	    
	    if(component.compare("start_goto_node") == 0)
        {
            this->current_action = parser.eclipse2IceGoToNodeAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("start_read_topo") == 0)
        {
            this->current_action = parser.eclipse2IceTopoGraphWriterAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("end_read_topo") == 0)
        {
            this->current_action = parser.eclipse2IceTopoGraphWriterAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("start_build_topo") == 0)
        {
            this->current_action = parser.eclipse2IceTopoGraphBuilderAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("end_build_topo") == 0)
        {
            this->current_action = parser.eclipse2IceTopoGraphBuilderAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("start_func_map") == 0)
        {
            this->current_action = parser.eclipse2IceFuncMappingAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("end_func_map") == 0)
        {
            this->current_action = parser.eclipse2IceFuncMappingAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("start_rotation") == 0)
        {
            this->current_action = parser.eclipse2IceRotatingLaserAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("end_rotation") == 0)
        {
            this->current_action = parser.eclipse2IceCenterLaserAction(FromEclipse);
            println("The first action has been parsed");
        }
        else if(component.compare("move") == 0)
        {
            this->current_action = parser.eclipse2IceFlipperAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("lock") == 0)
        {
            this->current_action = parser.eclipse2IceDifferentialAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("unlock") == 0)
        {
            this->current_action = parser.eclipse2IceDifferentialAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("start_gap_detection") == 0)
        {
            this->current_action = parser.eclipse2IceGapDetectionAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("end_gap_detection") == 0)
        {
            this->current_action = parser.eclipse2IceGapDetectionAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("move_forward") == 0)
        {
            this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("move_left") == 0)
        {
            this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("move_right") == 0)
        {
            this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("move_back") == 0)
        {
            this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("turn_left") == 0)
        {
            this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
            println("The next action has been parsed");
        }
        else if(component.compare("turn_right") == 0)
        {
            this->current_action = parser.eclipse2IceMoveBaseAction(FromEclipse);
            println("The next action has been parsed");
        }
        else
        {
            println("Execution phase: Unknown Action");
        }
        
        this->current_action->status = PENDING;
	    
	    //id_action = newDataID();
	    //addToWorkingMemory(id_action,current_action); Not in my WM
	    //println("the next action has been stored in the WM");
	    
	    as.action = this->utils.to_string(this->current_action);
	    as.status = "pending";
	    this->actions_pub.publish(as);
	    println("the next action has been published in the topic");
	    
	    performAction(this->current_action);
	
	    count = 0;
	    while(!waitForActionExecution())
	    {
	        if(count % 100000 == 0)
	        {
	        	//println("**************** Action Execution: WAITING *******************");
	        }
	        count++;
	    }
	
	    println("**************** Action Execution: DONE *******************");
	    if(this->current_action->status == EXECUTED)
	    {
	        status_flag = 0;
	        as.status = "executed";
	        this->actions_pub.publish(as);
		    println("**************** Action Execution: EXECUTED *******************");
	    }
	    else if(this->current_action->status == FAILED)
	    {
	        status_flag = 1;
	        as.status = "failed";
	        this->actions_pub.publish(as);
		    println("**************** Action Execution: FAILED *******************");
	    }
	    else
	    {
	        println("**************** Action Execution: ERROR *******************");
	    }
	
	    if(status_flag == 0 && this->mixed_status == 2)
	    {
	        println("The user is requesting to switch in the mixed initiative modality");
	        status_flag = this->mixed_status;
	        this->mixed_status = -1;
	    }
	    else
	    {
	        println("No mixed initiative modality has been requested");
	    }
	}
	
	long em_flag = -1;
	((EC_word)result).is_long(&em_flag);
	
	println("em_flag: %s",this->utils.to_string(em_flag).c_str());
	println("taks_aborted flag: %d",(int)this->task_aborted);

	if(em_flag == 0)
	{
		println("em_flag = 0");
	    myPlan->status = COMPLETED;
	    overwriteWorkingMemory(id_plan,myPlan);
	    println("the status of the plan has been changed in the WM");  
	    println("Plan COMPLETED");
	    this->task->status = COMPLETED;
	    overwriteWorkingMemory(this->id_task,this->task);
	    println("Task COMPLETED");
	}
	else if(em_flag == 1 && this->task_aborted == true)
	{
		println("em_flag = 1 and task_aborted = true");
	    this->task_aborted = false;
	    myPlan->status = ABORTED;
	    overwriteWorkingMemory(id_plan,myPlan);
	    println("the status of the plan has been changed in the WM");
	    println("Plan ABORTED");
	}
	else
	{
		println("em_flag = 1 and task_aborted = false");
	    myPlan->status = FAILED;
	    overwriteWorkingMemory(id_plan,myPlan);
	    println("The status of the plan has been changed in the WM");
	    println("Plan FAILED");
	    this->task->status = FAILED;
	    overwriteWorkingMemory(this->id_task,this->task);
	    println("Task FAILED");  
	}
	
	//TODO: il piano è comletato prima ancora che la posizione correntre venga stabilita
	//this->engine.my_retract(this->current_position);
	//this->current_position = in(this->position);
	//this->engine.my_assert(this->current_position);
	//println("the current position of the robot has been updated in the knowledge base");
	//this->exe_monitor = true;
	println("******************* Plan Execution FINISH ****************************");
	//t_Thread.detach();
}


void ExecutionMonitoring::performAction(ActionPtr action)
{
    if(GoToNodeActionPtr current = GoToNodeActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the Navigation Component");
    }
    else if(MoveBaseActionPtr current = MoveBaseActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the Navigation Component");
    }
    else if(DifferentialActionPtr current = DifferentialActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the Locomotion Component");
    }
    else if(FlipperActionPtr current = FlipperActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the Locomotion Component");
    }
    else if(RotatingLaserActionPtr current = RotatingLaserActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the Lidar Component");
    }
    else if(CenterLaserActionPtr current = CenterLaserActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the Lidar Component");
    }
    else if(TogoGraphBuilderActionPtr current = TogoGraphBuilderActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the TopoSegBuilder Component");
    }
    else if(TopoGraphWriterActionPtr current = TopoGraphWriterActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the TopoSegReader Component");
    }
    else if(FunctionalMappingActionPtr current = FunctionalMappingActionPtr::dynamicCast(action))
    {
        addToWorkingMemory(newDataID(),current);
        println("The current action has been sent to the FunctionalMapping Component");
    }
    else if(GapDetectionActionPtr current = GapDetectionActionPtr::dynamicCast(action))
    {
    	addToWorkingMemory(newDataID(),current);
    	println("The current action has been sent to the GapDetection Component");
    }
    else
    {
        println("Physical Execution: Action Unknown");
    }
}

void ExecutionMonitoring::runComponent()
{
	println("************************************************************");
	println("********** ExecutionMonitoring ROS CAST Component **********");
	println("******************* Status: running ************************");
	println("************************************************************");
    
}

extern "C" 
{
	CASTComponentPtr newComponent()
	{
    	return new ExecutionMonitoring();
  	}
}
