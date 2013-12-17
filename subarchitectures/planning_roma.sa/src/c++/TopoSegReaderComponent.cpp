#include <string>
#include <TopoSegReaderComponent.hpp>

/* this function converts a double ia a string */
string TopoSegReaderComponent::to_string(double value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

/* this function converts a ICE Node in a string */
string TopoSegReaderComponent::to_string(NodePtr n)
{
	string node = "node(" + n->label + ",";
	stringstream _x;
	_x << n->x;
	node = node + _x.str() + ",";
	stringstream _y;
	_y << n->y;
	node = node + _y.str() + ",";
	stringstream _flag;
	_flag << n->flag;
	node = node + _flag.str() + ")";
	return node;
}

/* this function converts a ICE Edge in a string */
string TopoSegReaderComponent::to_string(EdgePtr e)
{
	string s;
	string _edge = "edge(";
	s = _edge + "edge(" + to_string(e->a) + "," + to_string(e->b) + "," + "1)" + ").";
	return s;
}


NodePtr TopoSegReaderComponent::getNode(int label, GraphPtr graph)
{
    NodePtr node = new Node();
    string token = "n" + to_string(label);
    for(unsigned int i = 0; i < graph->nodes.size(); i++)
    {
        if(token.compare(graph->nodes[i]->label) == 0)
        {
            node = graph->nodes[i];
            break;
        }
    }
    
    return node;
}

bool TopoSegReaderComponent::waitForSubscriber()
{
    //if(this->current_topo_centers.polygon.points.size() != 0 && this->current_conn_graph.polygon.points.size() != 0)
	//Relaxed
	if(this->current_topo_centers.polygon.points.size() != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}


void TopoSegReaderComponent::listen_current_topo_centers(const PolygonStamped::ConstPtr& msg)
{
    //ROS_INFO("***************** ROS Callback function *********************");
	//ROS_INFO("**** Subscribed to the centers of the segmented areas *******");
    if(msg->polygon.points.size() != 0)
    {
        this->current_topo_centers = *msg;
    }
}

void TopoSegReaderComponent::listen_current_conn_graph(const PolygonStamped::ConstPtr& msg)
{
    //ROS_INFO("**************** ROS Callback function **********************");
	//ROS_INFO("** Subscribed to the connetion between the segmented areas **");
    if(msg->polygon.points.size() != 0)
    {
        this->current_conn_graph = *msg;
    }
}

void TopoSegReaderComponent::listen_current_node(const voronoiseg::PoseAndTopologicalID& msg)
{
	this->current_node = msg;
	//ROS_INFO("Callback function...");
	//ROS_INFO("Subscribed to the current region in which the robot is");
	NodePtr node = new Node();
	node->label = "n" + to_string((int)this->current_node.id);
	node->x = this->current_node.pose.position.x;
	node->y = this->current_node.pose.position.y;
	node->flag = 0;
	CurrentPosPtr cp = new CurrentPos();
	cp->node = node;
	println("Current position of the robot: %s",to_string(cp->node).c_str());
	addToWorkingMemory(newDataID(), cp);
}

void TopoSegReaderComponent::listen_base_position(const voronoiseg::PoseAndTopologicalID& msg)
{
	this->base_station = msg;
	//ROS_INFO("Callback function...");
	//ROS_INFO("Subscribed to the region in which the robot starts the task");
	NodePtr node = new Node();
	node->label = "n" + to_string((int)base_station.id);
	node->x = this->base_station.pose.position.x;
	node->y = this->base_station.pose.position.y;
	node->flag = 0;
	BasePosPtr base = new BasePos();
	base->node = node;
	println("Base position of the robot: %s",to_string(base->node).c_str());
	addToWorkingMemory(newDataID(), base);
	/*
	    The base position has to be stored in the beginning when the robot starts
	*/
	this->temp_base.shutdown();
}

void TopoSegReaderComponent::readAction(const WorkingMemoryChange& _wmc)
{
    println("***** CAST Filter Callback: a new action has been received *****");
    this->current = getMemoryEntry<TopoGraphWriterAction>(_wmc.address);
    
    if(this->current->op == START)
    {
        ros::spinOnce();
        
        while(!waitForSubscriber())
        {
            println("Data is not still available");
        }
        
        GraphPtr graph = new Graph();
        
        for(unsigned int i = 0; i < this->current_topo_centers.polygon.points.size(); i++)
		{
			NodePtr node = new Node();
			node->label = "n" + to_string((int)this->current_topo_centers.polygon.points[i].z);
			//println("%s",node->label.c_str());
			node->x = this->current_topo_centers.polygon.points[i].x;
			node->y = this->current_topo_centers.polygon.points[i].y;
			node->flag = 0;
			graph->nodes.push_back(node);
		    //println("Current node: %s",to_string(node).c_str());
		}
		for(unsigned int i = 0; i < this->current_conn_graph.polygon.points.size(); i++)
		{
		    EdgePtr edge = new Edge();
		    edge->a = getNode((int)this->current_conn_graph.polygon.points[i].x,graph);
		    //println("Node a: %s",edge->a->label.c_str());
		    edge->b = getNode((int)this->current_conn_graph.polygon.points[i].y,graph);
		    //println("Node b: %s",edge->b->label.c_str());
		    graph->edges.push_back(edge);
		    //println("Current edge: %s",to_string(edge).c_str());
		}
		
		this->sub_topo_centers.shutdown();
        this->sub_conn_graph.shutdown();
		
		this->current->status = EXECUTED;
		overwriteWorkingMemory(_wmc.address,this->current);
        println("Action Executed");

        addToWorkingMemory(newDataID(),graph);
        println("A new topological graph has been added in the WM");
    }
    else if(this->current->op == END)
    {
        this->sub_topo_centers.shutdown();
        this->sub_conn_graph.shutdown();
        this->current->status = EXECUTED;
        overwriteWorkingMemory(_wmc.address,this->current);
        println("Action Executed");
    }
    else
    {
        println("Action unknown");
    }
}

void TopoSegReaderComponent::start()
{
    println("************************************************************");
    println("******** TopoSegReaderComponent ROS CAST Component *********");
    println("******************* Status: starting ***********************");
    println("************************************************************");
    
    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	
	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "TopoSegReaderComponent");
	
	ros::NodeHandle node;
    this->sub_topo_centers = node.subscribe("/toposeg/topo_centers",10,&TopoSegReaderComponent::listen_current_topo_centers,this);
	this->sub_conn_graph = node.subscribe("/toposeg/conn_graph",10,&TopoSegReaderComponent::listen_current_conn_graph,this);
	
	this->current = new TopoGraphWriterAction();
	this->current->op = WAIT;
	
	addChangeFilter(cast::createLocalTypeFilter<TopoGraphWriterAction>(ADD),new cast::MemberFunctionChangeReceiver<TopoSegReaderComponent>(this,&TopoSegReaderComponent::readAction));
}

void TopoSegReaderComponent::runComponent()
{
    println("************************************************************");
    println("******** TopoSegReaderComponent ROS CAST Component *********");
    println("******************* Status: running ************************");
    println("************************************************************");

    ros::NodeHandle node;
    ros::Subscriber sub;
    sub = node.subscribe("/toposeg/currentpose_and_id",10,&TopoSegReaderComponent::listen_current_node,this);
    this->temp_base = node.subscribe("/toposeg/currentpose_and_id",10,&TopoSegReaderComponent::listen_base_position,this);
    ros::Rate r(10);

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
		return new TopoSegReaderComponent();
  	}
}
