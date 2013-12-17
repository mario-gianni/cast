#include <string>
#include <GapDetectionComponent.hpp>
#include <gap_traversal_msgs/gap_service.h>
#include <gap_traversal_msgs/gap_message.h>

string GapDetectionComponent::to_string(double value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

void GapDetectionComponent::start()
{

	println("************************************************************");
	println("********* GapDetectionComponent ROS CAST Component *********");
	println("******************* Status: starting ***********************");
	println("************************************************************");

	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);

	println("******************* ROS init() *****************************");
	ros::init(argc, argv, "GapDetectionComponent");

	NodeHandle node;
	client = node.serviceClient<gap_traversal_msgs::gap_service>("gap_detection");

	println("***************** Initializing filters *********************");
	addChangeFilter(createLocalTypeFilter<GapDetectionAction>(ADD),new MemberFunctionChangeReceiver<GapDetectionComponent>(this,&GapDetectionComponent::readAction));

}

void GapDetectionComponent::runComponent()
{
	println("************************************************************");
	println("********* GapDetectionComponent ROS CAST Component *********");
	println("******************* Status: running ************************");
	println("************************************************************");

}

void GapDetectionComponent::readAction(const WorkingMemoryChange& _wmc)
{
	GapDetectionActionPtr current = getMemoryEntry<GapDetectionAction>(_wmc.address);
	std::string id_action = _wmc.address.id;

	if(current->op == START)
	{
		gap_traversal_msgs::gap_service srv;
		srv.request.control = 0;

		if(client.exists())
		{
			println("gap_detection service exists");
		}
		else
		{
		    println("gap_detection service doesn't exist");
		}
		if(client.call(srv))
		{
			if(srv.response.result == 0)
			{
				println("The GapDetectionAction has been executed and a gap has been detected");
				gap_traversal_msgs::gap_message detected_gap;
				detected_gap = srv.response.gap;

				GapPtr _gap = new Gap();
				_gap->id = to_string(detected_gap.id);
				_gap->traversability = detected_gap.traversability;

				if(detected_gap.traversability == true)
				{
					println("The gap is traversable");

					_gap->init.pos.x = detected_gap.Init.position.x;
					_gap->init.pos.y = detected_gap.Init.position.y;
					_gap->init.pos.z = detected_gap.Init.position.z;
					_gap->init.orient.x = detected_gap.Init.orientation.x;
					_gap->init.orient.y = detected_gap.Init.orientation.y;
					_gap->init.orient.z = detected_gap.Init.orientation.z;
					_gap->init.orient.w = detected_gap.Init.orientation.w;

					_gap->final.pos.x = detected_gap.Final.position.x;
					_gap->final.pos.y = detected_gap.Final.position.y;
					_gap->final.pos.z = detected_gap.Final.position.z;
					_gap->final.orient.x = detected_gap.Final.orientation.x;
					_gap->final.orient.y = detected_gap.Final.orientation.y;
					_gap->final.orient.z = detected_gap.Final.orientation.z;
					_gap->final.orient.w = detected_gap.Final.orientation.w;

					_gap->conf.flipperAngleFL = detected_gap.configuration[0];
					_gap->conf.flipperAngleFR = detected_gap.configuration[1];
					_gap->conf.flipperAngleRR = detected_gap.configuration[2];
					_gap->conf.flipperAngleRL = detected_gap.configuration[3];
					_gap->conf.differential = detected_gap.configuration[4];
					println("The parameters of the structure of the gap have been parsed");

				}
				else
				{
					 println("The gap is not traversable");
				}

				addToWorkingMemory(newDataID(),_gap);
				current->status = EXECUTED;
				
				overwriteWorkingMemory(id_action,current);

				
			}
			else if(srv.response.result == 1)
			{
				println("The GapDetectionAction has failed because there are no point clouds");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
			}
			else if(srv.response.result == 2)
			{
				println("The GapDetectionAction has been executed but the gap has not been detected");
				current->status = EXECUTED;
				overwriteWorkingMemory(id_action,current);
			}
			else
			{
				println("Invalid return status");
			}
		}
		else
		{
			current->status = FAILED;
			overwriteWorkingMemory(id_action,current);
		}
	}
	else
	{
		println("Action unknown");
	}
}

extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
    	return new GapDetectionComponent();
  	}
}


