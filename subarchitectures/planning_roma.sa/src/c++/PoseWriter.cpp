#include <PoseWriter.hpp>
#include "tf/transform_listener.h"

/* this function converts a double ia a string */
std::string PoseWriter::to_string(double value)
{
	std::stringstream ss;
	ss << value;
	return ss.str();
}

std::string PoseWriter::to_string(eu::nifti::env::position::PosePtr pose)
{
    std::string result;
    result = "holds(navigation,at(";
    result = result + to_string(pose->x) + ",";
    result = result + to_string(pose->y) + ",";
    result = result + to_string(pose->z) + ",";
    result = result + to_string(pose->theta) + ",[])";
    return result;
}

void PoseWriter::start()
{
    println("PoseVWriter ROS CAST Component");
    println("Status: starting...");
	char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	ros::init(argc, argv, "PoseWriter");

}

void PoseWriter::runComponent()
{
    println("PoseVWriter ROS CAST Component");
    println("Status: running...");
    
    ros::NodeHandle node;
	tf::TransformListener tf;
	eu::nifti::env::position::PosePtr robot_pose = new eu::nifti::env::position::Pose();
	
	std::string source_frameid = std::string("/map");
	std::string target_frameid = std::string("/base_link");
	
	while(node.ok())
	{
	    if(tf.canTransform(source_frameid, target_frameid, ros::Time()))
	    {
	        tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));
	        
	        try
	        {
	            tf::StampedTransform echo_transform;
	            tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
                double yaw, pitch, roll;
                echo_transform.getBasis().getRPY(roll, pitch, yaw);
                tf::Quaternion q = echo_transform.getRotation();
                tf::Vector3 v = echo_transform.getOrigin();
                robot_pose->x = v.getX();
                robot_pose->y = v.getY();
                robot_pose->z = v.getZ();
                robot_pose->theta = yaw;
                println("Current position and orientation of the robot is : %s",to_string(robot_pose).c_str());
                addToWorkingMemory(newDataID(),robot_pose);
	        }
	        catch(tf::TransformException& ex)
	        {
	            std::cout << "Failure at "<< ros::Time::now() << std::endl;
				std::cout << "Exception thrown:" << ex.what()<< std::endl;
				std::cout << "The current list of frames is:" <<std::endl;
				std::cout << tf.allFramesAsString() << std::endl;

	        }
	    }
	    else
	    {
	        println("Transform is not available: current robot pose not available");
	    }
	    
	    sleep(5);
	}
}

extern "C" 
{
	cast::CASTComponentPtr newComponent() 
	{
    	return new PoseWriter();
  	}
}
