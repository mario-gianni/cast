#include <string>
#include <stdlib.h> 
#include <time.h>
#include <NavigationComponent.hpp>

void NavigationComponent::myforwardResponseReceived(const WorkingMemoryChange& _wmc)
{
	t_Thread = boost::thread(&NavigationComponent::forwardResponseReceived,this,_wmc);
}

void NavigationComponent::mymoveBaseActionReceived2(const WorkingMemoryChange& _wmc)
{
	m_Thread = boost::thread(&NavigationComponent::moveBaseActionReceived2,this,_wmc);
}


/* this function converts a double ia a string */
string NavigationComponent::to_string(double value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

string NavigationComponent::to_string(eu::nifti::env::position::PosePtr pose)
{
    string result;
    result = "holds(navigation,at(";
    result = result + to_string(pose->x) + ",";
    result = result + to_string(pose->y) + ",";
    result = result + to_string(pose->z) + ",";
    result = result + to_string(pose->theta) + ",[])";
    return result;
}

bool NavigationComponent::buildUserDefinedTrajectory(double vel, geometry_msgs::PoseStamped in, geometry_msgs::Pose& poseB, geometry_msgs::Twist& velB)
{
	poseB = in.pose;

	double yaw = tf::getYaw(in.pose.orientation);

	in.header.stamp = ros::Time::now();

	velB.linear.x = vel * cos(yaw);
	velB.linear.y = vel * sin(yaw);

	return true;
}

void NavigationComponent::getTracksVelCmd(double linear_vel, double angular_vel, double robot_width, nifti_robot_driver_msgs::Tracks& tracks_cmd)
{
	double d = robot_width/2;

	tracks_cmd.left = linear_vel - (d * angular_vel);
	tracks_cmd.right = linear_vel + (d * angular_vel);

	if(tracks_cmd.left < -0.6)
	{
		tracks_cmd.left = -0.6;
	}
	if(tracks_cmd.left > 0.6)
	{
		tracks_cmd.left = 0.6;
	}
	if(tracks_cmd.right < -0.6)
	{
		tracks_cmd.right = -0.6;
	}
	if(tracks_cmd.right > 0.6)
	{
		tracks_cmd.right = 0.6;
	}
	
	std::cout<<"Track Vel: ["<< tracks_cmd.right <<","<< tracks_cmd.left << "]"<< std::endl;
}

bool NavigationComponent::getRobotCommands(double displacement, tf::StampedTransform robot_pose, double k1, double k2, geometry_msgs::Pose poseB, geometry_msgs::Twist velB, double& linear_vel, double& angular_vel)
{
	double roll, pitch, yaw;
	robot_pose.getBasis().getRPY(roll,pitch,yaw);

	double a = cos(yaw);
	double b = sin(yaw);
	double c = -sin(yaw)/displacement;
	double d = cos(yaw)/displacement;

	double u1 = velB.linear.x + k1 * (poseB.position.x - robot_pose.getOrigin().getX());
	double u2 = velB.linear.y + k2 * (poseB.position.y - robot_pose.getOrigin().getY());
	
	std::cout<< "Cartesian error: [" << poseB.position.x - robot_pose.getOrigin().getX() << "," << poseB.position.y - robot_pose.getOrigin().getY() << "]" <<std::endl;

	//double u1 = k1 * (poseB.position.x - robot_pose.getOrigin().getX());
	//double u2 = k2 * (poseB.position.y - robot_pose.getOrigin().getY());

	linear_vel = a * u1 + b * u2;
	angular_vel = c * u1 + d * u2;

	return true;

}

void NavigationComponent::realRobotPoseB(double displacement, tf::StampedTransform real_robot_pose, tf::StampedTransform& real_robot_poseB)
{
	real_robot_poseB.frame_id_ = real_robot_pose.frame_id_;
	real_robot_poseB.stamp_ = real_robot_pose.stamp_;
	real_robot_poseB.child_frame_id_ = real_robot_pose.child_frame_id_;

	tf::Vector3 v;
	double roll, pitch, yaw;
	real_robot_pose.getBasis().getRPY(roll,pitch,yaw);

	v.setX(real_robot_pose.getOrigin().getX() + displacement*cos(yaw));
	v.setY(real_robot_pose.getOrigin().getY() + displacement*sin(yaw));
	v.setZ(real_robot_pose.getOrigin().getZ());
	real_robot_poseB.setOrigin(v);

	real_robot_poseB.setRotation(real_robot_pose.getRotation());
}

void NavigationComponent::forwardResponseReceived(const WorkingMemoryChange& _wmc)
{
	response = getMemoryEntry<eu::nifti::context::forwardResponse>(_wmc.address);
	println("findForwardUsingPointCloudsIterative Response received");
	response_flag = true;
}

bool NavigationComponent::waitForForwardRequest()
{
        return response_flag;

}

void NavigationComponent::moveBaseActionReceived2(const WorkingMemoryChange& _wmc)
{
	MoveBaseActionPtr current = getMemoryEntry<MoveBaseAction>(_wmc.address);
	string id_action = _wmc.address.id;
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	
	if(current->command == MOVEFORWARD)
	{
	    eu::nifti::context::forwardRequestPtr a = new eu::nifti::context::forwardRequest();
	    srand(time(NULL));
	    a->requestID = rand() % 100 + 1;
	    a->requestFlag = true;
	    addToWorkingMemory(newDataID(),a);
	    
	    ros::Time start = ros::Time::now();
	    println("findForwardUsingPointCloudsIterative waiting");
	    while(!waitForForwardRequest())
	    {
	        if(ros::Time::now().toSec() - start.toSec() > 10.00)
	        {
	            println("wait for findForwardUsingPointCloudsIterative");
	            start = ros::Time::now();
	        }
	    }
        
	    println("findForwardUsingPointCloudsIterative cheking");
	    
	    if(response->responseFlag == true)
	    {
	        println("findForwardUsingPointCloudsIterative responseflag true");
	        geometry_msgs::Pose poseB;
	        geometry_msgs::Twist velB;
	        ros::Rate rate(1);
	        //for(unsigned int i = 0; i < response->intermediatePoses.size(); i++)
	        for(unsigned int i = 0; i < 2; i++)
	        {
	            std::cout<< "Current Point Position " << response->intermediatePoses[i]->x << "," << response->intermediatePoses[i]->y << "," << response->intermediatePoses[i]->z << std::endl;
	            std::cout<< "Current Point Orientation " << response->intermediatePoses[i]->qx << "," << response->intermediatePoses[i]->qy << "," << response->intermediatePoses[i]->qz << "," << response->intermediatePoses[i]->qw << std::endl;
	            geometry_msgs::PoseStamped in;
	            in.pose.position.x = response->intermediatePoses[i]->x;
	            in.pose.position.y = response->intermediatePoses[i]->y;
	            in.pose.position.z = response->intermediatePoses[i]->z;
	            in.pose.orientation.x = response->intermediatePoses[i]->qx;
	            in.pose.orientation.y = response->intermediatePoses[i]->qy;
	            in.pose.orientation.z = response->intermediatePoses[i]->qz;
	            in.pose.orientation.w = response->intermediatePoses[i]->qw;
	            buildUserDefinedTrajectory(vel_g,in,poseB,velB);
	            realRobotPoseB(displacement,echo_transform, real_robot_poseB);
	            getRobotCommands(displacement,real_robot_poseB,k1,k2,poseB,velB,linear_vel,angular_vel);
	            getTracksVelCmd(linear_vel,angular_vel,robot_width,tracks_cmd);
	            tracks_vel_cmd_pub.publish(tracks_cmd);
	            rate.sleep();
	        }
	        
	        getTracksVelCmd(0,0,robot_width,tracks_cmd);
	        tracks_vel_cmd_pub.publish(tracks_cmd);
	        println("NavigationComponent job done");
	        current->status = EXECUTED;
			overwriteWorkingMemory(id_action,current);
	    }
	    else
	    {
	        println("findForwardUsingPointCloudsIterative responseflag false");
	        
	        goal.target_pose.pose.position.x = 0.5;
		    goal.target_pose.pose.orientation.w = 1.0;
		    println("move forward 0.5m");
		
		    if(client->waitForServer(ros::Duration(5.0)))
		    {
			    client->sendGoal(goal);
			    client->waitForResult();

			    if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			    {
				    println("SUCCEEDED - The goal was achieved successfully by the action server");
				    println("MoveBaseAction Status: EXECUTED");
				    current->status = EXECUTED;
				    overwriteWorkingMemory(id_action,current);

			    }
			    else
			    {
				    if(client2d->waitForServer(ros::Duration(5.0)))
				    {
					    client2d->sendGoal(goal);
					    client2d->waitForResult();
				
					    if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					    {
						    println("SUCCEEDED - The goal was achieved successfully by the action server");
						    println("MoveBaseAction Status: EXECUTED");
						    current->status = EXECUTED;
						    overwriteWorkingMemory(id_action,current);
					    }
					    else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
					    {
						    println("ABORTED - The goal was aborted by the action server");
						    println("MoveBase Action Status: FAILED");
						    current->status = FAILED;
						    overwriteWorkingMemory(id_action,current);

					    }else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
					    {
						    println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
						    println("MoveBase Action Status: FAILED");
						    current->status = FAILED;
						    overwriteWorkingMemory(id_action,current);

					    }else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
					    {
						    println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
						    println("MoveBase Action Status: FAILED");
						    current->status = FAILED;
						    overwriteWorkingMemory(id_action,current);

					    }else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
					    {
						    println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
						    println("MoveBase Action Status: FAILED");
						    current->status = FAILED;
						    overwriteWorkingMemory(id_action,current);

					    }else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
					    {
					    	println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
					    	println("MoveBase Action Status: FAILED");
					    	current->status = FAILED;
					    	overwriteWorkingMemory(id_action,current);
					    }
					    else
					    {
					    	println("PENDING or ACTIVE");
					    }
				    }
				    else
				    {
				    	println("MoveBase Action Status: FAILED");
				    	current->status = FAILED;
				    	overwriteWorkingMemory(id_action,current);
				    }
			
	            }
	    
	        }
	        else
		    {
			    if(client2d->waitForServer(ros::Duration(5.0)))
			    {
				    client2d->sendGoal(goal);
				    client2d->waitForResult();
				
				    if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				    {
					    println("SUCCEEDED - The goal was achieved successfully by the action server");
					    println("MoveBaseAction Status: EXECUTED");
					    current->status = EXECUTED;
					    overwriteWorkingMemory(id_action,current);
				    }
				    else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
				    {
				    	println("ABORTED - The goal was aborted by the action server");
				    	println("MoveBase Action Status: FAILED");
				    	current->status = FAILED;
				    	overwriteWorkingMemory(id_action,current);

				    }else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
				    {
				    	println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
				    	println("MoveBase Action Status: FAILED");
				    	current->status = FAILED;
				    	overwriteWorkingMemory(id_action,current);

				    }else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
			    	{
			    		println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
				    	println("MoveBase Action Status: FAILED");
			    		current->status = FAILED;
			    		overwriteWorkingMemory(id_action,current);

			    	}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
			    	{
				    	println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
				    	println("MoveBase Action Status: FAILED");
			    		current->status = FAILED;
			    		overwriteWorkingMemory(id_action,current);

			    	}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
			    	{
			    		println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
			    		println("MoveBase Action Status: FAILED");
			    		current->status = FAILED;
			    		overwriteWorkingMemory(id_action,current);
			    	}
			    	else
			    	{
			    		println("PENDING or ACTIVE");
			    	}
		    	}
		    	else
		    	{
			    	println("MoveBase Action Status: FAILED");
			    	current->status = FAILED;
			    	overwriteWorkingMemory(id_action,current);
			    }
		    }
	    }
	}
	else if(current->command == MOVELEFT)
	{
		//move to position 0.5m to the left and turn by 45 degrees
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.position.y = 0.5;
		goal.target_pose.pose.orientation.z = sin(PI/8);
		goal.target_pose.pose.orientation.w = cos(PI/8);
		println("move to position 0.5m to the left and turn by 45 degrees");

		if(client->waitForServer(ros::Duration(5.0)))
		{
			client->sendGoal(goal);
			client->waitForResult();

			if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				println("SUCCEEDED - The goal was achieved successfully by the action server");
				println("MoveBaseAction Status: EXECUTED");
				current->status = EXECUTED;
				overwriteWorkingMemory(id_action,current);

			}
			else
			{
				if(client2d->waitForServer(ros::Duration(5.0)))
				{
					client2d->sendGoal(goal);
					client2d->waitForResult();
				
					if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						println("SUCCEEDED - The goal was achieved successfully by the action server");
						println("MoveBaseAction Status: EXECUTED");
						current->status = EXECUTED;
						overwriteWorkingMemory(id_action,current);
					}
					else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						println("ABORTED - The goal was aborted by the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
					{
						println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
					{
						println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
					{
						println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
					{
						println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);
					}
					else
					{
						println("PENDING or ACTIVE");
					}
				}
				else
				{
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
			}
		}
		else
		{
			if(client2d->waitForServer(ros::Duration(5.0)))
			{
				client2d->sendGoal(goal);
				client2d->waitForResult();
				
				if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					println("SUCCEEDED - The goal was achieved successfully by the action server");
					println("MoveBaseAction Status: EXECUTED");
					current->status = EXECUTED;
					overwriteWorkingMemory(id_action,current);
				}
				else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					println("ABORTED - The goal was aborted by the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
				{
					println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
				{
					println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
				{
					println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
				{
					println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
				else
				{
					println("PENDING or ACTIVE");
				}
			}
			else
			{
				println("MoveBase Action Status: FAILED");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
				
			}
		}
	}
	else if(current->command == MOVERIGHT)
	{
		//move to position 0.5m to the right and turn by 45 degree
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.position.y = -0.5;
		goal.target_pose.pose.orientation.z = -sin(PI/8);
		goal.target_pose.pose.orientation.w = cos(PI/8);
		println("move to position 0.5m to the right and turn by 45 degree");

		if(client->waitForServer(ros::Duration(5.0)))
		{
			client->sendGoal(goal);
			client->waitForResult();

			if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				println("SUCCEEDED - The goal was achieved successfully by the action server");
				println("MoveBaseAction Status: EXECUTED");
				current->status = EXECUTED;
				overwriteWorkingMemory(id_action,current);

			}
			else
			{
				if(client2d->waitForServer(ros::Duration(5.0)))
				{
					client2d->sendGoal(goal);
					client2d->waitForResult();
				
					if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						println("SUCCEEDED - The goal was achieved successfully by the action server");
						println("MoveBaseAction Status: EXECUTED");
						current->status = EXECUTED;
						overwriteWorkingMemory(id_action,current);
					}
					else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						println("ABORTED - The goal was aborted by the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
					{
						println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
					{
						println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
					{
						println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
					{
						println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);
					}
					else
					{
						println("PENDING or ACTIVE");
					}
				}
				else
				{
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
			}
		}
		else
		{
			if(client2d->waitForServer(ros::Duration(5.0)))
			{
				client2d->sendGoal(goal);
				client2d->waitForResult();
				
				println("cat cat cat");
				
				if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					println("SUCCEEDED - The goal was achieved successfully by the action server");
					println("MoveBaseAction Status: EXECUTED");
					current->status = EXECUTED;
					overwriteWorkingMemory(id_action,current);
				}
				else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					println("ABORTED - The goal was aborted by the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
				{
					println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
				{
					println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
				{
					println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
				{
					println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
				else
				{
					println("PENDING or ACTIVE");
				}
			}
			else
			{
				println("MoveBase Action Status: FAILED");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
				
			}
		}
	}
	else if(current->command == MOVEBACK)
	{
		//move backward by 0.5m

		double max_walk_vel = 0.2;
		double speed = -1;

		geometry_msgs::Twist cmdvel;
		cmdvel.linear.x = speed * max_walk_vel;
		cmdvel.angular.z = 0.0;

		println("move backward by 0.5m");
		this->exe_pub.publish(cmdvel);

		current->status = EXECUTED;
		println("MoveBaseAction Status: EXECUTED");
		overwriteWorkingMemory(id_action,current);

		//goal.target_pose.pose.position.x = -0.5;
		//goal.target_pose.pose.orientation.w = 1.0;
		//println("move backward by 0.5m");
	}
	else if(current->command == TURNLEFT)
	{
		//turn left by 30 degrees

		double max_yaw_rate = 0.4;
		double turn = 1;

		geometry_msgs::Twist cmdvel;

		cmdvel.linear.x = 0.0;
		cmdvel.angular.z = turn * max_yaw_rate;

		println("turn left by 30 degrees");
		this->exe_pub.publish(cmdvel);
		current->status = EXECUTED;
		println("MoveBaseAction Status: EXECUTED");
		overwriteWorkingMemory(id_action,current);

		//goal.target_pose.pose.orientation.z = sin(PI/12);
		//goal.target_pose.pose.orientation.w = cos(PI/12);
		//println("turn left by 30 degrees");
	}
	else if(current->command == TURNRIGHT)
	{
		//turn right by 30 degrees

		double max_yaw_rate = 0.4;
		double turn = -1;

		geometry_msgs::Twist cmdvel;

		cmdvel.linear.x = 0.0;
		cmdvel.angular.z = turn * max_yaw_rate;
		println("turn right by 30 degrees");
		this->exe_pub.publish(cmdvel);
		current->status = EXECUTED;
		println("MoveBaseAction Status: EXECUTED");
		overwriteWorkingMemory(id_action,current);

		//goal.target_pose.pose.orientation.z = -sin(PI/12);
		//goal.target_pose.pose.orientation.w = cos(PI/12);
		//println("turn right by 30 degrees");
	}
	else
	{
		println("Physical Execution: Action Unknown");
	}
}

void NavigationComponent::gotoNodeActionReceived(const WorkingMemoryChange& _wmc)
{
    GoToNodeActionPtr current = getMemoryEntry<GoToNodeAction>(_wmc.address);
    string id_action = _wmc.address.id;
    
     tf::Quaternion quat;
     quat.setRPY(0.0, 0.0, current->theta);
     std::string fixed_frame = "/map";
     tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat,tf::Point(current->node->x, current->node->y, 0.0)), ros::Time::now(), fixed_frame);
    
	move_base_msgs::MoveBaseGoal goal;
	tf::poseStampedTFToMsg(p, goal.target_pose);

	if(client->waitForServer(ros::Duration(5.0)))
	{
		client->sendGoal(goal);
		client->waitForResult();

		if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			println("GoToNodeAction Status: EXECUTED");
			current->status = EXECUTED;
			overwriteWorkingMemory(id_action,current);
		}
		else
		{
			client->cancelAllGoals();
			println("3d path planning failed");
			if(client2d->waitForServer(ros::Duration(5.0)))
			{
				client2d->sendGoal(goal);
				client2d->waitForResult();
				
				if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
	    			println("GoToNodeAction Status: EXECUTED");
	    			current->status = EXECUTED;
	    			overwriteWorkingMemory(id_action,current);
				}
				else
				{
					println("GoToNodeAction Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
			}
			else
			{
				println("2d path planning is not working");
				println("GoToNodeAction Status: FAILED");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
			}
		}
	}
	else
	{
		println("3d path planning is not running");
		if(client2d->waitForServer(ros::Duration(5.0)))
		{
			client2d->sendGoal(goal);
			client2d->waitForResult();
				
			if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
	   			println("GoToNodeAction Status: EXECUTED");
	   			current->status = EXECUTED;
	   			overwriteWorkingMemory(id_action,current);
			}
			else
			{
				println("GoToNodeAction Status: FAILED");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
			}
		}
		else
		{
			println("2d path planning is not working");
			println("GoToNodeAction Status: FAILED");
			current->status = FAILED;
			overwriteWorkingMemory(id_action,current);
		}
	}
}

void NavigationComponent::moveBaseActionReceived(const WorkingMemoryChange& _wmc)
{
	MoveBaseActionPtr current = getMemoryEntry<MoveBaseAction>(_wmc.address);
	string id_action = _wmc.address.id;
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	
	if(current->command == MOVEFORWARD)
	{
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.orientation.w = 1.0;
		println("move forward 0.5m");
		
		if(client->waitForServer(ros::Duration(5.0)))
		{
			client->sendGoal(goal);
			client->waitForResult();

			if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				println("SUCCEEDED - The goal was achieved successfully by the action server");
				println("MoveBaseAction Status: EXECUTED");
				current->status = EXECUTED;
				overwriteWorkingMemory(id_action,current);

			}
			else
			{
				if(client2d->waitForServer(ros::Duration(5.0)))
				{
					client2d->sendGoal(goal);
					client2d->waitForResult();
				
					if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						println("SUCCEEDED - The goal was achieved successfully by the action server");
						println("MoveBaseAction Status: EXECUTED");
						current->status = EXECUTED;
						overwriteWorkingMemory(id_action,current);
					}
					else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						println("ABORTED - The goal was aborted by the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
					{
						println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
					{
						println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
					{
						println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
					{
						println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);
					}
					else
					{
						println("PENDING or ACTIVE");
					}
				}
				else
				{
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
			}
		}
		else
		{
			if(client2d->waitForServer(ros::Duration(5.0)))
			{
				client2d->sendGoal(goal);
				client2d->waitForResult();
				
				if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					println("SUCCEEDED - The goal was achieved successfully by the action server");
					println("MoveBaseAction Status: EXECUTED");
					current->status = EXECUTED;
					overwriteWorkingMemory(id_action,current);
				}
				else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					println("ABORTED - The goal was aborted by the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
				{
					println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
				{
					println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
				{
					println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
				{
					println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
				else
				{
					println("PENDING or ACTIVE");
				}
			}
			else
			{
				println("MoveBase Action Status: FAILED");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
			}
		}

	}
	else if(current->command == MOVELEFT)
	{
		//move to position 0.5m to the left and turn by 45 degrees
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.position.y = 0.5;
		goal.target_pose.pose.orientation.z = sin(PI/8);
		goal.target_pose.pose.orientation.w = cos(PI/8);
		println("move to position 0.5m to the left and turn by 45 degrees");

		if(client->waitForServer(ros::Duration(5.0)))
		{
			client->sendGoal(goal);
			client->waitForResult();

			if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				println("SUCCEEDED - The goal was achieved successfully by the action server");
				println("MoveBaseAction Status: EXECUTED");
				current->status = EXECUTED;
				overwriteWorkingMemory(id_action,current);

			}
			else
			{
				if(client2d->waitForServer(ros::Duration(5.0)))
				{
					client2d->sendGoal(goal);
					client2d->waitForResult();
				
					if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						println("SUCCEEDED - The goal was achieved successfully by the action server");
						println("MoveBaseAction Status: EXECUTED");
						current->status = EXECUTED;
						overwriteWorkingMemory(id_action,current);
					}
					else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						println("ABORTED - The goal was aborted by the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
					{
						println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
					{
						println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
					{
						println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
					{
						println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);
					}
					else
					{
						println("PENDING or ACTIVE");
					}
				}
				else
				{
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
			}
		}
		else
		{
			if(client2d->waitForServer(ros::Duration(5.0)))
			{
				client2d->sendGoal(goal);
				client2d->waitForResult();
				
				if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					println("SUCCEEDED - The goal was achieved successfully by the action server");
					println("MoveBaseAction Status: EXECUTED");
					current->status = EXECUTED;
					overwriteWorkingMemory(id_action,current);
				}
				else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					println("ABORTED - The goal was aborted by the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
				{
					println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
				{
					println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
				{
					println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
				{
					println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
				else
				{
					println("PENDING or ACTIVE");
				}
			}
			else
			{
				println("MoveBase Action Status: FAILED");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
				
			}
		}
	}
	else if(current->command == MOVERIGHT)
	{
		//move to position 0.5m to the right and turn by 45 degree
		goal.target_pose.pose.position.x = 0.5;
		goal.target_pose.pose.position.y = -0.5;
		goal.target_pose.pose.orientation.z = -sin(PI/8);
		goal.target_pose.pose.orientation.w = cos(PI/8);
		println("move to position 0.5m to the right and turn by 45 degree");

		if(client->waitForServer(ros::Duration(5.0)))
		{
			client->sendGoal(goal);
			client->waitForResult();

			if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				println("SUCCEEDED - The goal was achieved successfully by the action server");
				println("MoveBaseAction Status: EXECUTED");
				current->status = EXECUTED;
				overwriteWorkingMemory(id_action,current);

			}
			else
			{
				if(client2d->waitForServer(ros::Duration(5.0)))
				{
					client2d->sendGoal(goal);
					client2d->waitForResult();
				
					if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						println("SUCCEEDED - The goal was achieved successfully by the action server");
						println("MoveBaseAction Status: EXECUTED");
						current->status = EXECUTED;
						overwriteWorkingMemory(id_action,current);
					}
					else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						println("ABORTED - The goal was aborted by the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
					{
						println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
					{
						println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
					{
						println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);

					}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
					{
						println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
						println("MoveBase Action Status: FAILED");
						current->status = FAILED;
						overwriteWorkingMemory(id_action,current);
					}
					else
					{
						println("PENDING or ACTIVE");
					}
				}
				else
				{
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
			}
		}
		else
		{
			if(client2d->waitForServer(ros::Duration(5.0)))
			{
				client2d->sendGoal(goal);
				client2d->waitForResult();
				
				if(client2d->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					println("SUCCEEDED - The goal was achieved successfully by the action server");
					println("MoveBaseAction Status: EXECUTED");
					current->status = EXECUTED;
					overwriteWorkingMemory(id_action,current);
				}
				else if (client2d->getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					println("ABORTED - The goal was aborted by the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
				{
					println("PREEMPTED - The goal was preempted by either another goal, or a preempt message being sent to the action server");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if (client2d->getState() == actionlib::SimpleClientGoalState::REJECTED)
				{
					println("REJECTED - The goal was rejected by the action server without being processed and without a request from the action client to cancel");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::LOST)
				{
					println("LOST - The goal was sent by the ActionClient, but disappeared due to some communication error ");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);

				}else if(client2d->getState() == actionlib::SimpleClientGoalState::RECALLED)
				{
					println("RECALLED - The goal was canceled by either another goal, or a cancel request before the action server began processing the goal");
					println("MoveBase Action Status: FAILED");
					current->status = FAILED;
					overwriteWorkingMemory(id_action,current);
				}
				else
				{
					println("PENDING or ACTIVE");
				}
			}
			else
			{
				println("MoveBase Action Status: FAILED");
				current->status = FAILED;
				overwriteWorkingMemory(id_action,current);
				
			}
		}
	}
	else if(current->command == MOVEBACK)
	{
		//move backward by 0.5m

		double max_walk_vel = 0.2;
		double speed = -1;

		geometry_msgs::Twist cmdvel;
		cmdvel.linear.x = speed * max_walk_vel;
		cmdvel.angular.z = 0.0;

		println("move backward by 0.5m");
		this->exe_pub.publish(cmdvel);

		current->status = EXECUTED;
		println("MoveBaseAction Status: EXECUTED");
		overwriteWorkingMemory(id_action,current);

		//goal.target_pose.pose.position.x = -0.5;
		//goal.target_pose.pose.orientation.w = 1.0;
		//println("move backward by 0.5m");
	}
	else if(current->command == TURNLEFT)
	{
		//turn left by 30 degrees

		double max_yaw_rate = 0.4;
		double turn = 1;

		geometry_msgs::Twist cmdvel;

		cmdvel.linear.x = 0.0;
		cmdvel.angular.z = turn * max_yaw_rate;

		println("turn left by 30 degrees");
		this->exe_pub.publish(cmdvel);
		current->status = EXECUTED;
		println("MoveBaseAction Status: EXECUTED");
		overwriteWorkingMemory(id_action,current);

		//goal.target_pose.pose.orientation.z = sin(PI/12);
		//goal.target_pose.pose.orientation.w = cos(PI/12);
		//println("turn left by 30 degrees");
	}
	else if(current->command == TURNRIGHT)
	{
		//turn right by 30 degrees

		double max_yaw_rate = 0.4;
		double turn = -1;

		geometry_msgs::Twist cmdvel;

		cmdvel.linear.x = 0.0;
		cmdvel.angular.z = turn * max_yaw_rate;
		println("turn right by 30 degrees");
		this->exe_pub.publish(cmdvel);
		current->status = EXECUTED;
		println("MoveBaseAction Status: EXECUTED");
		overwriteWorkingMemory(id_action,current);

		//goal.target_pose.pose.orientation.z = -sin(PI/12);
		//goal.target_pose.pose.orientation.w = cos(PI/12);
		//println("turn right by 30 degrees");
	}
	else
	{
		println("Physical Execution: Action Unknown");
	}

	//	client->sendGoal(goal);
	//	client->waitForResult();
	//
	//	if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	//	{
	//	    println("MoveBaseAction Status: EXECUTED");
	//	    current->status = EXECUTED;
	//	    overwriteWorkingMemory(id_action,current);
	//	}
	//	else
	//    {
	//        println("MoveBase Action Status: FAILED");
	//        current->status = FAILED;
	//	    overwriteWorkingMemory(id_action,current);
	//    }
}

void NavigationComponent::start()
{
    println("************************************************************");
    println("********** NavigationComponent ROS CAST Component **********");
    println("******************* Status: starting ***********************");
    println("************************************************************");
    
    char* argv[] = {};
	int argc = sizeof (argv) / sizeof (char *);
	
	println("******************** ROS init() ****************************");
	ros::init(argc, argv, "NavigationComponent");
	
	ros::NodeHandle node;

	//client = new MoveBaseClient("move_base", true);
	client = new MoveBaseClient("trp_as", true);
	client2d = new MoveBaseClient("move_base", true);
	
	this->exe_pub = node.advertise<geometry_msgs::Twist>("private/nav2/cmd_vel",1);
	tracks_vel_cmd_pub = node.advertise<nifti_robot_driver_msgs::Tracks>("/tracks_vel_cmd",1);
	
	k1 = 0.3;
	k2 = 0.3;
	displacement = 0.2;
	vel_g = 0.005;
	
	response = new eu::nifti::context::forwardResponse();
	response_flag = false;

	println("***************** Initializing filters *********************");
	addChangeFilter(createLocalTypeFilter<GoToNodeAction>(ADD),new MemberFunctionChangeReceiver<NavigationComponent>(this,&NavigationComponent::gotoNodeActionReceived));	
	//addChangeFilter(createLocalTypeFilter<MoveBaseAction>(ADD),new MemberFunctionChangeReceiver<NavigationComponent>(this,&NavigationComponent::moveBaseActionReceived2));	
	
	addChangeFilter(createLocalTypeFilter<MoveBaseAction>(ADD),new MemberFunctionChangeReceiver<NavigationComponent>(this,&NavigationComponent::mymoveBaseActionReceived2));

	std::string src = "findForwardUsingPointCloudsIterative"; //name of the component
    std::string id;
    std::string sa;
    addChangeFilter(createChangeFilter<eu::nifti::context::forwardResponse>(OVERWRITE,src,id,sa,ALLSA),new MemberFunctionChangeReceiver<NavigationComponent>(this,&NavigationComponent::myforwardResponseReceived));
}

void NavigationComponent::runComponent()
{
    println("************************************************************");
    println("********** NavigationComponent ROS CAST Component **********");
    println("******************* Status: running ************************");
    println("************************************************************");

    ros::NodeHandle node;
    tf::TransformListener tf;
    PosePtr metric_robot_pose = new Pose();
    string source_frameid = std::string("/base_link");
    string target_frameid = std::string("/map");

	while(node.ok())
	{
	    if(tf.canTransform(source_frameid, target_frameid, ros::Time()))
	    {
	        tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

	        try
	        {
	            //tf::StampedTransform echo_transform;
	            tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
                double yaw, pitch, roll;
                echo_transform.getBasis().getRPY(roll, pitch, yaw);
                //tf::Quaternion q = echo_transform.getRotation();
                tf::Vector3 v = echo_transform.getOrigin();
                metric_robot_pose->x = v.getX();
                metric_robot_pose->y = v.getY();
                metric_robot_pose->z = v.getZ();
                metric_robot_pose->theta = yaw;
                //println("Current position and orientation of the robot is : %s",to_string(metric_robot_pose).c_str());
                addToWorkingMemory(newDataID(),metric_robot_pose);
	        }
	        catch(tf::TransformException& ex)
	        {
	            cout << "Failure at "<< ros::Time::now() << endl;
				cout << "Exception thrown:" << ex.what()<< endl;
				cout << "The current list of frames is:" << endl;
				cout << tf.allFramesAsString() << endl;

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
    	return new NavigationComponent();
  	}
}
