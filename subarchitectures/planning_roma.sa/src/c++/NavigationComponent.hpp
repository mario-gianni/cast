#ifndef _NAVIGATION_COMPONENT_HPP_
#define _NAVIGATION_COMPONENT_HPP_

#include <cast/architecture.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <planning_roma.hpp>
#include <EmbeddedEclipse.hpp>
#include <topograph.hpp>
#include <robot_position.hpp>
#include <forwardRequest.hpp>
#include <forwardResponse.hpp>
#include "tf/transform_listener.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nifti_robot_driver_msgs/Tracks.h>

#define PI 3.14159265

using namespace eu::nifti::Planning::slice;
using namespace cast;
using namespace cast::cdl;
using namespace eu::nifti::env::topograph;
using namespace eu::nifti::env::position;
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigationComponent : public cast::ManagedComponent
{
    public:
        MoveBaseClient* client;

		MoveBaseClient* client2d;
        
        ros::Publisher exe_pub;
        
        ros::Publisher tracks_vel_cmd_pub;
        
        eu::nifti::context::forwardResponsePtr response;
        
        tf::StampedTransform echo_transform, real_robot_poseB;
        double k1;
        double k2;
        double robot_width;
        double displacement;
        double vel_g;
        double linear_vel;
        double angular_vel;
        nifti_robot_driver_msgs::Tracks tracks_cmd;
        
        boost::thread t_Thread;
        boost::thread m_Thread;
        
        bool response_flag;

    protected:
    
        bool waitForForwardRequest();
        virtual void start();
        virtual void runComponent();
        
        virtual void gotoNodeActionReceived(const WorkingMemoryChange&);
        virtual void moveBaseActionReceived(const WorkingMemoryChange&);
		virtual void forwardResponseReceived(const WorkingMemoryChange&);
		virtual void myforwardResponseReceived(const WorkingMemoryChange&);

        virtual void moveBaseActionReceived2(const WorkingMemoryChange&);
        virtual void mymoveBaseActionReceived2(const WorkingMemoryChange&);
        
        bool buildUserDefinedTrajectory(double,geometry_msgs::PoseStamped,geometry_msgs::Pose&,geometry_msgs::Twist&);
        bool getRobotCommands(double,tf::StampedTransform,double,double,geometry_msgs::Pose,geometry_msgs::Twist,double&,double&);
        void getTracksVelCmd(double,double,double,nifti_robot_driver_msgs::Tracks&);
        void realRobotPoseB(double,tf::StampedTransform,tf::StampedTransform&);

    private:
        string to_string(double);
        string to_string(PosePtr);

};

#endif
