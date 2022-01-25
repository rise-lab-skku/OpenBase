#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <kdl/frames.hpp>

#include <open_base/KinematicsForward.h>
#include <open_base/Velocity.h>

long double duration;

int i;

ros::ServiceClient kinematicsForwardMobile;

ros::ServiceClient kinematicsForwardWorld;

std::string openBaseString = "open_base";

std::string originString = "origin";

geometry_msgs::Pose2D pose;

geometry_msgs::Pose poseCheat;

geometry_msgs::Pose2D poseMobile;

geometry_msgs::Pose2D poseWorld;

ros::Publisher publisherMobile;

ros::Publisher publisherWorld;

ros::Publisher publisherOdom;

long double r;

double rotX, rotY, rotZ;

KDL::Rotation rotation;

open_base::KinematicsForward service;

ros::Time timeCurrent;
ros::Time timePrevious;

void onEncoderMessage(const open_base::Velocity::ConstPtr& input){
    static tf::TransformBroadcaster br;
    service.request.input.v_left  = input->v_left  * r;
    service.request.input.v_back  = input->v_back  * r;
    service.request.input.v_right = input->v_right * r;
    timeCurrent = ros::Time::now();
    duration = (timeCurrent - timePrevious).toSec();
    timePrevious = timeCurrent;
    kinematicsForwardWorld.call(service);
    poseWorld.x     = (service.response.output.x     * duration) + poseWorld.x    ;
    poseWorld.y     = (service.response.output.y     * duration) + poseWorld.y    ;
    poseWorld.theta = (service.response.output.theta * duration) + poseWorld.theta;
    if ((!std::isnan(poseWorld.x)) && (!std::isnan(poseWorld.y)) && (!std::isnan(poseWorld.theta))) {
        publisherWorld.publish(poseWorld);
        poseWorld.x     = poseWorld.x    ;
        poseWorld.y     = poseWorld.y    ;
        poseWorld.theta = poseWorld.theta;
    }
    kinematicsForwardMobile.call(service);
    poseWorld.x     = (service.response.output.x     * duration) + poseMobile.x    ;
    poseWorld.y     = (service.response.output.y     * duration) + poseMobile.y    ;
    poseWorld.theta = (service.response.output.theta * duration) + poseMobile.theta;
    if ((!std::isnan(poseWorld.x)) && (!std::isnan(poseWorld.y)) && (!std::isnan(poseWorld.theta))) {
        publisherMobile.publish(poseWorld);
        poseMobile.x     = poseWorld.x    ;
        poseMobile.y     = poseWorld.y    ;
        poseMobile.theta = poseWorld.theta;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(poseMobile.theta);

        // First, we'll publish the transform over tf        
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = timeCurrent;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "origin_link";

        odom_trans.transform.translation.x = poseMobile.x;
        odom_trans.transform.translation.y = poseMobile.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        br.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = timeCurrent;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = poseMobile.x;
        odom.pose.pose.position.y = poseMobile.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "origin_link";
        // odom.twist.twist.linear.x = vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = vth;

        //publish the message
        publisherOdom.publish(odom);
    }
}

void onGazeboMessage(const gazebo_msgs::LinkStates::ConstPtr& input){
    for (i = 0; i < input->name.size(); i++) {
        if (((input->name[i]).find(openBaseString) == std::string::npos) || ((input->name[i]).find(originString) == std::string::npos)) {
            continue;
        }
        poseCheat = input->pose[i];
        rotation = KDL::Rotation::Quaternion(poseCheat.orientation.x, poseCheat.orientation.y, poseCheat.orientation.z, poseCheat.orientation.w);
        rotation.GetRPY(rotX, rotY, rotZ);
        poseWorld.x     = poseCheat.position.x;
        poseWorld.y     = poseCheat.position.y;
        poseWorld.theta = rotZ;
        publisherWorld.publish(poseWorld);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    ros::NodeHandle node;
    while (!ros::Time::waitForValid()) {}
    {
        double parameter;
        if (!node.getParam("parameter/wheel/radius", parameter)) {
            ROS_ERROR("Could not get wheel radius from parameter server.");
            return -1;
        }
        r = parameter;
        if (!node.getParam("parameter/initial/x", parameter)) {
            parameter = 0;
        }
        poseMobile.x = poseWorld.x  = parameter;
        if (!node.getParam("parameter/initial/y", parameter)) {
            parameter = 0;
        }
        poseMobile.y = poseWorld.y  = parameter;
        if (!node.getParam("parameter/initial/theta", parameter)) {
            parameter = 0;
        }
        poseMobile.theta = poseWorld.theta  = parameter;
    }
    ros::Subscriber subscriber;
    {
        std::string poseCheatString = "pose_cheat";
        std::string argument;
        bool poseCheatFound = false;
        for (int j = 0; j < argc; j++) {
            argument = std::string(argv[j]);
            if (poseCheatString.compare(argument) == 0) {
                poseCheatFound = true;
            }
        }
        if (poseCheatFound) {
            subscriber = node.subscribe("/gazebo/link_states", 1, onGazeboMessage);
        } else {
            subscriber = node.subscribe("sensor/wheel_velocity", 1, onEncoderMessage);
        }
    }
    kinematicsForwardWorld  = node.serviceClient<open_base::KinematicsForward>("kinematics_forward_world" );
    kinematicsForwardMobile = node.serviceClient<open_base::KinematicsForward>("kinematics_forward_mobile");
    publisherWorld  = node.advertise<geometry_msgs::Pose2D>("pose/world" , 100);
    publisherMobile = node.advertise<geometry_msgs::Pose2D>("pose/mobile", 100);
    publisherOdom = node.advertise<nav_msgs::Odometry>("odom", 100);
    tf::TransformBroadcaster odom_broadcaster;
    ros::spin();
    return 0;
}
