#include <math.h>
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
// #include <open_base/Velocity.h>

const double wheel_radius = 0.015 * 1.25686;

// This function is from <open_base/KinematicsInverse.h>
bool inverseMobile(
    const double &x_vel, const double &y_vel, const double &omega,
    double &v_left, double &v_back, double &v_right)
{
    const long double sqrt3 = std::sqrt(3.0L);
    const long double L = 0.04;  // wheel <-> centor
    long double V__m_x2 = - x_vel / 2.0L;
    long double sqrt3V__m_y2 = (sqrt3 * y_vel) / 2.0L;
    long double Lomega_p = L * omega;
    long double v_outer_left  = V__m_x2 - sqrt3V__m_y2 + Lomega_p;
    long double v_outer_back  = x_vel        + Lomega_p;
    long double v_outer_right = V__m_x2 + sqrt3V__m_y2 + Lomega_p;
    v_left = static_cast<double>(v_outer_left) / wheel_radius;
    v_back = static_cast<double>(v_outer_back) / wheel_radius;
    v_right = static_cast<double>(v_outer_right) / wheel_radius;
    return true;
}

void pushBackWithInterpolation(
    std::vector<geometry_msgs::Pose2D>& waypoints,
    const double& x, const double& y, const double& theta,
    const std::size_t &num)
{
    if (waypoints.size() == 0)
    {
        geometry_msgs::Pose2D init;
        init.x = 0.0;
        init.y = 0.0;
        init.theta = 0.0;
        waypoints.push_back(init);
    }
    // from
    geometry_msgs::Pose2D prev_target = waypoints.back();
    double dx = x - prev_target.x;
    double dy = y - prev_target.y;
    double dth = theta - prev_target.theta;
    for (std::size_t i = 1; i <= num; i++)
    {
        double i_double = static_cast<double>(i);
        double ratio = i_double / static_cast<double>(num);
        geometry_msgs::Pose2D temp;
        temp.x = prev_target.x + ratio * dx;
        temp.y = prev_target.y + ratio * dy;
        temp.theta = prev_target.theta + ratio * dth;
        waypoints.push_back(temp);
    }
}

bool isClose(
    const geometry_msgs::Pose2D& pose2d_A,
    const geometry_msgs::Pose2D& pose2d_B,
    const double &threshold)
{
    double dx = pose2d_B.x - pose2d_A.x;
    double dy = pose2d_B.y - pose2d_A.y;
    double dist = sqrt(dx * dx + dy * dy);
    return (dist < threshold);
}

void clampMagOfVelocity(const double &max_linear_vel,
    const double &vx, const double &vy, const double &vtheta,
    double &clamped_x, double &clamped_y, double &clamped_theta)
{
    double sum = sqrt(vx * vx + vy * vy);
    double ratio = (sum > max_linear_vel) ? (max_linear_vel / sum) : 1.0;    
    clamped_x = ratio * vx;
    clamped_y = ratio * vy;
    clamped_theta = ratio * vtheta;
}

geometry_msgs::Pose2D current;
bool rviz_started = false;
ros::Time sim_start_time;

void currentPoseCallback(const geometry_msgs::Pose2DConstPtr &msg)
{
    if (!rviz_started)
    {
        rviz_started = true;
        sim_start_time = ros::Time::now() + ros::Duration(3.0);
    }
    current.x = msg->x;
    current.y = msg->y;
    current.theta = msg->theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_command");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    sim_start_time = ros::Time::now() + ros::Duration(9999.0);

    // Publisher
    // ros::Publisher pub = nh.advertise<open_base::Velocity>("/open_base/sensor/wheel_velocity", 1);
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/open_base/joint_states", 1);
    
    // 토픽 이름 확인해야 함. 토픽 이름 앞에 노드 이름이 붙을 수 있음.
    // 만약 보내는 애랑 받는 애랑 이름 다르면
    // wheel_command.cpp랑 odometry.cpp 모두 앞에 / 그어서 "/sensor/wheel_velocity"로 맞추는걸 추천
    // open_base::Velocity wheel_msg;

    // Subscriber
    ros::Subscriber sub = nh.subscribe("/open_base/pose/mobile", 16, &currentPoseCallback);

    // Waypoints
    double length = 0.1;
    std::vector<geometry_msgs::Pose2D> waypoints;
    pushBackWithInterpolation(waypoints, length, 0.0, 0.0, 2);
    pushBackWithInterpolation(waypoints, -length, 0.0, 0.0, 2);
    pushBackWithInterpolation(waypoints, 0.0, 0.0, 0.0, 2);
    pushBackWithInterpolation(waypoints, 0.0, length, 0.0, 2);
    pushBackWithInterpolation(waypoints, 0.0, -length, 0.0, 2);
    // circle
    int max_i = 30;
    double radius = length;
    for (int i = 0; i <= max_i; i++)
    {
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(max_i);
        double x = sin(angle) * radius;
        double y = -cos(angle) * radius;
        pushBackWithInterpolation(waypoints, x, y, 0.0, 32);
    }
    pushBackWithInterpolation(waypoints, 0.0, 0.0, 0.0, 2);
    int max_point = waypoints.size();
    int i = 0;
    
    // speed
    const double speed = 0.05;  // m/sec
    current.x = 0.0; current.y = 0.0; current.theta = 0.0;
    ros::Time prev_time = ros::Time::now();

    sensor_msgs::JointState joint_state;
    std::vector<std::string> jnames = {
        "rim_left_joint", "roller_e_rim_left_joint", "roller_ne_rim_left_joint", "roller_n_rim_left_joint", "roller_nw_rim_left_joint",
        "roller_w_rim_left_joint", "roller_sw_rim_left_joint", "roller_s_rim_left_joint", "roller_se_rim_left_joint", "rim_back_joint",
        "roller_e_rim_back_joint", "roller_ne_rim_back_joint", "roller_n_rim_back_joint", "roller_nw_rim_back_joint", "roller_w_rim_back_joint",
        "roller_sw_rim_back_joint", "roller_s_rim_back_joint", "roller_se_rim_back_joint", "rim_right_joint", "roller_e_rim_right_joint",
        "roller_ne_rim_right_joint", "roller_n_rim_right_joint", "roller_nw_rim_right_joint", "roller_w_rim_right_joint", "roller_sw_rim_right_joint",
        "roller_s_rim_right_joint", "roller_se_rim_right_joint"};
    for (std::string& name : jnames)
    {
        joint_state.name.push_back(name);
        joint_state.position.push_back(0.0);
    }
    const int index_left = 0;
    const int index_back = 9;
    const int index_right = 18;

    ros::Rate rate(50);
    while (ros::ok() && (i < max_point))
    {
        // 현재 목표로 할 i번째 경유점 가져오기
        geometry_msgs::Pose2D local_target = waypoints.at(i);

        // Find linear velocity, angular velocity
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - prev_time).toSec();
        prev_time = current_time;

        double wheel_left = 0.0;
        double wheel_back = 0.0;
        double wheel_right= 0.0;

        double time_delay = (current_time - sim_start_time).toSec();
        if (time_delay > 0.0)
        {
            double raw_vx = (local_target.x - current.x) / dt;
            double raw_vy = (local_target.y - current.y) / dt;
            double raw_th = (local_target.theta - current.theta) / dt;

            double clamped_x, clamped_y, clamped_theta;
            clampMagOfVelocity(speed, raw_vx, raw_vy, raw_th, clamped_x, clamped_y, clamped_theta);

            // linear, angular -> three wheels
            inverseMobile(
                clamped_x, clamped_y, clamped_theta,
                wheel_left, wheel_back, wheel_right);
            // pub.publish(wheel_msg);
        }

        // wheel -> position
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[index_left] += wheel_left * dt;
        joint_state.position[index_back] += wheel_back * dt;
        joint_state.position[index_right] += wheel_right * dt;
        pub.publish(joint_state);

        if (time_delay > 0.0)
        {
            ROS_INFO_STREAM("current: ("
                << current.x << ", "
                << current.y << ", "
                << current.theta << ") "
                << "local target[" << i << "]: (" << std::setprecision(3)
                << local_target.x << ", "
                << local_target.y << ", "
                << local_target.theta << ") wheel vel: ("
                << wheel_left << ", "
                << wheel_back << ", "
                << wheel_right << ")");
        }
        else
        {
            ROS_INFO_STREAM("Not ready... " << time_delay << " secs left...");
        }
        ros::spinOnce();
        rate.sleep();

        if (isClose(current, local_target, 0.01)) { i++; }
    }
    ROS_INFO("The End!");
    ros::waitForShutdown();
    return 0;
}
