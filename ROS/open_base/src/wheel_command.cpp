#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <open_base/Velocity.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_command");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    // Publisher
    ros::Publisher pub = nh.advertise<open_base::Velocity>("sensor/wheel_velocity", 1);
    // 토픽 이름 확인해야 함. 토픽 이름 앞에 노드 이름이 붙을 수 있음.
    // 만약 보내는 애랑 받는 애랑 이름 다르면
    // wheel_command.cpp랑 odometry.cpp 모두 앞에 / 그어서 "/sensor/wheel_velocity"로 맞추는걸 추천
    open_base::Velocity wheel_msg;

    // Get parameters: Target velocity
    double linvel_x, linvel_y, angvel;
    nh.param<double>("linear_velocity_X", linvel_x, 0.0);
    nh.param<double>("linear_velocity_Y", linvel_y, 0.0);
    nh.param<double>("angular_velocity_Z", angvel, 0.0);

    // Kinematic data
    double wheel_radius = 0.1;
    // 여기다 모바일 로봇 바퀴 크기나 각종 안변하는 것들 적어두고,

    ros::Rate rate(100);
    while (ros::ok())
    {
        ROS_INFO("Hello, world!");

        // 여기서 모바일 키네마틱 계산


        wheel_msg.v_left = 0.0;
        wheel_msg.v_back = 0.0;
        wheel_msg.v_right = 0.0;
        pub.publish(wheel_msg);

        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}