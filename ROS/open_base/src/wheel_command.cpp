#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <open_base/Velocity.h>
#include <sensor_msgs/JointState.h>
#include <open_base/KinematicsInverse.h>
#include <open_base/KinematicsForward.h>



Eigen::Vector3f trajectory_maker(ros::ServiceClient client, Eigen::Vector3f start_pose, Eigen::Vector3f goal_pose, std::vector<Eigen::Vector3f> *traj, int segment)
{
    
    open_base::KinematicsInverse srv;
    long double r = 0.015*1.25686;

    Eigen::Vector3f result;

    for (int i=0; i<segment; i++)
    {
        srv.request.input.x = start_pose[0] + i*(goal_pose[0]-start_pose[0])/segment;
        srv.request.input.y = start_pose[1] + i*(goal_pose[1]-start_pose[1])/segment;
        srv.request.input.theta = start_pose[2] + i*(goal_pose[2]-start_pose[2])/segment;

        if (client.call(srv))
        {
            result[0] = srv.response.output.v_left/r;
            result[1] = srv.response.output.v_back/r;
            result[2] = srv.response.output.v_right/r;
        }
        
        traj->push_back(result);
    }

    return goal_pose;

    // std::cout << "successed" << std::endl;
}

void move_to_goal(ros::Publisher pub, std::vector<Eigen::Vector3f> *traj, int count)
{

    

    // for (int i=0; i<sizeof(&traj); i++)
    // {
        // std::cout<<traj->at(count)<<std::endl;

        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
 


        joint_msg.name.push_back("rim_left_joint");
        joint_msg.position.push_back(traj->at(count)[0]);
        joint_msg.name.push_back("roller_e_rim_left_joint");
        joint_msg.name.push_back("roller_ne_rim_left_joint");
        joint_msg.name.push_back("roller_n_rim_left_joint");
        joint_msg.name.push_back("roller_nw_rim_left_joint");
        joint_msg.name.push_back("roller_w_rim_left_joint");
        joint_msg.name.push_back("roller_sw_rim_left_joint");
        joint_msg.name.push_back("roller_s_rim_left_joint");
        joint_msg.name.push_back("roller_se_rim_left_joint");
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.name.push_back("rim_back_joint");
        joint_msg.position.push_back(traj->at(count)[1]);
        joint_msg.name.push_back("roller_e_rim_back_joint");
        joint_msg.name.push_back("roller_ne_rim_back_joint");
        joint_msg.name.push_back("roller_n_rim_back_joint");
        joint_msg.name.push_back("roller_nw_rim_back_joint");
        joint_msg.name.push_back("roller_w_rim_back_joint");
        joint_msg.name.push_back("roller_sw_rim_back_joint");
        joint_msg.name.push_back("roller_s_rim_back_joint");
        joint_msg.name.push_back("roller_se_rim_back_joint");
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.name.push_back("rim_right_joint");
        joint_msg.position.push_back(traj->at(count)[2]);
        joint_msg.name.push_back("roller_e_rim_right_joint");
        joint_msg.name.push_back("roller_ne_rim_right_joint");
        joint_msg.name.push_back("roller_n_rim_right_joint");
        joint_msg.name.push_back("roller_nw_rim_right_joint");
        joint_msg.name.push_back("roller_w_rim_right_joint");
        joint_msg.name.push_back("roller_sw_rim_right_joint");
        joint_msg.name.push_back("roller_s_rim_right_joint");
        joint_msg.name.push_back("roller_se_rim_right_joint");
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);


        pub.publish(joint_msg);

        // std::cout<<joint_msg.position[0]<< ",  "<<std::endl;

        // open_base::Velocity vel_msg;

        // vel_msg.v_left = traj->at(i)[0];
        // vel_msg.v_back = traj->at(i)[1];
        // vel_msg.v_right = traj->at(i)[2];
        
        // pub.publish(vel_msg);


    // }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_command");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    // Publisher
    ros::Publisher vel_pub = nh.advertise<open_base::Velocity>("/new/wheel_velocity", 1);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/open_base/joint_states", 1);
    ros::ServiceClient inverse_client = nh.serviceClient<open_base::KinematicsInverse>("/open_base/kinematics_inverse_mobile");
    ros::ServiceClient forward_client = nh.serviceClient<open_base::KinematicsForward>("/open_base/kinematics_forward_mobile");
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
    
    std::vector<Eigen::Vector3f> traj;
    std::vector<Eigen::Vector3f> *traj_v;
    traj_v = &traj; 

    Eigen::Vector3f start;
    Eigen::Vector3f result;
    Eigen::Vector3f goal;

    start << 0.0,0.0,0.0;
    goal << 1.0,0.0,0.0;
    start = trajectory_maker(inverse_client, start, goal, traj_v, 25);
    goal << -1.0,0.0,0.0;
    start = trajectory_maker(inverse_client, start, goal, traj_v, 50);
    goal << 0.0,0.0,0.0;
    start = trajectory_maker(inverse_client, start, goal, traj_v, 25);
    goal << 0.0,1.0,0.0;
    start = trajectory_maker(inverse_client, start, goal, traj_v, 25);
    goal << 0.0,-1.0,0.0;
    start = trajectory_maker(inverse_client, start, goal, traj_v, 50);

    int cnt_c=0;
    for (double d=0.0; d < 2.0 * M_PI; d=d+2.0*M_PI/100.0)
    {
        goal<<start[0]*cos(d)-start[1]*sin(d), start[0]*sin(d)+start[1]*cos(d), 0.0;
        if (cnt_c==0)
        {
            result = trajectory_maker(inverse_client, start, goal, traj_v, 1);
            std::cout << result[0] <<", " << result[1] <<std::endl;
        }
        else
        {
            result = trajectory_maker(inverse_client, result, goal, traj_v, 1);
            std::cout << result[0] <<", " << result[1] <<std::endl;
        }
        cnt_c++;

    }
    goal << 0.0,0.0,0.0;
    trajectory_maker(inverse_client, start, goal, traj_v, 25);

    int cnt=0;

    ros::Rate rate(10);
    while (ros::ok())
    {        
        if (cnt<traj.size())
        {
            move_to_goal(joint_pub, traj_v, cnt);
            cnt++;
        }
        else
        {            
            cnt=0;
        }          

        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}