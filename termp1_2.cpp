//
// Created by ys on 24. 11. 14.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <thread>
#include <mutex>

std::mutex data_mutex;
geometry_msgs::Twist received_cmd_vel;
trajectory_msgs::JointTrajectory joint_cmd;

double wheel_radius = 0.033; // 휠 반지름 (m)
double L = 0.185;            // 로봇 길이 (앞뒤 휠 거리)
double W = 0.185;            // 로봇 너비 (좌우 휠 거리)

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    received_cmd_vel = *msg;
}

void processCmdVel()
{
    ros::Rate rate(10);
    bool gripper_on = false;
    double lastRB = 0;
    while (ros::ok())
    {
        geometry_msgs::Twist local_cmd_vel;

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            local_cmd_vel = received_cmd_vel;
        }

        double v_x = local_cmd_vel.linear.x;
        double v_y = local_cmd_vel.linear.y;
        double omega = local_cmd_vel.angular.z;
        double RT = local_cmd_vel.angular.x;
        double RB = local_cmd_vel.angular.y;

        // 메카넘 휠 속도 계산
        double front_right_wheel = -(v_x + v_y + omega * (L + W)) / wheel_radius;
        double front_left_wheel = (v_x - v_y - omega * (L + W)) / wheel_radius;
        double rear_right_wheel = -(v_x - v_y + omega * (L + W)) / wheel_radius;
        double rear_left_wheel = (v_x + v_y - omega * (L + W)) / wheel_radius;


        trajectory_msgs::JointTrajectory local_joint_cmd;
        local_joint_cmd.joint_names = { "wheel1", "wheel2", "wheel3", "wheel4",
                                        "Arm1", "Arm2", "Arm3", "Arm4",
                                        "GripperL", "GripperR" };

        trajectory_msgs::JointTrajectoryPoint point;

// 휠 속도 제어 설정
        point.velocities = { front_right_wheel, front_left_wheel, rear_right_wheel, rear_left_wheel, 0, 0, 0, 0, 0, 0 };

// Arm 위치 제어 설정 (RT에 따라 계산된 값)
        point.positions = { 0, 0, 0, 0, 0, RT * 1.2, -RT * 1.2, 0, 0, 0 };

// Gripper 위치 제어 설정: 그리퍼에 위치 값 추가
        if (lastRB != RB)
        {
            if(RB == 1.0){
                if(gripper_on){
                    gripper_on = false;
                }
                else{
                    gripper_on = true;
                }
            }
        }
        lastRB = RB;
        if(gripper_on){
            point.positions[8] = -1.00;
            point.positions[9] = -1.00;
        }
        else{
            point.positions[8] = 0.50;
            point.positions[9] = 0.50;
        }

// 시간 설정
        point.time_from_start = ros::Duration(0.5);

// point를 local_joint_cmd에 추가
        local_joint_cmd.points.push_back(point);

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            joint_cmd = local_joint_cmd;
        }

        rate.sleep();
    }
}

void publishJointCmd(ros::Publisher& pub)
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        trajectory_msgs::JointTrajectory local_joint_cmd;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            local_joint_cmd = joint_cmd;
        }
        pub.publish(local_joint_cmd);
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "processor");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_cmd", 10);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::thread processing_thread(processCmdVel);
    std::thread publisher_thread(publishJointCmd, std::ref(pub));

    processing_thread.join();
    publisher_thread.join();

    return 0;
}
