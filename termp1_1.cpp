#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <thread>
#include <mutex>

std::mutex data_mutex;
geometry_msgs::Twist received_cmd_vel;
trajectory_msgs::JointTrajectory joint_cmd;

double wheel_radius = 0.066;
double wheel_base = 0.178;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    received_cmd_vel = *msg;
}

void processCmdVel() {
    ros::Rate rate(10);
    while (ros::ok()) {
        geometry_msgs::Twist local_cmd_vel;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            local_cmd_vel = received_cmd_vel;
        }

        double v = local_cmd_vel.linear.x;      // 전진 속도
        double omega = local_cmd_vel.angular.z; // 회전 속도

        // 각 바퀴의 각속도 계산
        double left_wheel_velocity = (v - (omega * wheel_base / 2.0)) / wheel_radius;
        double right_wheel_velocity = (v + (omega * wheel_base / 2.0)) / wheel_radius;

        trajectory_msgs::JointTrajectory local_joint_cmd;
        local_joint_cmd.joint_names = {"wheel1", "wheel2"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.velocities = {left_wheel_velocity, right_wheel_velocity};
        local_joint_cmd.points.push_back(point);
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            joint_cmd = local_joint_cmd;
        }
        rate.sleep();
    }
}

void publishJointCmd(ros::Publisher &pub) {
    ros::Rate rate(10);
    while (ros::ok()) {
        trajectory_msgs::JointTrajectory local_joint_cmd;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            local_joint_cmd = joint_cmd;
        }
        pub.publish(local_joint_cmd);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
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
