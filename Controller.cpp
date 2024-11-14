//
// Created by ys on 24. 11. 14.
//

#include "Gamepad.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <thread>

Gamepad* gamepad;
double x, y, yaw, RT, RB;

void consoleDataHandler()
{
    while (ros::ok())
    {
        gamepad->Read();
        y = -(double)gamepad->mJoystickAxis[0] / 30767;
        x = -(double)gamepad->mJoystickAxis[1] / 30767;
        yaw = -(double)gamepad->mJoystickAxis[3] / 30767;

        //this is for gripper
        RT = (double)gamepad->mJoystickAxis[5] / 30767;
        RB = (double)gamepad->mJoystickButton[5];

    }
}

void publishData(ros::Publisher& pub)
{
    ros::Rate rate(60);  // 1 Hz로 publish

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = x;
        msg.linear.y = y;
        msg.angular.z = yaw * 3.14;
        msg.angular.x = RT;
        msg.angular.y = RB;
        pub.publish(msg);
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "console");
    gamepad = Gamepad::getInstance();
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    std::thread console_thread(consoleDataHandler);
    std::thread publish_thread(publishData, std::ref(chatter_pub));
    ros::waitForShutdown(); // 노드가 종료될 때까지 대기
    ros::AsyncSpinner spinner(1);
    spinner.start();
    publish_thread.join();
    console_thread.join();

}