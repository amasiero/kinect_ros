#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "kinect_movements.h"
#include "geometry_msgs/Twist.h"

void moveRobotForward();
void moveRobotBackward();
void turnRobotLeft();
void turnRobotRight();
void stopRobot();

ros::Publisher velPub;

void handlerCallback(const std_msgs::Int32::ConstPtr& msg) {
    switch (msg->data) {
        case KinectMovements::kBothArmsOpen:
            moveRobotForward();
            break;
        case KinectMovements::kBothArmsPushing:
            moveRobotBackward();
            break;
        case KinectMovements::kOnlyLeftArmOpen:
            turnRobotLeft();
            break;
        case KinectMovements::kOnlyRightArmOpen:
            turnRobotRight();
            break;
        default:
            stopRobot();
            break;
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "kinect_robot_controller");
    ros::NodeHandle node;

    ros::Rate loop_rate(10);

    ros::Subscriber kTfSub = node.subscribe<std_msgs::Int32>("kinect_handler", 1000, handlerCallback);
    velPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();

    return 0;
}

void moveRobotForward() {    
    geometry_msgs::Twist velCmd;
    velCmd.linear.x = 0.1;
    velCmd.angular.z = 0.0;

    ROS_INFO("FRENTE +++++++++++++++++++");
    velPub.publish(velCmd);
}

void turnRobotLeft() {
    geometry_msgs::Twist velCmd;
    velCmd.linear.x = 0.0;
    velCmd.angular.z = -0.1;

    ROS_INFO("<<< ESQUERDA");
    velPub.publish(velCmd);
}

void turnRobotRight() {
    geometry_msgs::Twist velCmd;
    velCmd.linear.x = 0.0;
    velCmd.angular.z = 0.1;

    ROS_INFO("DIREITA >>>");
    velPub.publish(velCmd);
}

void stopRobot() {
    geometry_msgs::Twist velCmd;
    velCmd.linear.x = 0.0;
    velCmd.angular.z = 0.0;

    velPub.publish(velCmd);
}

void moveRobotBackward() {
    geometry_msgs::Twist velCmd;
    velCmd.linear.x = -0.05;
    velCmd.angular.z = 0.0;

    ROS_INFO("---------------------- TRAS");
    velPub.publish(velCmd);
}
