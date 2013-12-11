#include "kinect_listener.h"
#include "kinect_movements.h"
#include <std_msgs/Int32.h>

bool isLeftArmUp(KinectTf * kTf);
bool isRightArmUp(KinectTf * kTf);
bool areBothArmsUp(KinectTf * kTf);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_handler");
    ros::NodeHandle node;
    
    ros::Publisher kinect_handler_publisher = node.advertise<std_msgs::Int32>("kinect_handler", 1000);
    ros::Rate loop_rate(10);

    KinectTf kTf;
    
    int count = 0;
    while (ros::ok())
    {
        std_msgs::Int32 msg;

        if (areBothArmsUp(&kTf)) {
            msg.data = KinectMovements::kBothArmsUp;
            ROS_INFO("BOTH ARMS UP");
        }
        else if (isRightArmUp(&kTf)) {
            msg.data = KinectMovements::kRightArmUp;
            ROS_INFO("RIGHT ARM UP");
        }
        else if (isLeftArmUp(&kTf)) {
            msg.data = KinectMovements::kLeftArmUp;
            ROS_INFO("LEFT ARM UP");
        }
        else {
            msg.data = KinectMovements::kNotRecognized;
            ROS_INFO("NOT RECOGNIZED");
        }

        kinect_handler_publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    
    return 0;
}

bool areBothArmsUp(KinectTf *kTf) {
    return isLeftArmUp(kTf) && isRightArmUp(kTf);
}

bool isLeftArmUp(KinectTf * kTf) {
    //ROS_INFO("Trying left hand");
    tf::StampedTransform * left_hand = kTf->left_hand();
    //ROS_INFO("Trying left elbow");
    tf::StampedTransform * left_elbow = kTf->left_elbow();
    //ROS_INFO("Trying left shoulder");
    tf::StampedTransform * left_shoulder = kTf->left_shoulder();

    if (left_hand && left_elbow && left_shoulder) {
        // x is y
        if (left_hand->getOrigin().x() > left_elbow->getOrigin().x() && left_elbow->getOrigin().x() > left_shoulder->getOrigin().x()) {

            float distanceBetweenHandAndElbow = left_hand->getOrigin().x() - left_elbow->getOrigin().x();
            //ROS_INFO("Distance between hand and elbow: %f", distanceBetweenHandAndElbow);
            if (distanceBetweenHandAndElbow > 0.01) {

                float distanceBetweenElbowAndShoulder = left_elbow->getOrigin().x() - left_shoulder->getOrigin().x();
                //ROS_INFO("Distance between elbow and shoulder: %f", distanceBetweenElbowAndShoulder);
                if (distanceBetweenElbowAndShoulder > 0.01) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool isRightArmUp(KinectTf * kTf) {
    //ROS_INFO("Trying left hand");
    tf::StampedTransform * right_hand = kTf->right_hand();
    //ROS_INFO("Trying left elbow");
    tf::StampedTransform * right_elbow = kTf->right_elbow();
    //ROS_INFO("Trying left shoulder");
    tf::StampedTransform * right_shoulder = kTf->right_shoulder();

    if (right_hand && right_elbow && right_shoulder) {
        // x is y
        if (right_hand->getOrigin().x() > right_elbow->getOrigin().x() && right_elbow->getOrigin().x() > right_shoulder->getOrigin().x()) {

            float distanceBetweenHandAndElbow = right_hand->getOrigin().x() - right_elbow->getOrigin().x();
            //ROS_INFO("Distance between hand and elbow: %f", distanceBetweenHandAndElbow);
            if (distanceBetweenHandAndElbow > 0.01) {

                float distanceBetweenElbowAndShoulder = right_elbow->getOrigin().x() - right_shoulder->getOrigin().x();
                //ROS_INFO("Distance between elbow and shoulder: %f", distanceBetweenElbowAndShoulder);
                if (distanceBetweenElbowAndShoulder > 0.01) {
                    return true;
                }
            }
        }
    }
    return false;
}
