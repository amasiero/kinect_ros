#include "kinect_listener.h"
#include "kinect_movements.h"
#include <std_msgs/Int32.h>

bool isLeftArmUp(KinectTf * kTf);
bool isRightArmUp(KinectTf * kTf);
bool areBothArmsUp(KinectTf * kTf);
bool isLeftArmOpen(KinectTf *kTf);
bool isRightArmOpen(KinectTf * kTf);
bool isOnlyRightArmOpen(KinectTf * kTf);
bool isOnlyLeftArmOpen(KinectTf * kTf);

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
            ROS_INFO("            BOTH");
        }
        else if (isOnlyRightArmOpen(&kTf)) {
            msg.data = KinectMovements::kRightArmUp;
            ROS_INFO("                               RIGHT");
        }
        else if (isOnlyLeftArmOpen(&kTf)) {
            msg.data = KinectMovements::kLeftArmUp;
            ROS_INFO("LEFT");
        }
        else {
            msg.data = KinectMovements::kNotRecognized;
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
    tf::StampedTransform * left_hand = kTf->left_hand();
    tf::StampedTransform * left_elbow = kTf->left_elbow();
    tf::StampedTransform * left_shoulder = kTf->left_shoulder();

    if (left_hand && left_elbow && left_shoulder) {
        // x is y
        if (left_hand->getOrigin().x() > left_elbow->getOrigin().x()) {

            float distanceBetweenHandAndElbow = left_hand->getOrigin().x() - left_elbow->getOrigin().x();
            //ROS_INFO("Distance hands elbow left %f", distanceBetweenHandAndElbow);
            if (distanceBetweenHandAndElbow > 0.01 && distanceBetweenHandAndElbow < 0.1) {

                float distanceBetweenElbowAndShoulder = left_elbow->getOrigin().x() - left_shoulder->getOrigin().x();
                //ROS_INFO("Distance elbow shoulder left %f", distanceBetweenElbowAndShoulder);
                if (distanceBetweenElbowAndShoulder > 0.01 && distanceBetweenElbowAndShoulder < 0.1) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool isRightArmUp(KinectTf * kTf) {
    tf::StampedTransform * right_hand = kTf->right_hand();
    tf::StampedTransform * right_elbow = kTf->right_elbow();
    tf::StampedTransform * right_shoulder = kTf->right_shoulder();

    if (right_hand && right_elbow && right_shoulder) {
        // x is y
        if (right_hand->getOrigin().x() > right_elbow->getOrigin().x()) {

            float distanceBetweenHandAndElbow = right_hand->getOrigin().x() - right_elbow->getOrigin().x();
            //ROS_INFO("Distance hands elbow right %f", distanceBetweenHandAndElbow);
            if (distanceBetweenHandAndElbow > 0.01 && distanceBetweenHandAndElbow < 0.1) {

                float distanceBetweenElbowAndShoulder = right_elbow->getOrigin().x() - right_shoulder->getOrigin().x();
                //ROS_INFO("Distance elbow shoulder right %f", distanceBetweenElbowAndShoulder);
                if (distanceBetweenElbowAndShoulder > 0.01 && distanceBetweenElbowAndShoulder < 0.1) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool isLeftArmOpen(KinectTf * kTf) {
    tf::StampedTransform * left_hand = kTf->left_hand();
    tf::StampedTransform * left_elbow = kTf->left_elbow();
    tf::StampedTransform * left_shoulder = kTf->left_shoulder();

    if (left_hand && left_elbow && left_shoulder) {
        // x is y and vice versa
        if (left_hand->getOrigin().y() < left_elbow->getOrigin().y() && left_elbow->getOrigin().y() < left_shoulder->getOrigin().y()) {

            float distanceBetweenHandAndElbowOnX = left_hand->getOrigin().y() - left_elbow->getOrigin().y();
            if (distanceBetweenHandAndElbowOnX > -0.35 && distanceBetweenHandAndElbowOnX < -0.25) {
                return true;
            }
        }
    }

    return false;
}

bool isRightArmOpen(KinectTf * kTf) {
    tf::StampedTransform * right_hand = kTf->right_hand();
    tf::StampedTransform * right_elbow = kTf->right_elbow();
    tf::StampedTransform * right_shoulder = kTf->right_shoulder();

    if (right_hand && right_elbow && right_shoulder) {
        // x is y and vice versa
        if (right_hand->getOrigin().y() > right_elbow->getOrigin().y() && right_elbow->getOrigin().y() > right_shoulder->getOrigin().y()) {

            float distanceBetweenHandAndElbowOnX = right_hand->getOrigin().y() - right_elbow->getOrigin().y();

            if (distanceBetweenHandAndElbowOnX < 0.35 && distanceBetweenHandAndElbowOnX > 0.25) {
                return true;
            }
        }
    }

    return false;
}

bool isOnlyRightArmOpen(KinectTf * kTf) {
    return isRightArmOpen(kTf) && !isLeftArmOpen(kTf);
}

bool isOnlyLeftArmOpen(KinectTf * kTf) {
    return isLeftArmOpen(kTf) && !isRightArmOpen(kTf);
}
