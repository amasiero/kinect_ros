#ifndef KINECT_LISTENER_H
#define KINECT_LISTENER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <cstddef>

class KinectTf
{
  private:
    static const std::string kFixedFrame;
    static const std::string kTargetFrame;

    tf::TransformListener listener_;

    /* These are attributes used to transform everything we need */
    tf::StampedTransform left_hand_;
    tf::StampedTransform left_elbow_;
    tf::StampedTransform left_shoulder_;

    tf::StampedTransform right_hand_;
    tf::StampedTransform right_elbow_;
    tf::StampedTransform right_shoulder_;

    bool set_transform(const std::string targetFrame, tf::StampedTransform& transform);

  public:
    /* Kinect acts as if we were in front of a mirror, so our left side is what kinect thinks is our right one.*/

    inline tf::StampedTransform *left_hand() {
        if (set_transform("right_hand_1", left_hand_)) {
            return &left_hand_;
        }
        else {
            return NULL;
        }
    }

    inline tf::StampedTransform *right_hand() {
        if (set_transform("left_hand_1", right_hand_)) {
            return &right_hand_;
        }
        else {
            return NULL;
        }
    }

    inline tf::StampedTransform *left_elbow() {
        if (set_transform("right_elbow_1", left_elbow_)) {
            return &left_elbow_;
        }
        else {
            return NULL;
        }
    }

    inline tf::StampedTransform *right_elbow() {
        if (set_transform("left_elbow_1", right_elbow_)) {
            return &right_elbow_;
        }
        else {
            return NULL;
        }
    }

    inline tf::StampedTransform *left_shoulder() {
        if (set_transform("right_shoulder_1", left_shoulder_)) {
            return &left_shoulder_;
        }
        else {
            return NULL;
        }
    }

    inline tf::StampedTransform *right_shoulder() {
        if (set_transform("left_shoulder_1", right_shoulder_)) {
            return &right_shoulder_;
        }
        else {
            return NULL;
        }
    }
};

const std::string KinectTf::kFixedFrame = "camera_depth_optical_frame";
const std::string KinectTf::kTargetFrame = "camera_depth_optical_frame";

bool KinectTf::set_transform(const std::string sourceFrame, tf::StampedTransform& transform) {
    try {
        if (listener_.canTransform(kTargetFrame, ros::Time(0), sourceFrame, ros::Time(0), kFixedFrame)) {
            listener_.lookupTransform(kTargetFrame, sourceFrame, ros::Time(0), transform);

            if (ros::Time::now().toSec() - transform.stamp_.toSec() < 1.0) {
                return true;
            }
        }
        else {
            //ROS_INFO("Cannot transform from %s to %s using fixed frame %s", sourceFrame.c_str(), kTargetFrame.c_str(), kFixedFrame.c_str());
        }
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }

    return false;
}

#endif
