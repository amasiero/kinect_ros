#ifndef KINECT_MOVEMENTS_H
#define KINECT_MOVEMENTS_H

class KinectMovements {
  public:
    static const int kNotRecognized = 0; // robot should not move
    static const int kBothArmsUp = 2;    // robot should move forward
    static const int kLeftArmUp = 1;     // robot should turn left
    static const int kRightArmUp = -1;     // robot should turn right
};

#endif
