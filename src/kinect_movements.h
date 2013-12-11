#ifndef KINECT_MOVEMENTS_H
#define KINECT_MOVEMENTS_H

class KinectMovements {
  public:
    static const int kNotRecognized = 0; // robot should not move
    static const int kBothArmsUp = 1;     // robot should turn left
    static const int kBothArmsOpen = -1;     // robot should turn right
    static const int kLeftArmOpen = 2; // robot moves forward
    static const int kRightArmOpen = -2; // robot moves backwards
};

#endif
