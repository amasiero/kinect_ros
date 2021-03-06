#ifndef KINECT_MOVEMENTS_H
#define KINECT_MOVEMENTS_H

class KinectMovements {
  public:
    static const int kNotRecognized = 0; // robot should not move
    static const int kBothArmsPushing = 1;     // robot should move backward
    static const int kBothArmsOpen = -1;     // robot should move forward
    static const int kOnlyLeftArmOpen = 2; // robot should turn left
    static const int kOnlyRightArmOpen = -2; // robot should turn right
};

#endif
