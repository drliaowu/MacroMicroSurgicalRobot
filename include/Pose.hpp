#ifndef POSE
#define POSE
#include <tf2/LinearMath/Quaternion.h>

class Pose
{
public:
    tf2::Vector3 position;
    tf2::Quaternion orientation;

    Pose();
    Pose(tf2::Vector3 position, tf2::Quaternion orientation);
};
#endif
