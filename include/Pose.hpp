#ifndef Pose
#include <tf2/LinearMath/Quaternion.h>
#endif

class Pose
{
public:
    tf2::Vector3 position;
    tf2::Quaternion orientation;

    Pose();
    Pose(tf2::Vector3 position, tf2::Quaternion orientation);
};
