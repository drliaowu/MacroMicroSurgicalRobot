#include "../include/Pose.hpp"

Pose::Pose()
{
    this->position.setX(0.0);
    this->position.setY(0.0);
    this->position.setZ(0.0);

    this->orientation.setX(0.0);
    this->orientation.setY(0.0);
    this->orientation.setZ(0.0);
    this->orientation.setW(0.0);
}

Pose::Pose(tf2::Vector3 position, tf2::Quaternion orientation)
{
    this->position = position;
    this->orientation = orientation;
}
