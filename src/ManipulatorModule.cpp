#include "../include/ManipulatorModule.hpp"

ManipulatorModule::ManipulatorModule(bool isFirstJointPan, double diameter, double halfCurvatureAngle, int numJoints, double jointSeparationDistance)
{
    this->isFirstJointPan = isFirstJointPan;
    this->halfCurvatureAngle = halfCurvatureAngle;
    this->numJoints = numJoints;
    this->jointSeparationDistance = jointSeparationDistance;

    this->numPanJoints = numJoints / 2;

    // For an unveven number of joints, there will be one more of the first joint type (pan/tilt) than the other type
    if (isFirstJointPan)
    {
        this->numPanJoints += numJoints % 2;
    }

    this->numTiltJoints = numJoints - this->numTiltJoints;
}

double ManipulatorModule::GetCentralSeparation(double jointAngle)
{
    return 2 * this->curvatureRadius * (1 - cos(this->halfCurvatureAngle) * cos(jointAngle / 2));
}

void ManipulatorModule::SetTotalPanAngle(double angle)
{
    this->totalPanAngle = angle;
    this->panJointAngle = angle / this->numPanJoints;

    this->panCentralSeparation = GetCentralSeparation(this->panJointAngle);
}

void ManipulatorModule::SetTotalTiltAngle(double angle)
{
    this->totalTiltAngle = angle;
    this->tiltJointAngle = angle / this->numTiltJoints;

    this->tiltCentralSeparation = GetCentralSeparation(this->tiltJointAngle);
}

int ManipulatorModule::GetNumJoints()
{
    return this->numJoints;
}

bool ManipulatorModule::IsFirstJointPan()
{
    return this->isFirstJointPan;
}

Eigen::Matrix4d ManipulatorModule::GetPanJointTransform()
{
    double halfJointAngle = this->panJointAngle / 2;

    double cosJointAngle = cos(this->panJointAngle);
    double sinJointAngle = sin(this->panJointAngle);

    Eigen::Matrix4d transform(
        1, 0, 0, 0,
        0, cosJointAngle, -sinJointAngle, -this->panCentralSeparation * sin(halfJointAngle) - this->jointSeparationDistance * sinJointAngle,
        0, sinJointAngle, cosJointAngle, this->panCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
        0, 0, 0, 1
    );

    return transform;
}

Eigen::Matrix4d ManipulatorModule::GetTiltJointTransform()
{
    double halfJointAngle = this->tiltJointAngle / 2;

    double cosJointAngle = cos(this->tiltJointAngle);
    double sinJointAngle = sin(this->tiltJointAngle);

    Eigen::Matrix4d transform(
        cosJointAngle, 0, sinJointAngle, this->panCentralSeparation * sin(halfJointAngle) + this->jointSeparationDistance * sinJointAngle,
        0, 1, 0, 0,
        -sinJointAngle, 0, cosJointAngle, this->panCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
        0, 0, 0, 1
    );

    return transform;
}
