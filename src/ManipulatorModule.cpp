#include "../include/ManipulatorModule.hpp"

ManipulatorModule::ManipulatorModule(bool isFirstJointPan, double diameter, double halfCurvatureAngle, int numJoints, double jointSeparationDistance)
{
    this->isFirstJointPan = isFirstJointPan;
    this->diameter = diameter;
    this->halfCurvatureAngle = halfCurvatureAngle;

    this->curvatureRadius = this->diameter / (2.0 * sin(this->halfCurvatureAngle));

    this->numJoints = numJoints;
    this->jointSeparationDistance = jointSeparationDistance;

    this->numPanJoints = this->numJoints / 2;

    // For an unveven number of joints, there will be one more of the first joint type (pan/tilt) than the other type
    if (isFirstJointPan)
    {
        this->numPanJoints += this->numJoints % 2;
    }

    this->numTiltJoints = this->numJoints - this->numPanJoints;
}

double ManipulatorModule::GetCentralSeparation(double jointAngle)
{
    return 2 * this->curvatureRadius * (1 - cos(this->halfCurvatureAngle) * cos(jointAngle / 2));
}

void ManipulatorModule::SetTotalPanAngle(double angle)
{
    this->totalPanAngle = angle;
    this->panJointAngle = this->totalPanAngle / this->numPanJoints;

    this->panCentralSeparation = 2 * this->curvatureRadius * (1 - cos(this->halfCurvatureAngle) * cos(this->panJointAngle / 2));
}

void ManipulatorModule::SetTotalTiltAngle(double angle)
{
    this->totalTiltAngle = angle;
    this->tiltJointAngle = this->totalTiltAngle / this->numTiltJoints;

    this->tiltCentralSeparation = 2 * this->curvatureRadius * (1 - cos(this->halfCurvatureAngle) * cos(this->tiltJointAngle / 2));
}

void ManipulatorModule::ApplyPanAngleDelta(double delta)
{
    this->SetTotalPanAngle(this->totalPanAngle + delta);
}

void ManipulatorModule::ApplyTiltAngleDelta(double delta)
{
    this->SetTotalTiltAngle(this->totalTiltAngle + delta);
}

int ManipulatorModule::GetNumJoints()
{
    return this->numJoints;
}

int ManipulatorModule::GetNumPanJoints()
{
    return this->numPanJoints;
}

int ManipulatorModule::GetNumTiltJoints()
{
    return this->numTiltJoints;
}

bool ManipulatorModule::IsFirstJointPan()
{
    return this->isFirstJointPan;
}

double ManipulatorModule::GetPanJointAngle()
{
    return this->panJointAngle;
}

double ManipulatorModule::GetTiltJointAngle()
{
    return this->tiltJointAngle;
}

double ManipulatorModule::GetHalfCurvatureAngle()
{
    return this->halfCurvatureAngle;
}

double ManipulatorModule::GetCurvatureRadius()
{
    return this->curvatureRadius;
}

double ManipulatorModule::GetPanCentralSeparation()
{
    return this->panCentralSeparation;
}

double ManipulatorModule::GetTiltCentralSeparation()
{
    return this->tiltCentralSeparation;
}

double ManipulatorModule::GetJointSeparationDistance()
{
    return this->jointSeparationDistance;
}

Eigen::Matrix4d ManipulatorModule::GetPanJointTransform()
{
    double halfJointAngle = this->panJointAngle / 2;

    double cosJointAngle = cos(this->panJointAngle);
    double sinJointAngle = sin(this->panJointAngle);

    Eigen::Matrix4d transform;
    transform << 1, 0, 0, 0,
                 0, cosJointAngle, -sinJointAngle, -this->panCentralSeparation * sin(halfJointAngle) - this->jointSeparationDistance * sinJointAngle,
                 0, sinJointAngle, cosJointAngle, this->panCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
                 0, 0, 0, 1;

    return transform;
}

Eigen::Matrix4d ManipulatorModule::GetTiltJointTransform()
{
    double halfJointAngle = this->tiltJointAngle / 2;

    double cosJointAngle = cos(this->tiltJointAngle);
    double sinJointAngle = sin(this->tiltJointAngle);

    Eigen::Matrix4d transform;
    transform << cosJointAngle, 0, sinJointAngle, this->tiltCentralSeparation * sin(halfJointAngle) + this->jointSeparationDistance * sinJointAngle,
                 0, 1, 0, 0,
                 -sinJointAngle, 0, cosJointAngle, this->tiltCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
                 0, 0, 0, 1;

    return transform;
}

double ManipulatorModule::GetIsolatedPanLengthDelta(bool isLeftTendon)
{
    return 2 * this->curvatureRadius * (
        this->numPanJoints * (cos(this->halfCurvatureAngle) - cos(this->halfCurvatureAngle + (-1 * (int)isLeftTendon) * this->panJointAngle / 2)) +
        this->numTiltJoints * (1 - cos(this->tiltJointAngle / 2))
    );
}

double ManipulatorModule::GetIsolatedTiltLengthDelta(bool isLeftTendon)
{
    return 2 * this->curvatureRadius * (
        this->numTiltJoints * (cos(this->halfCurvatureAngle) - cos(this->halfCurvatureAngle + (-1 * (int)isLeftTendon) * this->tiltJointAngle / 2)) +
        this->numPanJoints * (1 - cos(this->panJointAngle / 2))
    );
}
