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
    this->SetTotalPanAngle(0.0);
    this->SetTotalTiltAngle(0.0);
}

double ManipulatorModule::CalculateCentralSeparation(double jointAngle)
{
    return 2 * this->curvatureRadius * (1 - cos(this->halfCurvatureAngle) * cos(jointAngle / 2));
}

double ManipulatorModule::CalculateIsolatedLengthDelta(bool isLeftTendon, int numPrimaryJoints, int numSecondaryJoints, double primaryJointAngle, double secondaryJointAngle)
{
    return 2 * this->curvatureRadius * (
        numPrimaryJoints * (cos(this->halfCurvatureAngle) - cos(this->halfCurvatureAngle + (isLeftTendon ? -1 : 1) * primaryJointAngle / 2)) +
        numSecondaryJoints * (1 - cos(secondaryJointAngle / 2))
    );
}

Eigen::Matrix4d ManipulatorModule::CalculatePanJointTransform()
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

Eigen::Matrix4d ManipulatorModule::CalculateTiltJointTransform()
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

void ManipulatorModule::SetTotalPanAngle(double angle)
{
    this->totalPanAngle = angle;
    this->panJointAngle = this->totalPanAngle / this->numPanJoints;

    this->panCentralSeparation = this->CalculateCentralSeparation(this->panJointAngle);

    this->panJointTransform = this->CalculatePanJointTransform();

    this->isolatedPanLengthDelta = this->CalculateIsolatedLengthDelta(true, this->numPanJoints, this->numTiltJoints, this->panJointAngle, this->tiltJointAngle);
}

void ManipulatorModule::SetTotalTiltAngle(double angle)
{
    this->totalTiltAngle = angle;
    this->tiltJointAngle = this->totalTiltAngle / this->numTiltJoints;

    this->tiltCentralSeparation = this->CalculateCentralSeparation(this->tiltJointAngle);

    this->tiltJointTransform = this->CalculateTiltJointTransform();

    this->isolatedTiltLengthDelta = this->CalculateIsolatedLengthDelta(true, this->numTiltJoints, this->numPanJoints, this->tiltJointAngle, this->panJointAngle);
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
    return this->panJointTransform;
}

Eigen::Matrix4d ManipulatorModule::GetTiltJointTransform()
{
    return this->tiltJointTransform;
}

double ManipulatorModule::GetIsolatedPanLengthDelta()
{
    return this->isolatedPanLengthDelta;
}

double ManipulatorModule::GetIsolatedTiltLengthDelta()
{
    return this->isolatedTiltLengthDelta;
}
