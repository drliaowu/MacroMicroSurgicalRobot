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

    this->totalPanAngle = 0.0;
    this->panJointAngle = this->totalPanAngle / this->numPanJoints;

    this->panCentralSeparation = 2 * this->curvatureRadius * (1 - cos(this->halfCurvatureAngle) * cos(this->panJointAngle / 2));

    double halfJointAngle = this->panJointAngle / 2;
    double cosJointAngle = cos(this->panJointAngle);
    double sinJointAngle = sin(this->panJointAngle);

    this->panJointTransform = Eigen::Matrix4d::Zero();

    this->panJointTransform << 1, 0, 0, 0,
                 0, cosJointAngle, -sinJointAngle, -this->panCentralSeparation * sin(halfJointAngle) - this->jointSeparationDistance * sinJointAngle,
                 0, sinJointAngle, cosJointAngle, this->panCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
                 0, 0, 0, 1;

    this->isolatedPanLengthDelta = 2 * this->curvatureRadius * (
        this->numPanJoints * (cos(this->halfCurvatureAngle) - cos(this->halfCurvatureAngle -this->panJointAngle / 2)) +
        this->numTiltJoints * (1 - cos(this->tiltJointAngle / 2))
    );

    this->totalTiltAngle = 0.0;
    this->tiltJointAngle = this->totalTiltAngle / this->numTiltJoints;

    this->tiltCentralSeparation = 2 * this->curvatureRadius * (1 - cos(this->halfCurvatureAngle) * cos(this->tiltJointAngle / 2));

    halfJointAngle = this->tiltJointAngle / 2;
    cosJointAngle = cos(this->tiltJointAngle);
    sinJointAngle = sin(this->tiltJointAngle);

    this->tiltJointTransform = Eigen::Matrix4d::Zero();
    this->tiltJointTransform << cosJointAngle, 0, sinJointAngle, this->tiltCentralSeparation * sin(halfJointAngle) + this->jointSeparationDistance * sinJointAngle,
                 0, 1, 0, 0,
                 -sinJointAngle, 0, cosJointAngle, this->tiltCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
                 0, 0, 0, 1;

    this->isolatedTiltLengthDelta = 2 * this->curvatureRadius * (
        this->numTiltJoints * (cos(this->halfCurvatureAngle) - cos(this->halfCurvatureAngle -this->tiltJointAngle / 2)) +
        this->numPanJoints * (1 - cos(this->panJointAngle / 2))
    );
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

    double halfJointAngle = this->panJointAngle / 2;
    double cosJointAngle = cos(this->panJointAngle);
    double sinJointAngle = sin(this->panJointAngle);

    this->panJointTransform << 1, 0, 0, 0,
                 0, cosJointAngle, -sinJointAngle, -this->panCentralSeparation * sin(halfJointAngle) - this->jointSeparationDistance * sinJointAngle,
                 0, sinJointAngle, cosJointAngle, this->panCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
                 0, 0, 0, 1;

    this->isolatedPanLengthDelta = this->CalculateIsolatedLengthDelta(true, this->numPanJoints, this->numTiltJoints, this->panJointAngle, this->tiltJointAngle);
}

void ManipulatorModule::SetTotalTiltAngle(double angle)
{
    this->totalTiltAngle = angle;
    this->tiltJointAngle = this->totalTiltAngle / this->numTiltJoints;

    this->tiltCentralSeparation = this->CalculateCentralSeparation(this->tiltJointAngle);

    double halfJointAngle = this->tiltJointAngle / 2;
    double cosJointAngle = cos(this->tiltJointAngle);
    double sinJointAngle = sin(this->tiltJointAngle);

    this->tiltJointTransform << cosJointAngle, 0, sinJointAngle, this->tiltCentralSeparation * sin(halfJointAngle) + this->jointSeparationDistance * sinJointAngle,
                 0, 1, 0, 0,
                 -sinJointAngle, 0, cosJointAngle, this->tiltCentralSeparation * cos(halfJointAngle) + this->jointSeparationDistance * cosJointAngle,
                 0, 0, 0, 1;

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

const int& ManipulatorModule::GetNumJoints() const
{
    return this->numJoints;
}

const int& ManipulatorModule::GetNumPanJoints() const
{
    return this->numPanJoints;
}

const int& ManipulatorModule::GetNumTiltJoints() const
{
    return this->numTiltJoints;
}

const bool& ManipulatorModule::IsFirstJointPan() const
{
    return this->isFirstJointPan;
}

const double& ManipulatorModule::GetPanJointAngle() const
{
    return this->panJointAngle;
}

const double& ManipulatorModule::GetTiltJointAngle() const
{
    return this->tiltJointAngle;
}

const double& ManipulatorModule::GetHalfCurvatureAngle() const
{
    return this->halfCurvatureAngle;
}

const double& ManipulatorModule::GetCurvatureRadius() const
{
    return this->curvatureRadius;
}

const double& ManipulatorModule::GetPanCentralSeparation() const
{
    return this->panCentralSeparation;
}

const double& ManipulatorModule::GetTiltCentralSeparation() const
{
    return this->tiltCentralSeparation;
}

const double& ManipulatorModule::GetJointSeparationDistance() const
{
    return this->jointSeparationDistance;
}

const Eigen::Matrix4d& ManipulatorModule::GetPanJointTransform() const
{
    return this->panJointTransform;
}

const Eigen::Matrix4d& ManipulatorModule::GetTiltJointTransform() const
{
    return this->tiltJointTransform;
}

const double& ManipulatorModule::GetIsolatedPanLengthDelta() const
{
    return this->isolatedPanLengthDelta;
}

const double& ManipulatorModule::GetIsolatedTiltLengthDelta() const
{
    return this->isolatedTiltLengthDelta;
}
