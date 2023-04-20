#ifndef MANIPULATOR_MODULE
#define MANIPULATOR_MODULE
#include <Eigen/Dense>

class ManipulatorModule
{
public:
    // This macro must be added to all classes with Eigen matrix members to ensure proper memory alignment
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ManipulatorModule(bool isFirstJointPan, double diameter, double halfCurvatureAngle, int numJoints, double jointSeparationDistance);
    void SetTotalPanAngle(double angle);
    void SetTotalTiltAngle(double angle);
    void ApplyPanAngleDelta(double delta);
    void ApplyTiltAngleDelta(double delta);
    const int& GetNumJoints() const;
    const int& GetNumPanJoints() const;
    const int& GetNumTiltJoints() const;
    const bool& IsFirstJointPan() const;
    const double& GetPanJointAngle() const;
    const double& GetTiltJointAngle() const;
    const double& GetHalfCurvatureAngle() const;
    const double& GetCurvatureRadius() const;
    const double& GetPanCentralSeparation() const;
    const double& GetTiltCentralSeparation() const;
    const double& GetJointSeparationDistance() const;
    const double& GetIsolatedPanLengthDelta() const;
    const double& GetIsolatedTiltLengthDelta() const;
    const Eigen::Matrix4d& GetPanJointTransform() const;
    const Eigen::Matrix4d& GetTiltJointTransform() const;
private:
    bool isFirstJointPan;
    double diameter;
    double halfCurvatureAngle;
    double curvatureRadius;
    int numJoints;
    int numPanJoints;
    int numTiltJoints;
    double jointSeparationDistance;
    double totalPanAngle;
    double totalTiltAngle;
    double panJointAngle;
    double tiltJointAngle;
    double panCentralSeparation;
    double tiltCentralSeparation;
    Eigen::Matrix4d panJointTransform;
    Eigen::Matrix4d tiltJointTransform;
    double isolatedPanLengthDelta;
    double isolatedTiltLengthDelta;

    double CalculateCentralSeparation(double jointAngle);
    Eigen::Matrix4d CalculatePanJointTransform();
    Eigen::Matrix4d CalculateTiltJointTransform();
    double CalculateIsolatedLengthDelta(bool isLeftTendon, int numPrimaryJoints, int numSecondaryJoints, double primaryJointAngle, double secondaryJointAngle);
};
#endif
