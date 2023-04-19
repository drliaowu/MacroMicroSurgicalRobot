#ifndef MANIPULATOR_MODULE
#define MANIPULATOR_MODULE
#include <Eigen/Dense>

class ManipulatorModule
{
public:
    ManipulatorModule(bool isFirstJointPan, double diameter, double halfCurvatureAngle, int numJoints, double jointSeparationDistance);
    void SetTotalPanAngle(double angle);
    void SetTotalTiltAngle(double angle);
    void ApplyPanAngleDelta(double delta);
    void ApplyTiltAngleDelta(double delta);
    int GetNumJoints();
    int GetNumPanJoints();
    int GetNumTiltJoints();
    bool IsFirstJointPan();
    double GetPanJointAngle();
    double GetTiltJointAngle();
    double GetHalfCurvatureAngle();
    double GetCurvatureRadius();
    double GetPanCentralSeparation();
    double GetTiltCentralSeparation();
    double GetJointSeparationDistance();
    double GetIsolatedPanLengthDelta();
    double GetIsolatedTiltLengthDelta();
    Eigen::Matrix4d GetPanJointTransform();
    Eigen::Matrix4d GetTiltJointTransform();
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
