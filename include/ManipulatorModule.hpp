#ifndef MANIPULATOR_MODULE
#include <Eigen/Dense>
#endif

class ManipulatorModule
{
public:
    ManipulatorModule(bool isFirstJointPan, double diameter, double halfCurvatureAngle, int numJoints, double jointSeparationDistance);
    void SetTotalPanAngle(double angle);
    void SetTotalTiltAngle(double angle);
    int GetNumJoints();
    bool IsFirstJointPan();
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

    double GetCentralSeparation(double jointAngle);
};
