#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "omni_msgs/OmniState.h"
#include "omni_msgs/OmniButtonEvent.h"
#include <memory>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosbag/bag.h>
#include "../include/ManipulatorModule.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>

// Diameter of the manipulator, in metres
const double MANIPULATOR_DIAMETER = 0.004;

// Radius of the actuator pulley wheels, in metres
const double PULLEY_WHEEL_RADIUS = 0.01125;

// Conversion factor between a tendon length delta in metres and a pulley wheel rotation in degrees
const double LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES = 180 / M_PI / PULLEY_WHEEL_RADIUS;

// Centre-to-centre distance between tendon pairs in the cross-section of a sub-module, in metres
const double TENDON_PAIR_SEPARATION = 0.003;

// Half angle of curvature of a proximal sub-module, in radians
const double PROXIMAL_HALF_CURVATURE_ANGLE = 0.2;

// Number of joints in the proximal module
const int PROXIMAL_NUM_JOINTS = 3;

// Separation between rolling surfaces in a proximal sub-module, in radians
const double PROXIMAL_JOINT_SEPARATION = 0.001;

// Length of a proximal sub-module, in metres
const double PROXIMAL_LENGTH = 0.0014;

// Half angle of curvature of a proximal sub-module, in radians
const double DISTAL_HALF_CURVATURE_ANGLE = 0.88;

// Number of joints in the distal module
const int DISTAL_NUM_JOINTS = 3;

// Separation between rolling surfaces in a distal sub-module, in radians
const double DISTAL_JOINT_SEPARATION = 0.001;

// Length of a distal sub-module, in metres
const double DISTAL_LENGTH = 0.00288;

// The damping factor, lambda, used in the damped least squares method of Jacobian matrix inversion
const double LEAST_SQUARES_DAMPING_FACTOR = 1;

void PrintMatrix(const Eigen::MatrixXd matrix, const char* title)
{
    std::stringstream output;

    output << matrix;

    ROS_INFO("%s:\n%s", title, output.str().c_str());
}

Eigen::Matrix3d xRotationMatrix(double theta)
{
    Eigen::Matrix3d rotation;

    rotation << 1, 0, 0,
                0, cos(theta), -sin(theta),
                0, sin(theta), cos(theta);

    return rotation;
}

Eigen::Matrix3d yRotationMatrix(double theta)
{
    Eigen::Matrix3d rotation;

    rotation << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

    return rotation;
}

Eigen::Matrix4d zRotationTransform(double theta)
{
    Eigen::Matrix4d transform;

    transform << cos(theta), -sin(theta), 0.0, 0.0,
                 sin(theta), cos(theta), 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

    return transform;
}

Eigen::Matrix4d zRotationTransform(double theta)
{
    Eigen::Matrix4d transform;

    transform << cos(theta), -sin(theta), 0.0, 0.0,
                 sin(theta), cos(theta), 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

    return transform;
}

// Extract the position vector from a homogeneous transform
Eigen::Vector3d GetTransformPosition(Eigen::Matrix4d transform)
{
    Eigen::Vector3d position;
    position << transform(0, 3), transform(1, 3), transform(2, 3);
    return position;
}

std::vector<Eigen::Vector3d> GetJointPositions(Eigen::Matrix4d baseTransform, std::vector<ManipulatorModule> modules)
{
    Eigen::Matrix4d moduleInterfaceAngularOffset = zRotationTransform(-M_PI_4);

    Eigen::Matrix4d endTransform = baseTransform;

    Eigen::Matrix4d panJointTransform = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d tiltJointTransform = Eigen::Matrix4d::Zero();

    std::vector<Eigen::Vector3d> jointPositions;

    for (int i = 0; i < modules.size(); i++)
    {
        panJointTransform = modules[i].GetPanJointTransform().eval();
        tiltJointTransform = modules[i].GetTiltJointTransform().eval();

        PrintMatrix(panJointTransform, "Pan Joint Transform");
        PrintMatrix(tiltJointTransform, "Tilt Joint Transform");

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (j % 2 != 0))
            {
                endTransform *= panJointTransform;
            }
            else
            {
                endTransform *= tiltJointTransform;
            }

            Eigen::Vector3d jointPosition = GetTransformPosition(endTransform);

            jointPositions.push_back(jointPosition);
        }

        // Modules are offset by a 45-degree rotation at their interface
        if (i != modules.size() - 1)
        {
            endTransform *= moduleInterfaceAngularOffset;
        }
    }

    Eigen::Vector3d endPosition = GetTransformPosition(endTransform);

    jointPositions.push_back(endPosition);

    PrintMatrix(endPosition, "End Position");

    return jointPositions;
}

Eigen::MatrixXd GetJacobian(std::vector<Eigen::Vector3d> jointPositions, std::vector<ManipulatorModule> modules)
{
    Eigen::Vector3d xAxis;
    xAxis << 1, 0, 0;

    Eigen::Vector3d yAxis;
    yAxis << 0, 1, 0;

    // The final Jacobian will have 6 rows and 2 * {num. modules} columns
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 2 * modules.size());

    Eigen::VectorXd jointJacobianColumn(6);
    Eigen::Vector3d jointCrossProduct;

    Eigen::VectorXd modulePanJacobianColumn;
    Eigen::VectorXd moduleTiltJacobianColumn;

    Eigen::Vector3d endPosition = jointPositions.back();

    for (int i = 0; i < modules.size(); i++)
    {
        modulePanJacobianColumn = Eigen::VectorXd::Zero(6);
        moduleTiltJacobianColumn = Eigen::VectorXd::Zero(6);

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            PrintMatrix(jointPositions[i+j], "Joint Position");
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (j % 2 != 0))
            {
                jointCrossProduct = xAxis.cross(endPosition - jointPositions[i + j]);

                PrintMatrix(jointCrossProduct, "Joint cross product");

                jointJacobianColumn << jointCrossProduct(0),
                                       jointCrossProduct(1),
                                       jointCrossProduct(2),
                                       xAxis(0),
                                       xAxis(1),
                                       xAxis(2);

                modulePanJacobianColumn += jointJacobianColumn;
            }
            else
            {
                jointCrossProduct = yAxis.cross(endPosition - jointPositions[i + j]);

                PrintMatrix(jointCrossProduct, "Joint cross product");

                jointJacobianColumn << jointCrossProduct(0),
                                       jointCrossProduct(1),
                                       jointCrossProduct(2),
                                       yAxis(0),
                                       yAxis(1),
                                       yAxis(2);

                moduleTiltJacobianColumn += jointJacobianColumn;
            }
        }

        PrintMatrix(modulePanJacobianColumn, "Pan Jacobian column pre-division");

        modulePanJacobianColumn /= modules[i].GetNumPanJoints();
        moduleTiltJacobianColumn /= modules[i].GetNumTiltJoints();

        PrintMatrix(modulePanJacobianColumn, "Pan Jacobian column post-division");

        if (modules[i].IsFirstJointPan())
        {
            jacobian.col(2 * i) = modulePanJacobianColumn;
            jacobian.col(2 * i + 1) = moduleTiltJacobianColumn;
        }
        else
        {
            jacobian.col(2 * i) = moduleTiltJacobianColumn;
            jacobian.col(2 * i + 1) = modulePanJacobianColumn;
        }
    }

    return jacobian;
}

// Obtain an approximation of the inverse Jacobian using the damped least squares method
Eigen::MatrixXd GetInverseJacobian(Eigen::MatrixXd jacobian, double leastSquaresDampingFactor)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    Eigen::VectorXd singularValues = svd.singularValues();

    Eigen::MatrixXd inverseJacobian = Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());

    for (int i = 0; i < svd.nonzeroSingularValues(); i++)
    {
        inverseJacobian += singularValues(i) / (pow(singularValues(i), 2) + pow(leastSquaresDampingFactor, 2)) * V.col(i) * U.col(i).transpose();
    }

    return inverseJacobian;
}

Eigen::Vector4d GetMotorPositionsFromJointPositions(ManipulatorModule proximal, ManipulatorModule distal)
{
    Eigen::Vector4d motorPositions = Eigen::Vector4d::Zero();

    double test = 2 * proximal.GetCurvatureRadius() * (
        proximal.GetNumPanJoints() * (cos(proximal.GetHalfCurvatureAngle()) - cos(proximal.GetHalfCurvatureAngle() + (-1 * (int)true) * proximal.GetPanJointAngle() / 2)) +
        proximal.GetNumTiltJoints() * (1 - cos(proximal.GetTiltJointAngle() / 2))
    );

    ROS_INFO("test: %.6f", test);

    double proximalPanLengthDelta = proximal.GetIsolatedPanLengthDelta();
    double proximalTiltLengthDelta = proximal.GetIsolatedTiltLengthDelta();

    ROS_INFO("prox. pan: %.6f, prox. tilt: %.6f", proximalPanLengthDelta, proximalTiltLengthDelta);

    motorPositions(0) = proximalPanLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    motorPositions(1) = proximalTiltLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    double proximalCurvatureRadius = proximal.GetCurvatureRadius();
    double proximalHalfCurvatureAngle = proximal.GetHalfCurvatureAngle();

    // The difference in tendon length change resulting from the angular offset of the distal tendons on the proximal joints
    double distalTendonProximalJointOffset = (2 - sqrt(2)) * proximal.GetCurvatureRadius() * sin(proximal.GetHalfCurvatureAngle()) * (
        proximal.GetNumPanJoints() * sin(proximal.GetPanJointAngle() / 2) +
        proximal.GetNumTiltJoints() * sin(proximal.GetTiltJointAngle() / 2)
    );

    // The contribution of the proximal joint positions to the length delta of the distal tendons
    double distalLengthDeltaDueToProximalJoints = proximalPanLengthDelta + proximalTiltLengthDelta + distalTendonProximalJointOffset;

    ROS_INFO("dLengthDelta: %.6f", distalLengthDeltaDueToProximalJoints);

    motorPositions(2) = (distal.GetIsolatedPanLengthDelta() + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    motorPositions(3) = (distal.GetIsolatedTiltLengthDelta() + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    ROS_INFO("dist. pan: %.6f, dist. tilt: %.6f", motorPositions(0), motorPositions(1));

    return motorPositions;
}

Eigen::Vector4d GetJointPositionsFromMotorPositions(Eigen::Vector4d motorStates, ManipulatorModule proximal, ManipulatorModule distal)
{
    Eigen::Vector4d jointPositions = Eigen::Vector4d::Zero();

    // Ratio of proximal curvature radius to pulley radius
    double proximalRadiusRatio = proximal.GetCurvatureRadius() / PULLEY_WHEEL_RADIUS;
    double proximalHalfPanJointAngle = proximal.GetPanJointAngle() / 2;
    double proximalHalfTiltJointAngle = proximal.GetTiltJointAngle() / 2;

    // Construct proximal actuator Jacobian
    Eigen::Matrix2d proximalJacobian;
    proximalJacobian << -proximalRadiusRatio * sin(proximal.GetHalfCurvatureAngle() - proximalHalfPanJointAngle), proximalRadiusRatio * sin(proximalHalfTiltJointAngle),
                         proximalRadiusRatio * sin(proximalHalfPanJointAngle), -proximalRadiusRatio * sin(proximal.GetHalfCurvatureAngle() - proximalHalfTiltJointAngle);

    PrintMatrix(proximalJacobian, "Proximal Actuator Jacobian");

    double distalRadiusRatio = distal.GetCurvatureRadius() / PULLEY_WHEEL_RADIUS;
    double distalHalfPanJointAngle = distal.GetPanJointAngle() / 2;
    double distalHalfTiltJointAngle = distal.GetTiltJointAngle() / 2;

    // Constant coefficient of partial derivatives of distal tendon lengths w.r.t. proximal joint angles
    double derivativeCoefficient = 1 - 1 / sqrt(2);

    // Construct distal actuator Jacobian
    Eigen::MatrixXd distalJacobian(2, 4);
    distalJacobian <<  proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfPanJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfTiltJointAngle),
                       -distalRadiusRatio * sin(distal.GetHalfCurvatureAngle() - distalHalfPanJointAngle),
                       distalRadiusRatio * sin(distalHalfTiltJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfPanJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfTiltJointAngle),
                       distalRadiusRatio * sin(distalHalfPanJointAngle),
                       -distalRadiusRatio * sin(distal.GetHalfCurvatureAngle() - distalHalfTiltJointAngle);

    PrintMatrix(distalJacobian, "Distal Actuator Jacobian");

    // Combine proximal and distal Jacobians. Note that the distal motor angles have no effect on the proximal joint positions
    Eigen::Matrix4d combinedJacobian = Eigen::Matrix4d::Zero();
    combinedJacobian.block(0, 0, 2, 2) = proximalJacobian;
    combinedJacobian.block(2, 0, 2, 4) = distalJacobian;

    jointPositions = combinedJacobian * motorStates.inverse();

    return jointPositions;
}

Eigen::Vector3d SkewSymmetricMatrixToVector(Eigen::Matrix3d matrix)
{
    // Creates a vector v from a skew-symmetric matrix, assuming the following matrix layout:
    // |   0 -vz  vy |
    // |  vz   0 -vx |
    // | -vy  vx   0 |
    Eigen::Vector3d vector;
    vector << -matrix(1, 2), matrix(0, 2), -matrix(0, 1);
    return vector;
}

int main(int argc, char **argv)
{
    ManipulatorModule proximal(
        true,
        MANIPULATOR_DIAMETER,
        PROXIMAL_HALF_CURVATURE_ANGLE,
        PROXIMAL_NUM_JOINTS,
        PROXIMAL_JOINT_SEPARATION
    );

    ManipulatorModule distal(
        true,
        MANIPULATOR_DIAMETER,
        DISTAL_HALF_CURVATURE_ANGLE,
        DISTAL_NUM_JOINTS,
        DISTAL_JOINT_SEPARATION
    );

    std::vector<ManipulatorModule> modules { proximal, distal };

    proximal.SetTotalPanAngle(0.0);
    proximal.SetTotalTiltAngle(0.0);
    distal.SetTotalPanAngle(0.0);
    distal.SetTotalTiltAngle(0.0);

    ROS_INFO("Proximal parameters:\n\tcurvature radius: %.6f\n\tnum pan joints: %d\n\thalf curvature angle: %.6f\n\tpan joint angle: %.6f\n\tnum tilt joints: %d\n\ttilt joint angle: %.6f\n\tpan central separation: %.6f\n\ttilt central separation: %.6f\n\tjoint separation distance: %.6f\n\tisolated pan length delta: %.6f\n\tisolated tilt length delta: %.6f",
        proximal.GetCurvatureRadius(),
        proximal.GetNumPanJoints(),
        proximal.GetHalfCurvatureAngle(),
        proximal.GetPanJointAngle(),
        proximal.GetNumTiltJoints(),
        proximal.GetTiltJointAngle(),
        proximal.GetPanCentralSeparation(),
        proximal.GetTiltCentralSeparation(),
        proximal.GetJointSeparationDistance(),
        proximal.GetIsolatedPanLengthDelta(),
        proximal.GetIsolatedTiltLengthDelta()
    );

    ROS_INFO("Proximal parameters:\n\tcurvature radius: %.6f\n\tnum pan joints: %d\n\thalf curvature angle: %.6f\n\tpan joint angle: %.6f\n\tnum tilt joints: %d\n\ttilt joint angle: %.6f\n\tpan central separation: %.6f\n\ttilt central separation: %.6f\n\tjoint separation distance: %.6f\n\tisolated pan length delta: %.6f\n\tisolated tilt length delta: %.6f",
        distal.GetCurvatureRadius(),
        distal.GetNumPanJoints(),
        distal.GetHalfCurvatureAngle(),
        distal.GetPanJointAngle(),
        distal.GetNumTiltJoints(),
        distal.GetTiltJointAngle(),
        distal.GetPanCentralSeparation(),
        distal.GetTiltCentralSeparation(),
        distal.GetJointSeparationDistance(),
        proximal.GetIsolatedPanLengthDelta(),
        proximal.GetIsolatedTiltLengthDelta()
    );

    Eigen::Matrix4d panJointTransform = Eigen::Matrix4d::Zero();

    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();

    std::vector<Eigen::Vector3d> jointPositions = GetJointPositions(baseTransform, modules);

    Eigen::Vector3d endPosition = jointPositions.back();

    Eigen::MatrixXd jacobian = GetJacobian(jointPositions, modules);
    Eigen::MatrixXd truncatedJacobian = jacobian.block(0, 0, 3, jacobian.cols());

    PrintMatrix(jacobian, "Jacobian");

    Eigen::MatrixXd inverseJacobian = GetInverseJacobian(truncatedJacobian, LEAST_SQUARES_DAMPING_FACTOR);

    PrintMatrix(inverseJacobian, "Inverse Jacobian");

    Eigen::Vector4d motorStates;
    motorStates << 90, 90, 90, 90;

    Eigen::Vector3d desiredPos;
    desiredPos << 0.005, 0.005, 0.0128534;

    PrintMatrix(desiredPos, "Desired Pos");

    // Eigen::MatrixXd truncatedInverseJacobian(inverseJacobian.rows(), 3);
    // truncatedInverseJacobian << inverseJacobian.block(0, 0, inverseJacobian.rows(), 3);

    // PrintMatrix(truncatedInverseJacobian, "Truncated inverse Jacobian");

    Eigen::Vector4d jointDeltas = inverseJacobian * (desiredPos - endPosition);

    PrintMatrix(jointDeltas, "Final joint deltas");

    proximal.ApplyPanAngleDelta(jointDeltas(0));
    proximal.ApplyTiltAngleDelta(jointDeltas(1));

    distal.ApplyPanAngleDelta(jointDeltas(2));
    distal.ApplyTiltAngleDelta(jointDeltas(3));

    Eigen::Vector4d stateDelta = GetMotorPositionsFromJointPositions(proximal, distal);

    motorStates += stateDelta;

    ROS_INFO("%.6f %.6f %.6f %.6f", motorStates(0), motorStates(1), motorStates(2), motorStates(3));

    return 0;
}
