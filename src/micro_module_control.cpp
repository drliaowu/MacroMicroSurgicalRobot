#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include "omni_msgs/OmniState.h"
#include "omni_msgs/OmniButtonEvent.h"
#include "sensor_msgs/JointState.h"
#include "actionlib/client/simple_action_client.h"
#include "cartesian_control_msgs/FollowCartesianTrajectoryAction.h"
#include "cartesian_control_msgs/FollowCartesianTrajectoryGoal.h"
#include "cartesian_control_msgs/CartesianTrajectoryPoint.h"
#include "boost/bind.hpp"
#include "boost/thread.hpp"
#include <memory>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosbag/bag.h>
#include "../include/Pose.hpp"
#include "../include/MotorGroupState.hpp"
#include "../include/ManipulatorModule.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>

// Diameter of the manipulator, in metres
const double MANIPULATOR_DIAMETER = 0.004;

// Radius of the actuator pulley wheels, in metres
const double PULLEY_WHEEL_RADIUS = 0.0225;

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

Eigen::MatrixXd GetJacobian(Eigen::Matrix4d baseTransform, std::vector<ManipulatorModule> modules)
{
    Eigen::Matrix4d moduleInterfaceAngularOffset = zRotationTransform(-M_PI_4);

    Eigen::Matrix4d endTransform = baseTransform;

    Eigen::Vector3d xAxis;
    xAxis << 1, 0, 0;

    Eigen::Vector3d yAxis;
    yAxis << 0, 1, 0;

    Eigen::Matrix4d panJointTransform;
    Eigen::Matrix4d tiltJointTransform;

    std::vector<std::vector<Eigen::Vector3d>> jointPositionsByModule;

    for (ManipulatorModule module : modules)
    {
        panJointTransform = module.GetPanJointTransform();
        tiltJointTransform = module.GetTiltJointTransform();

        PrintMatrix(panJointTransform, "Pan Joint Transform");
        PrintMatrix(tiltJointTransform, "Tilt Joint Transform");

        std::vector<Eigen::Vector3d> moduleJointPositions;

        for (int i = 0; i < module.GetNumJoints(); i++)
        {
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (module.IsFirstJointPan() ^ (i % 2 != 0))
            {
                endTransform *= panJointTransform;
            }
            else
            {
                endTransform *= tiltJointTransform;
            }

            Eigen::Vector3d jointPosition = GetTransformPosition(endTransform);

            moduleJointPositions.push_back(jointPosition);
        }

        jointPositionsByModule.push_back(moduleJointPositions);

        // Modules are offset by a 45-degree rotation at their interface
        endTransform *= moduleInterfaceAngularOffset;
    }

    Eigen::Vector3d endPosition = GetTransformPosition(endTransform);

    PrintMatrix(endPosition, "End Position");

    // The final Jacobian will have 6 rows and 2 * {num. modules} columns
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 2 * modules.size());

    Eigen::VectorXd jointJacobianColumn(6);
    Eigen::Vector3d jointCrossProduct;

    Eigen::VectorXd modulePanJacobianColumn;
    Eigen::VectorXd moduleTiltJacobianColumn;

    for (int i = 0; i < modules.size(); i++)
    {
        modulePanJacobianColumn = Eigen::VectorXd::Zero(6);
        moduleTiltJacobianColumn = Eigen::VectorXd::Zero(6);

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (i % 2 != 0))
            {
                jointCrossProduct = xAxis.cross(endPosition - jointPositionsByModule[i][j]);

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
                jointCrossProduct = yAxis.cross(endPosition - jointPositionsByModule[i][j]);

                jointJacobianColumn << jointCrossProduct(0),
                                       jointCrossProduct(1),
                                       jointCrossProduct(2),
                                       yAxis(0),
                                       yAxis(1),
                                       yAxis(2);

                moduleTiltJacobianColumn += jointJacobianColumn;
            }
        }

        modulePanJacobianColumn /= modules[i].GetNumPanJoints();
        moduleTiltJacobianColumn /= modules[i].GetNumTiltJoints();

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

MotorGroupState GetMotorPositionsFromJointPositions(ManipulatorModule proximal, ManipulatorModule distal)
{
    MotorGroupState state;

    double proximalPanLengthDelta = proximal.GetIsolatedPanLengthDelta(true);
    double proximalTiltLengthDelta = proximal.GetIsolatedTiltLengthDelta(true);

    state.ProximalPanAngle = proximalPanLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    state.ProximalTiltAngle = proximalTiltLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    double proximalCurvatureRadius = proximal.GetCurvatureRadius();
    double proximalHalfCurvatureAngle = proximal.GetHalfCurvatureAngle();

    // The difference in tendon length change resulting from the angular offset of the distal tendons on the proximal joints
    double distalTendonProximalJointOffset = (2 - sqrt(2)) * proximal.GetCurvatureRadius() * sin(proximal.GetHalfCurvatureAngle()) * (
        proximal.GetNumPanJoints() * sin(proximal.GetPanJointAngle() / 2) +
        proximal.GetNumTiltJoints() * sin(proximal.GetTiltJointAngle() / 2)
    );

    // The contribution of the proximal joint positions to the length delta of the distal tendons
    double distalLengthDeltaDueToProximalJoints = proximalPanLengthDelta + proximalTiltLengthDelta + distalTendonProximalJointOffset;

    state.DistalPanAngle = (distal.GetIsolatedPanLengthDelta(true) + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    state.DistalTiltAngle = (distal.GetIsolatedTiltLengthDelta(true) + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    return state;
}

int main(int argc, char **argv)
{
    ManipulatorModule proximal(true, MANIPULATOR_DIAMETER, PROXIMAL_HALF_CURVATURE_ANGLE, PROXIMAL_NUM_JOINTS, PROXIMAL_JOINT_SEPARATION);
    ManipulatorModule distal(true, MANIPULATOR_DIAMETER, DISTAL_HALF_CURVATURE_ANGLE, DISTAL_NUM_JOINTS, DISTAL_JOINT_SEPARATION);

    std::vector<ManipulatorModule> modules { proximal, distal };

    proximal.SetTotalPanAngle(0);
    proximal.SetTotalTiltAngle(0);
    distal.SetTotalPanAngle(0);
    distal.SetTotalTiltAngle(0);

    Eigen::Matrix4d baseTransform;
    baseTransform << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;

    Eigen::MatrixXd jacobian = GetJacobian(baseTransform, modules);

    PrintMatrix(jacobian, "Jacobian");

    Eigen::MatrixXd inverseJacobian = GetInverseJacobian(jacobian, LEAST_SQUARES_DAMPING_FACTOR);

    PrintMatrix(inverseJacobian, "Inverse Jacobian");

    MotorGroupState motorStates { 90, 90, 90, 90 };

    Eigen::VectorXd desiredPose(6);

    desiredPose << 0.005, 0.005, 0.005, 0.5, 0.5, 0.5;

    Eigen::Vector4d jointDeltas = inverseJacobian * desiredPose;

    PrintMatrix(jointDeltas, "Final joint deltas");

    MotorGroupState newState = GetMotorPositionsFromJointPositions(proximal, distal);

    return 0;
}
