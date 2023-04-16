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

// Centre-to-centre distance between tendon pairs in the cross-section of a sub-module, in metres
const double TENDON_PAIR_SEPARATION = 0.003;

// Half angle of curvature of a proximal sub-module, in radians
const double PROXIMAL_HALF_CURVATURE_ANGLE = 0.2;

// Number of pan joints in the proximal module
const int PROXIMAL_NUM_PAN_JOINTS = 2;

// Number of tilt joints in the proximal module
const int PROXIMAL_NUM_TILT_JOINTS = 1;

// Separation between rolling surfaces in a proximal sub-module, in radians
const double PROXIMAL_JOINT_SEPARATION = 0.001;

// Length of a proximal sub-module, in metres
const double PROXIMAL_LENGTH = 0.0014;

// The radius of curvature ('r' in the original model) of a rolling surface in the proximal module, in metres
const double PROXIMAL_CURVATURE_RADIUS = MANIPULATOR_DIAMETER / (2 * sin(PROXIMAL_HALF_CURVATURE_ANGLE));

// Half angle of curvature of a proximal sub-module, in radians
const double DISTAL_HALF_CURVATURE_ANGLE = 0.88;

// Number of pan joints in the distal module
const int DISTAL_NUM_PAN_JOINTS = 2;

// Number of tilt joints in the distal module
const int DISTAL_NUM_TILT_JOINTS = 1;

// Separation between rolling surfaces in a distal sub-module, in radians
const double DISTAL_JOINT_SEPARATION = 0.001;

// Length of a distal sub-module, in metres
const double DISTAL_LENGTH = 0.00288;

// The radius of curvature ('r' in the original model) of a rolling surface in the distal module, in metres
const double DISTAL_CURVATURE_RADIUS = MANIPULATOR_DIAMETER / (2 * sin(DISTAL_HALF_CURVATURE_ANGLE));

// Whether the first joint in the manipulator is a pan joint
const bool IS_FIRST_JOINT_PAN = true;

// The damping factor, lambda, used in the damped least squares method of Jacobian matrix inversion
const double LEAST_SQUARES_DAMPING_FACTOR = 1;

Eigen::Matrix4d zRotationTransform(double theta)
{
    Eigen::Matrix4d transform(
        cos(theta), -sin(theta), 0, 0,
        sin(theta), cos(theta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    );

    return transform;
}

Eigen::Vector3d GetTransformPosition(Eigen::Matrix4d transform)
{
    Eigen::Vector3d position { transform(0, 3), transform(1, 3), transform(2, 3) };
    return position;
}

Eigen::MatrixXd GetJacobian(Eigen::Matrix4d baseTransform, std::vector<ManipulatorModule> modules)
{
    Eigen::Matrix4d moduleInterfaceAngularOffset = zRotationTransform(-M_PI_4);

    Eigen::Matrix4d endTransform = baseTransform;

    const Eigen::Vector3d xAxis(1, 0, 0);
    const Eigen::Vector3d yAxis(0, 1, 0);

    Eigen::Matrix4d panJointTransform;
    Eigen::Matrix4d tiltJointTransform;

    std::vector<std::vector<Eigen::Vector3d>> jointPositionsByModule;

    for (ManipulatorModule module : modules)
    {
        panJointTransform = module.GetPanJointTransform();
        tiltJointTransform = module.GetTiltJointTransform();

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

    // The final Jacobian will have 6 rows and 2 * {num. modules} columns
    Eigen::MatrixXd jacobian(6, 2 * modules.size());

    Eigen::VectorXd jointJacobianColumn(6);
    Eigen::Vector3d jointCrossProduct;

    for (int i = 0; i < modules.size(); i++)
    {
        Eigen::VectorXd modulePanJacobianColumn(6);
        Eigen::VectorXd moduleTiltJacobianColumn(6);

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (i % 2 != 0))
            {
                jointCrossProduct = xAxis.cross(endPosition - jointPositionsByModule[i][j]);

                jointJacobianColumn =
                {
                    jointCrossProduct(0),
                    jointCrossProduct(1),
                    jointCrossProduct(2),
                    xAxis(0),
                    xAxis(1),
                    xAxis(2)
                };

                modulePanJacobianColumn += jointJacobianColumn;
            }
            else
            {
                jointCrossProduct = yAxis.cross(endPosition - jointPositionsByModule[i][j]);

                jointJacobianColumn =
                {
                    jointCrossProduct(0),
                    jointCrossProduct(1),
                    jointCrossProduct(2),
                    yAxis(0),
                    yAxis(1),
                    yAxis(2)
                };

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

// Retrieve an approximation of the inverse Jacobian using the damped least squares method
Eigen::VectorXd GetInverseJacobian(Eigen::MatrixXd jacobian, double leastSquaresDampingFactor)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    Eigen::VectorXd singularValues = svd.singularValues();

    Eigen::MatrixXd inverseJacobian(jacobian.cols(), jacobian.cols());

    for (int i = 0; i < singularValues.size(); i++)
    {
        inverseJacobian.col(i) = singularValues(i) / (pow(singularValues(i), 2) + pow(leastSquaresDampingFactor, 2)) * V.col(i) * U.col(i).transpose();
    }

    return inverseJacobian;
}
