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

double GetCentralSeparation(double curvatureRadius, double halfCurvatureAngle, double halfJointAngle)
{
    // The value 'h' from the original model
    return 2 * curvatureRadius * (1 - cos(halfCurvatureAngle) * cos(halfJointAngle));
}

Eigen::Matrix4d GetPanJointTransform(double jointAngle, double jointSeparation, double halfCurvatureAngle, double curvatureRadius)
{
    double halfJointAngle = jointAngle / 2;

    double cosJointAngle = cos(jointAngle);
    double sinJointAngle = sin(jointAngle);

    double centralSeparation = GetCentralSeparation(curvatureRadius, halfCurvatureAngle, halfJointAngle);

    Eigen::Matrix4d transform(
        1, 0, 0, 0,
        0, cosJointAngle, -sinJointAngle, -centralSeparation * sin(halfJointAngle) - jointSeparation * sinJointAngle,
        0, sinJointAngle, cosJointAngle, centralSeparation * cos(halfJointAngle) + jointSeparation * cosJointAngle,
        0, 0, 0, 1
    );

    return transform;
}

Eigen::Matrix4d GetTiltJointTransform(double jointAngle, double jointSeparation, double halfCurvatureAngle, double curvatureRadius)
{
    double halfJointAngle = jointAngle / 2;

    double cosJointAngle = cos(jointAngle);
    double sinJointAngle = sin(jointAngle);

    double centralSeparation = GetCentralSeparation(curvatureRadius, halfCurvatureAngle, halfJointAngle);

    Eigen::Matrix4d transform(
        cosJointAngle, 0, sinJointAngle, centralSeparation * sin(halfJointAngle) + jointSeparation * sinJointAngle,
        0, 1, 0, 0,
        -sinJointAngle, 0, cosJointAngle, centralSeparation * cos(halfJointAngle) + jointSeparation * cosJointAngle,
        0, 0, 0, 1
    );

    return transform;
}

Eigen::Matrix4d GetPoseFromJointPositions(std::vector<ManipulatorModule> modules)
{
    Eigen::Matrix4d moduleInterfaceAngularOffset = zRotationTransform(-M_PI_4);

    Eigen::Matrix4d TEnd;

    for (ManipulatorModule module : modules)
    {
        Eigen::Matrix4d panJointTransform = module.GetPanJointTransform();

        Eigen::Matrix4d tiltJointTransform = module.GetTiltJointTransform();

        for (int i = 0; i < module.GetNumJoints(); i++)
        {
            if (module.IsFirstJointPan() ^ (i % 2 != 0))
            {
                TEnd *= panJointTransform;
            }
            else
            {
                TEnd *= tiltJointTransform;
            }
        }

        // Modules are offset by a 45-degree rotation at their interface
        TEnd *= moduleInterfaceAngularOffset;
    }

    return TEnd;
}

Eigen::MatrixXd GetJacobian(std::vector<Eigen::Vector3d> jointPositions)
{
    Eigen::MatrixXd jacobian(6, jointPositions.size());

    return jacobian;
}

std::vector<Eigen::Vector3d> GetJointPositionsFromPose(Pose targetPose, Eigen::MatrixXd jacobian)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    std::vector<Eigen::Vector3d> jointPositions;

    return jointPositions;
}
