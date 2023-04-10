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

// Touch position coordinates are output in millimetres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Period in seconds of each UR5e pose control operation
const double UR5E_CONTROL_PERIOD = 1e-2;

// Number of subdivisions of a single trajectory to send to the UR5E, increase number to improve motion smoothness
const double UR5E_CONTROL_NUM_SUBDIVISIONS = 10;

const double UR5E_CONTROL_TIME_DELTA = UR5E_CONTROL_PERIOD / UR5E_CONTROL_NUM_SUBDIVISIONS;

const int UR5E_JOINT_COUNT = 6;

// The maximum delay, in seconds, from the last pose update of the robot being controlled
const double MAX_CONTROL_DELAY = 0.1;

const std::string UR5E_CONTROLLER_NAME = "pose_based_cartesian_traj_controller";

typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianActionClient;

enum ControlType
{
    Velocity,
    PoseDelta
};

tf2::Quaternion QuaternionExponential(tf2::Quaternion quaternion)
{
    tf2::Vector3 vectorComponent(quaternion.getX(), quaternion.getY(), quaternion.getZ());

    double vectorMagnitude = vectorComponent.length();
    double expW = exp(quaternion.getW());

    tf2::Vector3 expVectorComponent = expW * vectorComponent.normalized() * sin(vectorMagnitude);

    tf2::Quaternion result(
        expVectorComponent.getX(),
        expVectorComponent.getY(),
        expVectorComponent.getZ(),
        expW * cos(vectorMagnitude)
    );

    return result;
}

tf2::Quaternion QuaternionLogarithm(tf2::Quaternion quaternion)
{
    tf2::Vector3 vectorComponent(quaternion.getX(), quaternion.getY(), quaternion.getZ());

    double magnitude = quaternion.length();

    tf2::Vector3 logVectorComponent = vectorComponent.normalized() * acos(quaternion.getW() / magnitude);

    tf2::Quaternion result(
        logVectorComponent.getX(),
        logVectorComponent.getY(),
        logVectorComponent.getZ(),
        log(magnitude)
    );

    return result;
}

// Function to calculate the average angular velocity from two timestamped poses and the desired time delta in seconds
tf2::Quaternion GetAngularVelocity(
    const geometry_msgs::PoseStamped& prevPose,
    const geometry_msgs::PoseStamped& currentPose,
    const double dt
)
{
    // Time difference between the two pose measurements
    double poseDt = (currentPose.header.stamp - prevPose.header.stamp).toSec();

    // Convert orientations from quaternion messages to tf2 quaternions
    tf2::Quaternion prevOrientation, currentOrientation;
    tf2::fromMsg(prevPose.pose.orientation, prevOrientation);
    tf2::fromMsg(currentPose.pose.orientation, currentOrientation);

    tf2::Quaternion diffQuater = currentOrientation * prevOrientation.inverse();

    tf2::Quaternion prevConjQuaternion;

    prevConjQuaternion.setX(-(diffQuater.x()));
    prevConjQuaternion.setY(-(diffQuater.y()));
    prevConjQuaternion.setZ(-(diffQuater.z()));
    prevConjQuaternion.setW(diffQuater.getW());

    tf2::Quaternion output = QuaternionExponential(QuaternionLogarithm(diffQuater) / dt) * prevConjQuaternion * 2;

    return output;
}

geometry_msgs::Quaternion StylusRotationToRobotFrame(const tf2::Quaternion& stylusToRobotRotation, const geometry_msgs::Quaternion& stylus_quaternion) {
    tf2::Quaternion stylus_tf_quaternion, robot_tf_quaternion;

    // Convert the geometry_msgs::Quaternion to tf2::Quaternion
    tf2::fromMsg(stylus_quaternion, stylus_tf_quaternion);

    // Apply the fixed transformation
    robot_tf_quaternion = stylusToRobotRotation * stylus_tf_quaternion;

    // robot_tf_quaternion.setX(-robot_tf_quaternion.getX());
    // robot_tf_quaternion.setY(-robot_tf_quaternion.getY());
    // robot_tf_quaternion.setZ(-robot_tf_quaternion.getZ());
    // robot_tf_quaternion.setW(-robot_tf_quaternion.getW());

    // Convert the tf2::Quaternion back to geometry_msgs::Quaternion
    geometry_msgs::Quaternion robot_quaternion = tf2::toMsg(robot_tf_quaternion);

    return robot_quaternion;
}

void TransformStampedToPoseStamped(const geometry_msgs::TransformStamped& transformStamped, geometry_msgs::PoseStamped& poseStamped)
{
    poseStamped.header = transformStamped.header;

    poseStamped.pose.position.x = transformStamped.transform.translation.x;
    poseStamped.pose.position.y = transformStamped.transform.translation.y;
    poseStamped.pose.position.z = transformStamped.transform.translation.z;

    poseStamped.pose.orientation = transformStamped.transform.rotation;
}

std::string PoseToString(geometry_msgs::Pose pose, const double position_unit_scale_factor)
{
    std::stringstream output;

    output << "Position: (x: "
        << pose.position.x * position_unit_scale_factor
        << "m, y: "
        << pose.position.y * position_unit_scale_factor
        << "m, z: "
        << pose.position.z * position_unit_scale_factor
        << "m), Orientation: (x: "
        << pose.orientation.x
        << ", y: "
        << pose.orientation.y
        << ", z: "
        << pose.orientation.z
        << ", w: "
        << pose.orientation.w
        << ")";

    return output.str();
}

std::string Vector3ToString(geometry_msgs::Vector3 vector3)
{
    std::stringstream output;

    output << "(x: "
        << vector3.x
        << ", y: "
        << vector3.y
        << ", z: "
        << vector3.z
        << ")";

    return output.str();
}

void touchStateCallback(
    const omni_msgs::OmniState::ConstPtr& omniState,
    geometry_msgs::PoseStamped& poseStamped,
    geometry_msgs::PoseStamped& prevPoseStamped,
    geometry_msgs::Vector3& translationVelocity,
    tf2::Quaternion& angularVelocity
)
{
    prevPoseStamped = geometry_msgs::PoseStamped(poseStamped);
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose = omniState->pose;

    // Scale touch position units to metres
    poseStamped.pose.position.x = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->pose.position.x;
    poseStamped.pose.position.y = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->pose.position.y;
    poseStamped.pose.position.z = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->pose.position.z;

    translationVelocity.x = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->velocity.x;
    translationVelocity.y = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->velocity.y;
    translationVelocity.z = TOUCH_POSITION_UNIT_SCALE_FACTOR * omniState->velocity.z;

    ROS_INFO_THROTTLE(1, "Current Touch Pose: %s, Prev Touch Pose: %s", PoseToString(poseStamped.pose, 1).c_str(), PoseToString(prevPoseStamped.pose, 1).c_str());

    angularVelocity = GetAngularVelocity(prevPoseStamped, poseStamped, UR5E_CONTROL_TIME_DELTA);

    ROS_INFO_THROTTLE(1, "Current Touch Angular Velocity: x: %.3f, y: %.3f, z: %.3f, w: %.3f", angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ(), angularVelocity.getW());

    geometry_msgs::Vector3 current = omniState->current;

    // ROS_INFO_THROTTLE(1, "Touch State:\n\tPose: %s\n\tCurrent: %s\n\tVelocity: %s",
    //     PoseToString(omniState->pose, TOUCH_POSITION_UNIT_SCALE_FACTOR).c_str(),
    //     Vector3ToString(omniState->current).c_str(),
    //     Vector3ToString(omniState->velocity).c_str()
    // );
}

void touchButtonCallback(
    const omni_msgs::OmniButtonEvent::ConstPtr& omniButtonStates,
    bool& isGreyButtonPressed,
    bool& isWhiteButtonPressed
)
{
    isGreyButtonPressed = (bool)omniButtonStates->grey_button;
    isWhiteButtonPressed = (bool)omniButtonStates->white_button;

    // ROS_INFO_THROTTLE(
    //     1,
    //     "Grey button: %d, white button: %d",
    //     isGreyButtonPressed,
    //     isWhiteButtonPressed
    // );
}

bool LoadController(ros::NodeHandle& nodeHandle, const std::string controllerName)
{
    // Create a client for the load_controller service
    ros::ServiceClient loadControllerClient = nodeHandle.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");

    // Create a service request
    controller_manager_msgs::LoadController loadController;
    loadController.request.name = controllerName;

    // Call the service to load the controller
    if (loadControllerClient.call(loadController))
    {
        if (loadController.response.ok)
        {
            ROS_INFO("Successfully loaded controller: %s", controllerName.c_str());
            return true;
        }

        ROS_ERROR("Failed to load controller: %s", controllerName.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call load_controller service");
    }

    return false;
}

bool SwitchController(ros::NodeHandle& nodeHandle, const std::string controllerName)
{
    ros::ServiceClient switchControllerClient = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switchController;
    switchController.request.stop_controllers = ros::V_string
    {
        "scaled_pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "pos_joint_traj_controller",
        "vel_joint_traj_controller",
        "forward_joint_traj_controller",
        "joint_based_cartesian_traj_controller",
        "forward_cartesian_traj_controller",
        "joint_group_vel_controller",
        "twist_controller"
    };
    switchController.request.start_controllers.push_back(controllerName);
    switchController.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

    if (switchControllerClient.call(switchController))
    {
        ROS_INFO("Controller started");
        return true;
    }

    ROS_ERROR("Failed to start controller");
    return false;
}

int main(int argc, char **argv)
{
    geometry_msgs::PoseStamped currentUR5EPose;
    geometry_msgs::Vector3 currentTouchTranslationVelocity;
    tf2::Quaternion currentTouchAngularVelocity;
    geometry_msgs::PoseStamped currentTouchPose;
    geometry_msgs::PoseStamped prevTouchPose;

    // Fixed transformation quaternion between stylus and robot frames
    tf2::Quaternion stylusToRobotRotation;
    stylusToRobotRotation.setRPY(M_PI_2, M_PI, M_PI); // Replace roll, pitch, and yaw with the fixed rotation angles

    ros::init(argc, argv, "ur5e_control");

    ros::NodeHandle nodeHandle;

    // Load and switch to the desired UR5e position controller
    // LoadController(nodeHandle, UR5E_CONTROLLER_NAME);

    // if (!SwitchController(nodeHandle, UR5E_CONTROLLER_NAME))
    // {
    //     return -1;
    // }

    cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;
    cartesian_control_msgs::CartesianTrajectoryPoint targetPoint;

    CartesianActionClient trajectoryClient(
        "pose_based_cartesian_traj_controller/follow_cartesian_trajectory",
        true
    );

    bool hasFoundUR5ETransform = false;
    bool isGreyButtonPressed = false;
    bool isWhiteButtonPressed = false;

    ros::Subscriber touchStateSubscriber = nodeHandle.subscribe<omni_msgs::OmniState>(
        "/phantom/state",
        1,
        boost::bind(
            &touchStateCallback,
            _1,
            boost::ref(currentTouchPose),
            boost::ref(prevTouchPose),
            boost::ref(currentTouchTranslationVelocity),
            boost::ref(currentTouchAngularVelocity)
        )
    );

    ros::Subscriber touchButtonSubscriber = nodeHandle.subscribe<omni_msgs::OmniButtonEvent>(
        "/phantom/button",
        1,
        boost::bind(&touchButtonCallback, _1, boost::ref(isGreyButtonPressed), boost::ref(isWhiteButtonPressed))
    );

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
        // try
        // {
        //     TransformStampedToPoseStamped(tfBuffer.lookupTransform("base", "tool0_controller", ros::Time(0)), currentUR5EPose);
        //     hasFoundUR5ETransform = true;
        //     // ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: %s", PoseToString(currentUR5EPose.pose, 1).c_str());
        // }
        // catch (tf2::TransformException& exception)
        // {
        //     ROS_WARN_THROTTLE(1, "%s", exception.what());
        //     ROS_WARN_THROTTLE(1, "Unable to find UR5e base to TCP transform");
        // }

        // if (isGreyButtonPressed && hasFoundUR5ETransform && (ros::Time::now() - currentUR5EPose.header.stamp) <= ros::Duration(MAX_CONTROL_DELAY))
        // {
        //     // ROS_INFO_THROTTLE(
        //     //     1,
        //     //     "Current Pos: x: %.3f, y: %.3f, z: %.3f\nPrev Pos: x: %.3f, y: %.3f, z: %.3f",
        //     //     currentTouchPose.pose.position.x,
        //     //     currentTouchPose.pose.position.y,
        //     //     currentTouchPose.pose.position.z,
        //     //     prevTouchPose.pose.position.x,
        //     //     prevTouchPose.pose.position.y,
        //     //     prevTouchPose.pose.position.z
        //     // );

        //     for (int i = 1; i <= UR5E_CONTROL_NUM_SUBDIVISIONS; i++)
        //     {
        //         targetPoint.pose.position.x = currentUR5EPose.pose.position.x - 50 * UR5E_CONTROL_TIME_DELTA * currentTouchTranslationVelocity.y;
        //         targetPoint.pose.position.y = currentUR5EPose.pose.position.y + 50 * UR5E_CONTROL_TIME_DELTA * currentTouchTranslationVelocity.x;
        //         targetPoint.pose.position.z = currentUR5EPose.pose.position.z + 50 * UR5E_CONTROL_TIME_DELTA * currentTouchTranslationVelocity.z;

        //         tf2::Quaternion velocityIncrement, ur5eOrientation;
        //         tf2::fromMsg(currentUR5EPose.pose.orientation, ur5eOrientation);
        //         velocityIncrement = currentTouchAngularVelocity * UR5E_CONTROL_TIME_DELTA;
        //         targetPoint.pose.orientation = tf2::toMsg(ur5eOrientation * stylusToRobotRotation * velocityIncrement);
        //         // targetPoint.pose.orientation = StylusRotationToRobotFrame(stylusToRobotRotation, currentTouchPose.pose.orientation);
        //         // targetPoint.pose.orientation = currentUR5EPose.pose.orientation;
        //         // ROS_INFO_THROTTLE(1, "Sending UR5e to Pose: %s", PoseToString(targetPoint.pose, 1).c_str());
        //         // ROS_INFO_THROTTLE(1, "Current UR5e Pose: %s", PoseToString(currentUR5EPose.pose, 1).c_str());
        //         targetPoint.time_from_start = ros::Duration(i * UR5E_CONTROL_TIME_DELTA);

        //         goal.trajectory.points.push_back(targetPoint);
        //     }

        //     ROS_INFO_THROTTLE(1, "Sending UR5e to Pose: %s", PoseToString(goal.trajectory.points.back().pose, 1).c_str());

        //     trajectoryClient.waitForServer();
        //     trajectoryClient.sendGoal(goal);
        //     trajectoryClient.waitForResult(ros::Duration(5.0));

        //     try
        //     {
        //         TransformStampedToPoseStamped(tfBuffer.lookupTransform("base", "tool0_controller", ros::Time(0)), currentUR5EPose);
        //         hasFoundUR5ETransform = true;
        //         // ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: %s", PoseToString(currentUR5EPose.pose, 1).c_str());
        //     }
        //     catch (tf2::TransformException& exception)
        //     {
        //         ROS_WARN_THROTTLE(1, "%s", exception.what());
        //         ROS_WARN_THROTTLE(1, "Unable to find UR5e base to TCP transform");
        //     }

        //     ROS_INFO_THROTTLE(1, "Actual UR5e Pose: %s", PoseToString(currentUR5EPose.pose, 1).c_str());

        //     goal.trajectory.points.clear();

        //     if (trajectoryClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        //     {
        //         ROS_INFO_THROTTLE(1, "Trajectory successfully executed");
        //     }
        //     else
        //     {
        //         ROS_WARN("Failed to execute trajectory or timed out");
        //     }
        // }

        ros::spinOnce();
    }

    return 0;
}
