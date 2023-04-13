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

// Touch position coordinates are output in millimetres, must convert to metres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Period in seconds of each UR5e pose control operation
const double UR5E_CONTROL_PERIOD = 1e-2;

// Number of subdivisions of a single trajectory to send to the UR5E, increase number to improve motion smoothness
const double UR5E_CONTROL_NUM_SUBDIVISIONS = 10;

// The duration, in seconds, of a single pose trajectory command sent to the UR5e
const double UR5E_CONTROL_TIME_INCREMENT = UR5E_CONTROL_PERIOD / UR5E_CONTROL_NUM_SUBDIVISIONS;

// The maximum age, in seconds, of the latest pose update received from the UR5e. Past this age, control is halted until the next update.
const double UR5E_MAX_POSE_AGE = 0.1;

// Maximum tolerable uncertainty in a double
const double EPSILON = 1e-6;

// Factor by which Touch translational movements are scaled before being sent to the UR5e
const double UR5E_TRANSLATION_SCALE_FACTOR = 3;

// Factor by which Touch rotational movements are scaled before being sent to the UR5e
const double UR5E_ROTATION_SCALE_FACTOR = 0.2;

const std::string UR5E_CONTROLLER_NAME = "pose_based_cartesian_traj_controller";

const std::string UR5E_ACTION_CLIENT_NAME = "pose_based_cartesian_traj_controller/follow_cartesian_trajectory";

typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianActionClient;

class Pose
{
public:
    tf2::Vector3 position;
    tf2::Quaternion orientation;

    Pose()
    {
        this->position.setX(0.0);
        this->position.setY(0.0);
        this->position.setZ(0.0);

        this->orientation.setX(0.0);
        this->orientation.setY(0.0);
        this->orientation.setZ(0.0);
        this->orientation.setW(0.0);
    }

    Pose(tf2::Vector3 position, tf2::Quaternion orientation)
    {
        this->position = position;
        this->orientation = orientation;
    }
};

bool IsZero(double value)
{
    return abs(value) < EPSILON;
}

bool IsValidQuaternion(tf2::Quaternion quaternion)
{
    return !((isnan(quaternion.getX()) || isnan(quaternion.getY()) || isnan(quaternion.getZ()) || isnan(quaternion.getW())) ||
           (IsZero(quaternion.getX()) && IsZero(quaternion.getY()) && IsZero(quaternion.getZ()) && IsZero(quaternion.getW())));
}

void TransformStampedToPoseStamped(const geometry_msgs::TransformStamped& transformStamped, geometry_msgs::PoseStamped& poseStamped)
{
    poseStamped.header = transformStamped.header;

    poseStamped.pose.position.x = transformStamped.transform.translation.x;
    poseStamped.pose.position.y = transformStamped.transform.translation.y;
    poseStamped.pose.position.z = transformStamped.transform.translation.z;

    poseStamped.pose.orientation = transformStamped.transform.rotation;
}

std::string PoseToString(Pose pose, const double position_unit_scale_factor)
{
    std::stringstream output;

    output << "Position: (x: "
        << pose.position.getX() * position_unit_scale_factor
        << "m, y: "
        << pose.position.getY() * position_unit_scale_factor
        << "m, z: "
        << pose.position.getZ() * position_unit_scale_factor
        << "m), Orientation: (x: "
        << pose.orientation.getX()
        << ", y: "
        << pose.orientation.getY()
        << ", z: "
        << pose.orientation.getZ()
        << ", w: "
        << pose.orientation.getW()
        << ")";

    return output.str();
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

std::string QuaternionToString(tf2::Quaternion quaternion)
{
    std::stringstream output;

    output << "x: "
        << quaternion.getX()
        << ", y: "
        << quaternion.getY()
        << ", z: "
        << quaternion.getZ()
        << ", w: "
        << quaternion.getW()
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
    Pose& currentPose
)
{
    tf2::fromMsg(omniState->pose.position, currentPose.position);

    // Scale touch position units to metres
    currentPose.position *= TOUCH_POSITION_UNIT_SCALE_FACTOR;

    tf2::fromMsg(omniState->pose.orientation, currentPose.orientation);

    geometry_msgs::Vector3 current = omniState->current;
}

void touchButtonCallback(
    const omni_msgs::OmniButtonEvent::ConstPtr& omniButtonStates,
    bool& isGreyButtonPressed,
    bool& isWhiteButtonPressed,
    Pose& currentUR5EPose,
    Pose& ur5eOriginPose,
    Pose& currentTouchPose,
    Pose& touchOriginPose
)
{
    // Update control origin poses if white button was just pressed
    if (omniButtonStates->white_button && !isWhiteButtonPressed)
    {
        ur5eOriginPose.position = currentUR5EPose.position;
        ur5eOriginPose.orientation = currentUR5EPose.orientation;

        touchOriginPose.position = currentTouchPose.position;
        touchOriginPose.orientation = currentTouchPose.orientation;
    }

    isGreyButtonPressed = (bool)omniButtonStates->grey_button;
    isWhiteButtonPressed = (bool)omniButtonStates->white_button;
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

void UpdateMovementGoal(
    cartesian_control_msgs::FollowCartesianTrajectoryGoal& goal,
    cartesian_control_msgs::CartesianTrajectoryPoint& targetPoint,
    Pose& currentUR5EPose,
    Pose& movementPoseDelta
)
{
    for (int i = 1; i <= UR5E_CONTROL_NUM_SUBDIVISIONS; i++)
    {
        targetPoint.pose.position = Vector3ToPoint(currentUR5EPose.position + movementPoseDelta.position / UR5E_CONTROL_NUM_SUBDIVISIONS);

        // Only attempt to apply the orientation delta if it is real and non-zero
        if (IsValidQuaternion(movementPoseDelta.orientation))
        {
            targetPoint.pose.orientation = tf2::toMsg(
                currentUR5EPose.orientation.slerp(
                    movementPoseDelta.orientation * currentUR5EPose.orientation,
                    UR5E_ROTATION_SCALE_FACTOR * i / UR5E_CONTROL_NUM_SUBDIVISIONS
                )
            );
        }
        // If there is no valid orientation delta, the UR5e should remain at its current orientation
        else
        {
            targetPoint.pose.orientation = tf2::toMsg(currentUR5EPose.orientation);
        }

        targetPoint.time_from_start = ros::Duration(i * UR5E_CONTROL_TIME_INCREMENT);

        goal.trajectory.points.push_back(targetPoint);
    }
}

geometry_msgs::Point Vector3ToPoint(tf2::Vector3 vector)
{
    geometry_msgs::Point point;
    point.x = vector.getX();
    point.y = vector.getY();
    point.z = vector.getZ();

    return point;
}

bool areSimilar(tf2::Quaternion a, tf2::Quaternion b)
{
    double maxDiff = 0.3;
    return (abs(a.getX() - b.getX()) < maxDiff) && (abs(a.getY() - b.getY()) < maxDiff) && (abs(a.getZ() - b.getZ()) < maxDiff);
}

int main(int argc, char **argv)
{
    Pose ur5eOriginPose;
    Pose touchOriginPose;

    Pose currentTouchPose;
    Pose currentUR5EPose;
    geometry_msgs::PoseStamped currentUR5EPoseStamped;

    Pose touchPoseDelta;
    Pose ur5ePoseDelta;
    Pose movementPoseDelta;

    // Fixed transformation quaternion between touch and UR5e frames
    tf2::Quaternion touchToUR5ERotation;
    touchToUR5ERotation.setRPY(M_PI_2, 0, M_PI_2);

    ros::init(argc, argv, "ur5e_control");

    ros::NodeHandle nodeHandle;

    // Load and switch to the desired UR5e position controller
    LoadController(nodeHandle, UR5E_CONTROLLER_NAME);

    if (!SwitchController(nodeHandle, UR5E_CONTROLLER_NAME))
    {
        // Failed to switch controller, exit program
        return -1;
    }

    cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;
    cartesian_control_msgs::CartesianTrajectoryPoint targetPoint;

    CartesianActionClient trajectoryClient(
        UR5E_ACTION_CLIENT_NAME,
        true
    );

    bool isGreyButtonPressed = false;
    bool isWhiteButtonPressed = false;

    ros::Subscriber touchStateSubscriber = nodeHandle.subscribe<omni_msgs::OmniState>(
        "/phantom/state",
        1,
        boost::bind(
            &touchStateCallback,
            _1,
            boost::ref(currentTouchPose)
        )
    );

    ros::Subscriber touchButtonSubscriber = nodeHandle.subscribe<omni_msgs::OmniButtonEvent>(
        "/phantom/button",
        1,
        boost::bind(
            &touchButtonCallback,
            _1,
            boost::ref(isGreyButtonPressed),
            boost::ref(isWhiteButtonPressed),
            boost::ref(currentUR5EPose),
            boost::ref(ur5eOriginPose),
            boost::ref(currentTouchPose),
            boost::ref(touchOriginPose)
        )
    );

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
        try
        {
            TransformStampedToPoseStamped(tfBuffer.lookupTransform("base", "tool0_controller", ros::Time(0)), currentUR5EPoseStamped);
            tf2::fromMsg(currentUR5EPoseStamped.pose.position, currentUR5EPose.position);
            tf2::fromMsg(currentUR5EPoseStamped.pose.orientation, currentUR5EPose.orientation);
        }
        catch (tf2::TransformException& exception)
        {
            ROS_WARN_THROTTLE(1, "%s", exception.what());
            ROS_WARN_THROTTLE(1, "Unable to find UR5e base to TCP transform");
            continue;
        }

        // Enable UR5e teleoperation if:
        // User is pressing the white button on the Touch
        // The current pose of the UR5e is not outdated
        if (isWhiteButtonPressed &&
            (ros::Time::now() - currentUR5EPoseStamped.header.stamp) <= ros::Duration(UR5E_MAX_POSE_AGE)
        )
        {
            // Update Touch position delta (in UR5E coordinate frame)
            // Touch +y-axis is aligned with UR5e -x-axis
            touchPoseDelta.position.setX(touchOriginPose.position.getY() - currentTouchPose.position.getY());
            // Touch +x-axis is aligned with UR5e -y-axis
            touchPoseDelta.position.setY(touchOriginPose.position.getX() - currentTouchPose.position.getX());
            touchPoseDelta.position.setZ(currentTouchPose.position.getZ() - touchOriginPose.position.getZ());

            // Update UR5E position delta
            ur5ePoseDelta.position = currentUR5EPose.position - ur5eOriginPose.position;

            // Final movement position delta is difference between Touch and UR5E position deltas
            movementPoseDelta.position = UR5E_TRANSLATION_SCALE_FACTOR * (touchPoseDelta.position - ur5ePoseDelta.position);

            ROS_INFO_THROTTLE(
                1,
                "Current Touch Orientation: %s, Touch origin orientation: %s",
                QuaternionToString(currentTouchPose.orientation).c_str(),
                QuaternionToString(touchOriginPose.orientation).c_str()
            );

            // Update Touch orientation delta (in UR5E coordinate frame)
            touchPoseDelta.orientation = (touchToUR5ERotation * currentTouchPose.orientation) * (touchToUR5ERotation * touchOriginPose.orientation).inverse();

            ROS_INFO_THROTTLE(1, "Touch orientation delta: %s", QuaternionToString(touchPoseDelta.orientation).c_str());

            ROS_INFO_THROTTLE(
                1,
                "Current UR5e Orientation: %s, UR5e origin orientation: %s",
                QuaternionToString(currentUR5EPose.orientation).c_str(),
                QuaternionToString(ur5eOriginPose.orientation).c_str()
            );

            // Update UR5E orientation delta
            ur5ePoseDelta.orientation = currentUR5EPose.orientation * ur5eOriginPose.orientation.inverse();

            ROS_INFO_THROTTLE(1, "UR5e orientation delta: %s", QuaternionToString(ur5ePoseDelta.orientation).c_str());

            // Final movement orientation delta is difference between Touch and UR5E orientation deltas
            movementPoseDelta.orientation = touchPoseDelta.orientation * ur5ePoseDelta.orientation.inverse();

            ROS_INFO_THROTTLE(1, "Movement pose delta: %s", PoseToString(movementPoseDelta, 1).c_str());



            trajectoryClient.waitForServer();
            trajectoryClient.sendGoal(goal);
            trajectoryClient.waitForResult(ros::Duration(5.0));

            goal.trajectory.points.clear();

            if (trajectoryClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_WARN_THROTTLE(1, "Failed to execute trajectory or timed out");
            }
        }

        ros::spinOnce();
    }

    return 0;
}
