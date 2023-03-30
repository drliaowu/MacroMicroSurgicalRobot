#include <string>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "omni_msgs/OmniState.h"
#include "omni_msgs/OmniButtonEvent.h"
#include "sensor_msgs/JointState.h"
#include "actionlib/client/simple_action_client.h"
#include "cartesian_control_msgs/FollowCartesianTrajectoryAction.h"
#include "cartesian_control_msgs/FollowCartesianTrajectoryGoal.h"
#include "cartesian_control_msgs/CartesianTrajectoryPoint.h"
#include <tf/transform_datatypes.h>
#include "boost/bind.hpp"
#include <memory>

// Touch position is output in millimetres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Scale factor for translating stylus motion to robot motion
const double STYLUS_CONTROL_MOVEMENT_SCALE_FACTOR = 1;

typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianActionClient;

std::string TouchPoseToString(geometry_msgs::Pose pose)
{
    std::stringstream output;

    output << "Position: (x: "
        << pose.position.x * TOUCH_POSITION_UNIT_SCALE_FACTOR
        << "m, y: "
        << pose.position.y * TOUCH_POSITION_UNIT_SCALE_FACTOR
        << "m, z: "
        << pose.position.z * TOUCH_POSITION_UNIT_SCALE_FACTOR
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

void stateCallback(
    const omni_msgs::OmniState::ConstPtr& omniState, 
    CartesianActionClient& trajectoryClient,
    const bool& isGreyButtonPressed,
    const bool& isWhiteButtonPressed
)
{
    geometry_msgs::Pose pose = omniState->pose;
    geometry_msgs::Vector3 current = omniState->current;
    geometry_msgs::Vector3 velocity = omniState->velocity;

    // cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;
    // cartesian_control_msgs::CartesianTrajectoryPoint point;
    // point.pose.position;

    // trajectoryClient.waitForServer();

    // trajectoryClient.sendGoal(goal);
    // trajectoryClient.waitForResult(ros::Duration(5.0));

    // if (trajectoryClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     ROS_INFO("Trajectory successfully executed");
    // else
    //     ROS_WARN("Failed to execute trajectory or timed out");

    ROS_INFO_THROTTLE(1, "Pose: %s, Current: %s, Velocity: %s, Grey Button: %d, White Button: %d", 
        TouchPoseToString(omniState->pose).c_str(), 
        Vector3ToString(omniState->current).c_str(), 
        Vector3ToString(omniState->velocity).c_str(),
        isGreyButtonPressed,
        isWhiteButtonPressed
    );
}

void buttonCallback(
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "touch_subscriber");

    ros::NodeHandle nodeHandle;

    std::unique_ptr<CartesianActionClient> trajectoryClient = std::make_unique<CartesianActionClient>(
        "pose_based_cartesian_traj_controller/follow_cartesian_trajectory", 
        true
    );

    std::unique_ptr<bool> isGreyButtonPressed = std::make_unique<bool>(false);
    std::unique_ptr<bool> isWhiteButtonPressed = std::make_unique<bool>(false);

    ros::Subscriber touchStateSubscriber = nodeHandle.subscribe<omni_msgs::OmniState>(
        "/phantom/state", 
        1, 
        boost::bind(&stateCallback, _1, boost::ref(*trajectoryClient), boost::cref(*isGreyButtonPressed), boost::cref(*isWhiteButtonPressed))
    );

    ros::Subscriber touchButtonSubscriber = nodeHandle.subscribe<omni_msgs::OmniButtonEvent>(
        "/phantom/button", 
        1, 
        boost::bind(&buttonCallback, _1, boost::ref(*isGreyButtonPressed), boost::ref(*isWhiteButtonPressed))
    );

    ros::Rate rate(1);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
