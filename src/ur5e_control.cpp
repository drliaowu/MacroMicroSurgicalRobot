#include <string>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "omni_msgs/OmniState.h"

std::string PoseToString(geometry_msgs::Pose pose)
{
    std::stringstream output;

    output << "Position: (x: "
        << pose.position.x
        << ", y: "
        << pose.position.y
        << ", z: "
        << pose.position.z
        << "), Orientation: (x: "
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

void stateCallback(const omni_msgs::OmniState::ConstPtr& omniState)
{
    geometry_msgs::Pose pose = omniState->pose;
    geometry_msgs::Vector3 current = omniState->current;
    geometry_msgs::Vector3 velocity = omniState->velocity;

    ROS_INFO("Pose: %s, Current: %s, Velocity: %s", 
        PoseToString(omniState->pose).c_str(), 
        Vector3ToString(omniState->current).c_str(), 
        Vector3ToString(omniState->velocity).c_str()
    );
}

void stateCallback(const omni_msgs::OmniButtonEvent::ConstPtr& omniButtonStates)
{
    std_msgs::Int32 grey_button = omniButtonStates->grey_button;
    std_msgs::Int32 white_button = omniButtonStates->white_button;

    ROS_INFO("Grey button: %d, white button: %d", 
        grey_button, 
        white_button
    );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "touch_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/phantom/state", 1, stateCallback);
    ros::Subscriber sub = n.subscribe("/phantom/button", 1, stateCallback);

    ros::Rate rate(1);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
