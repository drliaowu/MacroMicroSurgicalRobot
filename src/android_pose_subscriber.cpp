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
#include <visualization_msgs/Marker.h>

// Touch position coordinates are output in millimetres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

const double EPSILON = 1e-6;

bool IsValidDouble(double value)
{
    return !(isnan(value) || abs(value) < EPSILON);
}

bool IsValidQuaternion(tf2::Quaternion quaternion)
{
    return IsValidDouble(quaternion.getX()) &&
           IsValidDouble(quaternion.getY()) &&
           IsValidDouble(quaternion.getZ()) &&
           IsValidDouble(quaternion.getW());
}

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

tf2::Quaternion GetAngularVelocitySimple(
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

    tf2::Quaternion output = QuaternionExponential(QuaternionLogarithm(diffQuater) / poseDt) * prevConjQuaternion * 2;

    return output;
}

// Function to calculate the average angular velocity from two timestamped poses and the desired time delta in seconds
tf2::Quaternion GetAngularVelocity(
    const geometry_msgs::PoseStamped& prevPose,
    const geometry_msgs::PoseStamped& currentPose
)
{
    // Time difference between the two pose measurements
    double poseDt = (currentPose.header.stamp - prevPose.header.stamp).toSec();

    // Convert orientations from quaternion messages to tf2 quaternions
    tf2::Quaternion prevOrientation, currentOrientation;
    tf2::fromMsg(prevPose.pose.orientation, prevOrientation);
    tf2::fromMsg(currentPose.pose.orientation, currentOrientation);

    tf2::Quaternion prevConjQuaternion = prevOrientation.inverse();

    tf2::Quaternion diffQuater = currentOrientation * prevConjQuaternion;

    tf2::Quaternion output = QuaternionExponential((QuaternionLogarithm(diffQuater) * 2) / poseDt) * prevConjQuaternion;

    return output;
}

void TransformStampedToPoseStamped(const geometry_msgs::TransformStamped& transformStamped, geometry_msgs::PoseStamped& poseStamped)
{
    poseStamped.header = transformStamped.header;

    poseStamped.pose.position.x = transformStamped.transform.translation.x;
    poseStamped.pose.position.y = transformStamped.transform.translation.y;
    poseStamped.pose.position.z = transformStamped.transform.translation.z;

    poseStamped.pose.orientation = transformStamped.transform.rotation;
}

std::string QuaternionToString(geometry_msgs::Quaternion quaternion)
{
    std::stringstream output;

    output << "x: "
        << quaternion.x
        << ", y: "
        << quaternion.y
        << ", z: "
        << quaternion.z
        << ", w: "
        << quaternion.w;

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
        << quaternion.getW();

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
        << "m), Orientation: ("
        << QuaternionToString(pose.orientation)
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

void poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& poseStamped,
    geometry_msgs::PoseStamped& currentPoseStamped,
    geometry_msgs::PoseStamped& prevPoseStamped,
    tf2::Quaternion& angularVelocity
)
{
    prevPoseStamped = currentPoseStamped;
    currentPoseStamped = *poseStamped;

    angularVelocity = GetAngularVelocity(prevPoseStamped, currentPoseStamped);
    ROS_INFO("Received pose, stamp (%d.%d): %s", poseStamped->header.stamp.sec, poseStamped->header.stamp.nsec, PoseToString(poseStamped->pose, 1).c_str());
    ROS_INFO("Angular velocity: %s", QuaternionToString(angularVelocity).c_str());
}

void touchStateCallback(
    const omni_msgs::OmniState::ConstPtr& omniState,
    geometry_msgs::PoseStamped& poseStamped,
    geometry_msgs::PoseStamped& prevPoseStamped,
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

    ROS_INFO_THROTTLE(1, "Current Touch Pose: %s, Prev Touch Pose: %s", PoseToString(poseStamped.pose, 1).c_str(), PoseToString(prevPoseStamped.pose, 1).c_str());

    angularVelocity = GetAngularVelocity(prevPoseStamped, poseStamped);

    ROS_INFO_THROTTLE(1, "Current Touch Angular Velocity: x: %.3f, y: %.3f, z: %.3f, w: %.3f", angularVelocity.getX(), angularVelocity.getY(), angularVelocity.getZ(), angularVelocity.getW());

    geometry_msgs::Vector3 current = omniState->current;

    // ROS_INFO_THROTTLE(1, "Touch State:\n\tPose: %s\n\tCurrent: %s\n\tVelocity: %s",
    //     PoseToString(omniState->pose, TOUCH_POSITION_UNIT_SCALE_FACTOR).c_str(),
    //     Vector3ToString(omniState->current).c_str(),
    //     Vector3ToString(omniState->velocity).c_str()
    // );
}

int main(int argc, char **argv)
{
    geometry_msgs::PoseStamped prevPoseStamped;
    geometry_msgs::PoseStamped currentPoseStamped;
    tf2::Quaternion currentAngularVelocity;

    prevPoseStamped.pose.orientation.w = 0.5;
    prevPoseStamped.pose.orientation.x = 0.5;
    prevPoseStamped.pose.orientation.y = 0.5;
    prevPoseStamped.pose.orientation.z = 0.5;

    currentPoseStamped.pose.orientation.w = 0.4;
    currentPoseStamped.pose.orientation.x = 0.4;
    currentPoseStamped.pose.orientation.y = 0.4;
    currentPoseStamped.pose.orientation.z = 0.4;

    currentAngularVelocity.setW(0.5);
    currentAngularVelocity.setX(0.5);
    currentAngularVelocity.setY(0.5);
    currentAngularVelocity.setZ(0.5);

    ros::init(argc, argv, "android_pose_subscriber");

    ros::NodeHandle nodeHandle;

    // ros::Subscriber poseSubscriber = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
    //     "android_pose",
    //     1,
    //     boost::bind(
    //         &poseCallback,
    //         _1,
    //         boost::ref(currentPoseStamped),
    //         boost::ref(prevPoseStamped),
    //         boost::ref(currentAngularVelocity)
    //     )
    // );

    ros::Subscriber touchStateSubscriber = nodeHandle.subscribe<omni_msgs::OmniState>(
        "/phantom/state",
        1,
        boost::bind(
            &touchStateCallback,
            _1,
            boost::ref(currentPoseStamped),
            boost::ref(prevPoseStamped),
            boost::ref(currentAngularVelocity)
        )
    );

    ros::Publisher vis_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish(marker);

    marker.action = visualization_msgs::Marker::MODIFY;

    ros::Rate rate(100);

    ROS_INFO("Starting subscriber");

    while (ros::ok())
    {
        if (IsValidQuaternion(currentAngularVelocity))
        {
            tf2::Quaternion markerOrientation;
            tf2::fromMsg(marker.pose.orientation, markerOrientation);
            ROS_INFO_THROTTLE(1, "Prev marker orientation: %s", QuaternionToString(marker.pose.orientation).c_str());
            geometry_msgs::Quaternion newOrientation = tf2::toMsg((currentAngularVelocity * 1e-5 * markerOrientation).normalize());
            ROS_INFO_THROTTLE(1, "New marker orientation: %s", QuaternionToString(newOrientation).c_str());
            marker.pose.orientation = newOrientation;
            vis_pub.publish(marker);

            currentAngularVelocity.setX(0);
            currentAngularVelocity.setY(0);
            currentAngularVelocity.setZ(0);
            currentAngularVelocity.setW(1);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
