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
#include <tf/transform_datatypes.h>
#include "boost/bind.hpp"
#include "boost/thread.hpp"
#include <memory>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <tf2_ros/transform_listener.h>

// Touch position coordinates are output in millimetres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Scale factor for translating stylus motion to robot motion
const double TOUCH_CONTROL_MOVEMENT_SCALE_FACTOR = 1;

const int UR5E_JOINT_COUNT = 6;

const double MAX_CONTROL_DELAY = 0.1;

typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianActionClient;

bool JointStatesToCartesianPose(const sensor_msgs::JointState::ConstPtr& jointState, const KDL::Chain& chain, geometry_msgs::PoseStamped& cartesianPose)
{
    KDL::JntArray jointPositions(chain.getNrOfJoints());

    for (size_t i = 0; i < jointState->position.size(); ++i) 
    {
        jointPositions(i) = jointState->position[i];
    }

    KDL::Frame endEffectorFrame;
    KDL::ChainFkSolverPos_recursive fkSolver(chain);
    int kinematicsStatus = fkSolver.JntToCart(jointPositions, endEffectorFrame);

    if (kinematicsStatus >= 0)
    {
        cartesianPose.header.stamp = ros::Time::now();
        cartesianPose.header.frame_id = "base_link";

        cartesianPose.pose.position.x = endEffectorFrame.p.x();
        cartesianPose.pose.position.y = endEffectorFrame.p.y();
        cartesianPose.pose.position.z = endEffectorFrame.p.z();

        endEffectorFrame.M.GetQuaternion(
            cartesianPose.pose.orientation.x,
            cartesianPose.pose.orientation.y,
            cartesianPose.pose.orientation.z,
            cartesianPose.pose.orientation.w
        );
        return true;
    }

    ROS_ERROR("Failed to compute forward kinematics.");
    return false;
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

void ur5eJointStateCallback(
    const sensor_msgs::JointState::ConstPtr& jointState, 
    const tf2_ros::Buffer& tfBuffer, 
    geometry_msgs::PoseStamped& cartesianPose
)
{
    // std::stringstream output;

    // output << "UR5e Joint States:\n";

    // for (int i = 0; i < UR5E_JOINT_COUNT; i++)
    // {
    //     output << "\tJoint " 
    //         << i + 1
    //         << ":\n"
    //         << "\t\tName: "
    //         << jointState->name[i]
    //         << "\n\t\tPosition: "
    //         << jointState->position[i]
    //         << "rad\n\t\tVelocity:"
    //         << jointState->velocity[i]
    //         << "rad/s\n\t\tEffort:"
    //         << jointState->effort[i]
    //         << "N\n\n";
    // }

    // ROS_INFO_THROTTLE(1, output.str().c_str());

    // JointStatesToCartesianPose(jointState, chain, cartesianPose);

    try
    {
        TransformStampedToPoseStamped(tfBuffer.lookupTransform("base_link", "tool0", ros::Time(0)), cartesianPose);
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("base_link", "tool0", ros::Time(0));
        // ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: %s", PoseToString(cartesianPose.pose, 1).c_str());
        ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: x: %.3fm, y: %.3fm, z: %.3fm", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    }
    catch (tf2::TransformException& exception)
    {
        ROS_WARN_THROTTLE(1, "%s", exception.what());
        ROS_WARN_THROTTLE(1, "Unable to find UR5e base to TCP transform");
    }

    // ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: %s", PoseToString(cartesianPose.pose, 1).c_str());
}

void touchStateCallback(
    const omni_msgs::OmniState::ConstPtr& omniState, 
    geometry_msgs::PoseStamped& poseStamped,
    geometry_msgs::PoseStamped& prevPose
)
{
    if ((ros::Time::now() - prevPose.header.stamp) > ros::Duration(0.1))
    {
        prevPose = geometry_msgs::PoseStamped(poseStamped);
    }

    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose = omniState->pose;

    // Scale touch position units to metres
    poseStamped.pose.position.x *= TOUCH_POSITION_UNIT_SCALE_FACTOR;
    poseStamped.pose.position.y *= TOUCH_POSITION_UNIT_SCALE_FACTOR;
    poseStamped.pose.position.z *= TOUCH_POSITION_UNIT_SCALE_FACTOR;

    geometry_msgs::Vector3 current = omniState->current;
    geometry_msgs::Vector3 velocity = omniState->velocity;

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

void DHTableToChain(const std::array<std::array<double, 4>, UR5E_JOINT_COUNT>& dh_table, KDL::Chain& chain) {
    for (const std::array<double, 4>& params : dh_table) 
    {
        double a = params[0];
        double alpha = params[1];
        double d = params[2];
        double theta = params[3];

        KDL::Joint joint(KDL::Joint::RotZ);
        KDL::Frame frame = KDL::Frame(KDL::Rotation::RPY(0, 0, alpha), KDL::Vector(a, 0, 0)) *
                           KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0, 0, d));
        chain.addSegment(KDL::Segment(joint, frame));
    }
}

void RunTrajectoryClient(
    const bool& hasFoundUR5ETransform,
    CartesianActionClient& trajectoryClient,
    cartesian_control_msgs::FollowCartesianTrajectoryGoal& goal,
    cartesian_control_msgs::CartesianTrajectoryPoint& targetPoint, 
    const geometry_msgs::PoseStamped& currentUR5EPose,
    const geometry_msgs::PoseStamped& currentTouchPose,
    const geometry_msgs::PoseStamped& prevTouchPose
    )
{
    if (hasFoundUR5ETransform && (ros::Time::now() - currentUR5EPose.header.stamp) <= ros::Duration(MAX_CONTROL_DELAY))
    {
        // ROS_INFO_THROTTLE(
        //     1, 
        //     "Current Pos: x: %.3f, y: %.3f, z: %.3f\nPrev Pos: x: %.3f, y: %.3f, z: %.3f",
        //     currentTouchPose.pose.position.x,
        //     currentTouchPose.pose.position.y,
        //     currentTouchPose.pose.position.z,
        //     prevTouchPose.pose.position.x,
        //     prevTouchPose.pose.position.y,
        //     prevTouchPose.pose.position.z
        // );

        targetPoint.pose.position.x = -currentUR5EPose.pose.position.x + TOUCH_CONTROL_MOVEMENT_SCALE_FACTOR * (currentTouchPose.pose.position.x - prevTouchPose.pose.position.x);
        targetPoint.pose.position.y = -currentUR5EPose.pose.position.y + TOUCH_CONTROL_MOVEMENT_SCALE_FACTOR * (currentTouchPose.pose.position.z - prevTouchPose.pose.position.z);
        targetPoint.pose.position.z = currentUR5EPose.pose.position.z + TOUCH_CONTROL_MOVEMENT_SCALE_FACTOR * (currentTouchPose.pose.position.y - prevTouchPose.pose.position.y);

        targetPoint.pose.orientation = currentUR5EPose.pose.orientation;

        ROS_INFO_THROTTLE(1, "Sending UR5e to Pose: %s", PoseToString(targetPoint.pose, 1).c_str());
        goal.trajectory.points.push_back(targetPoint);
        trajectoryClient.waitForServer();
        trajectoryClient.sendGoal(goal);
        trajectoryClient.waitForResult(ros::Duration(5.0));

        goal.trajectory.points.clear();

        if (trajectoryClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Trajectory successfully executed");
        else
            ROS_WARN("Failed to execute trajectory or timed out");
    }
}

int main(int argc, char **argv)
{
    // DH parameters sourced from: 
    // https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    std::array<std::array<double, 4>, UR5E_JOINT_COUNT> ur5eDHTable{{
        {{ 0, 0, 0.1625, M_PI / 2 }},
        {{ 0, -0.425, 0, 0 }},
        {{ 0, -0.3922, 0, 0 }},
        {{ 0, 0, 0.1333, M_PI / 2 }},
        {{ 0, 0, 0.0997, -M_PI / 2 }},
        {{ 0, 0, 0.0996, 0 }}
    }};

    KDL::Chain chain;
    DHTableToChain(ur5eDHTable, chain);

    geometry_msgs::PoseStamped currentUR5EPose;
    geometry_msgs::PoseStamped currentTouchPose;
    geometry_msgs::PoseStamped prevTouchPose;

    prevTouchPose.header.stamp = ros::Time(0);

    ros::init(argc, argv, "touch_subscriber");

    ros::NodeHandle nodeHandle;

    std::string controller_name = "pose_based_cartesian_traj_controller";

    // Create a client for the load_controller service
    ros::ServiceClient load_controller_client = nodeHandle.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");

    // Create a service request
    controller_manager_msgs::LoadController srv;
    srv.request.name = controller_name;

    // Call the service to load the controller
    if (load_controller_client.call(srv))
    {
        if (srv.response.ok)
        {
            ROS_INFO("Successfully loaded controller: %s", controller_name.c_str());
        }
        else
        {
            ROS_ERROR("Failed to load controller: %s", controller_name.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call load_controller service");
    }

    ros::ServiceClient switch_controller_client = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switch_controller_srv;
    switch_controller_srv.request.start_controllers.push_back(controller_name);
    switch_controller_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    if (switch_controller_client.call(switch_controller_srv)) {
        ROS_INFO("Controller started");
    } else {
        ROS_ERROR("Failed to start controller");
    }

    cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;
    cartesian_control_msgs::CartesianTrajectoryPoint targetPoint;

    CartesianActionClient trajectoryClient(
        "pose_based_cartesian_traj_controller/follow_cartesian_trajectory", 
        true
    );

    bool hasFoundUR5ETransform = false;
    bool isGreyButtonPressed = false;
    bool isWhiteButtonPressed = false;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // ros::Subscriber ur5eJointStateSubscriber = nodeHandle.subscribe<sensor_msgs::JointState>(
    //     "/joint_states",
    //     1,
    //     boost::bind(&ur5eJointStateCallback, _1, boost::cref(tfBuffer), boost::ref(currentUR5EPose))
    // );

    ros::Subscriber touchStateSubscriber = nodeHandle.subscribe<omni_msgs::OmniState>(
        "/phantom/state", 
        1, 
        boost::bind(&touchStateCallback, _1, boost::ref(currentTouchPose), boost::ref(prevTouchPose))
    );

    ros::Subscriber touchButtonSubscriber = nodeHandle.subscribe<omni_msgs::OmniButtonEvent>(
        "/phantom/button", 
        1, 
        boost::bind(&touchButtonCallback, _1, boost::ref(isGreyButtonPressed), boost::ref(isWhiteButtonPressed))
    );

    while (ros::ok())
    {
        try
        {
            TransformStampedToPoseStamped(tfBuffer.lookupTransform("base_link", "tool0", ros::Time(0)), currentUR5EPose);
            hasFoundUR5ETransform = true;
            // ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: %s", PoseToString(currentUR5EPose.pose, 1).c_str());
        }
        catch (tf2::TransformException& exception)
        {
            ROS_WARN_THROTTLE(1, "%s", exception.what());
            ROS_WARN_THROTTLE(1, "Unable to find UR5e base to TCP transform");
        }

        if (hasFoundUR5ETransform && (ros::Time::now() - currentUR5EPose.header.stamp) <= ros::Duration(MAX_CONTROL_DELAY))
        {
            // ROS_INFO_THROTTLE(
            //     1, 
            //     "Current Pos: x: %.3f, y: %.3f, z: %.3f\nPrev Pos: x: %.3f, y: %.3f, z: %.3f",
            //     currentTouchPose.pose.position.x,
            //     currentTouchPose.pose.position.y,
            //     currentTouchPose.pose.position.z,
            //     prevTouchPose.pose.position.x,
            //     prevTouchPose.pose.position.y,
            //     prevTouchPose.pose.position.z
            // );

            targetPoint.pose.position.x = -currentUR5EPose.pose.position.x + TOUCH_CONTROL_MOVEMENT_SCALE_FACTOR * (currentTouchPose.pose.position.x - prevTouchPose.pose.position.x);
            targetPoint.pose.position.y = -currentUR5EPose.pose.position.y + TOUCH_CONTROL_MOVEMENT_SCALE_FACTOR * (currentTouchPose.pose.position.z - prevTouchPose.pose.position.z);
            targetPoint.pose.position.z = currentUR5EPose.pose.position.z + TOUCH_CONTROL_MOVEMENT_SCALE_FACTOR * (currentTouchPose.pose.position.y - prevTouchPose.pose.position.y);

            targetPoint.pose.orientation = currentUR5EPose.pose.orientation;
            targetPoint.time_from_start = ros::Duration(0.1);

            ROS_INFO_THROTTLE(1, "Sending UR5e to Pose: %s", PoseToString(targetPoint.pose, 1).c_str());
            goal.trajectory.points.push_back(targetPoint);
            trajectoryClient.waitForServer();
            trajectoryClient.sendGoal(goal);
            trajectoryClient.waitForResult(ros::Duration(5.0));

            goal.trajectory.points.clear();

            if (trajectoryClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Trajectory successfully executed");
            else
                ROS_WARN("Failed to execute trajectory or timed out");
        }

        ros::spinOnce();
    }

    return 0;
}
