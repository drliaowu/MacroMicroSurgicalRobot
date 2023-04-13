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

const double EPSILON = 1e-6;

const double UR5E_TRANSLATION_SCALE_FACTOR = 3;

const double UR5E_ROTATION_SCALE_FACTOR = 0.2;

const std::string UR5E_CONTROLLER_NAME = "pose_based_cartesian_traj_controller";

typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianActionClient;

enum ControlType
{
    Velocity,
    PoseDelta
};

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

    tf2::Quaternion output = QuaternionExponential(QuaternionLogarithm(diffQuater) / poseDt) * prevConjQuaternion * 2;

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

    // ROS_INFO_THROTTLE(1, "Touch State:\n\tPose: %s\n\tCurrent: %s\n\tVelocity: %s",
    //     PoseToString(omniState->pose, TOUCH_POSITION_UNIT_SCALE_FACTOR).c_str(),
    //     Vector3ToString(omniState->current).c_str(),
    //     Vector3ToString(omniState->velocity).c_str()
    // );
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
    // Update origin poses if grey button was just pressed
    if (omniButtonStates->grey_button && !isGreyButtonPressed)
    {
        ur5eOriginPose.position = currentUR5EPose.position;
        ur5eOriginPose.orientation = currentUR5EPose.orientation;

        touchOriginPose.position = currentTouchPose.position;
        touchOriginPose.orientation = currentTouchPose.orientation;
    }

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

    // Fixed transformation quaternion between stylus and robot frames
    tf2::Quaternion stylusToRobotRotation;
    stylusToRobotRotation.setRPY(M_PI_2, 0, M_PI_2);

    ros::init(argc, argv, "ur5e_control");

    // tf2::Quaternion ur5eForward(0.0124195, -0.698029, -0.0369087, 0.71501);
    // tf2::Quaternion ur5eLeft(0.517401, -0.482418, 0.464211, 0.532989);
    // tf2::Quaternion ur5eBackward(0.719279, 0.0157842, 0.69346, 0.0387527);
    // tf2::Quaternion ur5eDown(0.999209, 0.0359766, -0.00302336, 0.0166865);

    // tf2::Quaternion touchForward(0.038666, -0.0109472, -0.698358, 0.71462);
    // tf2::Quaternion touchLeft(0.499847, -0.483226, 0.535441, -0.47953);
    // tf2::Quaternion touchBackward(0.698114, -0.660017, -0.191376, 0.200971);
    // tf2::Quaternion touchDown(-0.564923, 0.368878, 0.330237, 0.660102);

    // double R;
    // double P;
    // double Y;

    // for (int i = 0; i < 4; i++)
    // {
    //     R = -M_PI_2 + M_PI_2 * i;

    //     for (int j = 0; j < 4; j++)
    //     {
    //         P = -M_PI_2 + M_PI_2 * j;

    //         for (int k = 0; k < 4; k++)
    //         {
    //             Y = -M_PI_2 + M_PI_2 * k;

    //             stylusToRobotRotation.setRPY(R, P, Y);

    //             if (
    //                 areSimilar(ur5eForward, stylusToRobotRotation * touchForward) ||
    //                 areSimilar(ur5eLeft, stylusToRobotRotation * touchLeft) ||
    //                 areSimilar(ur5eBackward, stylusToRobotRotation * touchBackward) ||
    //                 areSimilar(ur5eDown, stylusToRobotRotation * touchDown)
    //             )
    //             {
    //                 ROS_INFO("RPY: %d, %d, %d\n", i, j, k);

    //                 ROS_INFO(
    //                     "Forward:\n\tur5e:   \t\t%s\n\ttransformed touch: %s\n",
    //                     QuaternionToString(ur5eForward).c_str(),
    //                     QuaternionToString(stylusToRobotRotation * touchForward).c_str()
    //                 );

    //                 ROS_INFO(
    //                     "Left:\n\tur5e:   \t\t%s\n\ttransformed touch: %s\n",
    //                     QuaternionToString(ur5eLeft).c_str(),
    //                     QuaternionToString(stylusToRobotRotation * touchLeft).c_str()
    //                 );

    //                 ROS_INFO(
    //                     "Backward:\n\tur5e:   \t\t%s\n\ttransformed touch: %s\n",
    //                     QuaternionToString(ur5eBackward).c_str(),
    //                     QuaternionToString(stylusToRobotRotation * touchBackward).c_str()
    //                 );

    //                 ROS_INFO(
    //                     "Down:\n\tur5e:   \t\t%s\n\ttransformed touch: %s\n",
    //                     QuaternionToString(ur5eDown).c_str(),
    //                     QuaternionToString(stylusToRobotRotation * touchDown).c_str()
    //                 );
    //             }
    //         }
    //     }
    // }

    // return 0;

    ros::NodeHandle nodeHandle;

    // Load and switch to the desired UR5e position controller
    LoadController(nodeHandle, UR5E_CONTROLLER_NAME);

    if (!SwitchController(nodeHandle, UR5E_CONTROLLER_NAME))
    {
        return -1;
    }

    cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;
    cartesian_control_msgs::CartesianTrajectoryPoint targetPoint;

    CartesianActionClient trajectoryClient(
        "pose_based_cartesian_traj_controller/follow_cartesian_trajectory",
        true
    );

    bool hasTouchAngularVelocity = false;
    bool hasFoundUR5ETransform = false;
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
            hasFoundUR5ETransform = true;
            // ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: %s", PoseToString(currentUR5EPoseStamped.pose, 1).c_str());
        }
        catch (tf2::TransformException& exception)
        {
            ROS_WARN_THROTTLE(1, "%s", exception.what());
            ROS_WARN_THROTTLE(1, "Unable to find UR5e base to TCP transform");
        }

        if (
            isGreyButtonPressed &&
            hasFoundUR5ETransform &&
            (ros::Time::now() - currentUR5EPoseStamped.header.stamp) <= ros::Duration(MAX_CONTROL_DELAY)
        )
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

            // Update Touch position delta (in UR5E coordinate frame)
            touchPoseDelta.position.setX(touchOriginPose.position.getY() - currentTouchPose.position.getY());
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
            touchPoseDelta.orientation = (stylusToRobotRotation * currentTouchPose.orientation) * (stylusToRobotRotation * touchOriginPose.orientation).inverse();

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

            for (int i = 1; i <= UR5E_CONTROL_NUM_SUBDIVISIONS; i++)
            {
                targetPoint.pose.position = Vector3ToPoint(currentUR5EPose.position + movementPoseDelta.position / UR5E_CONTROL_NUM_SUBDIVISIONS);

                // tf2::Quaternion velocityIncrement, ur5eOrientation, currentTouchOrientation, touchOriginOrientation;
                // tf2::fromMsg(currentUR5EPoseStamped.pose.orientation, ur5eOrientation);
                // tf2::fromMsg(currentTouchPoseStamped.pose.orientation, currentTouchOrientation);
                // tf2::fromMsg(TouchOriginPose.orientation, touchOriginOrientation);
                // velocityIncrement = currentTouchOrientation * touchOriginOrientation.inverse();
                // targetPoint.pose.orientation = tf2::toMsg((stylusToRobotRotation * velocityIncrement) * ur5eOrientation);
                // targetPoint.pose.orientation = StylusRotationToRobotFrame(stylusToRobotRotation, currentTouchPoseStamped.pose.orientation);

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
                    targetPoint.pose.orientation = currentUR5EPoseStamped.pose.orientation;
                }
                // ROS_INFO_THROTTLE(1, "Sending UR5e to Pose: %s", PoseToString(targetPoint.pose, 1).c_str());
                // ROS_INFO_THROTTLE(1, "Current UR5e Pose: %s", PoseToString(currentUR5EPose.pose, 1).c_str());

                targetPoint.time_from_start = ros::Duration(i * UR5E_CONTROL_TIME_DELTA);

                goal.trajectory.points.push_back(targetPoint);
            }

            // ROS_INFO_THROTTLE(1, "Sending UR5e to Pose: %s", PoseToString(goal.trajectory.points.back().pose, 1).c_str());

            trajectoryClient.waitForServer();
            trajectoryClient.sendGoal(goal);
            trajectoryClient.waitForResult(ros::Duration(5.0));

            try
            {
                TransformStampedToPoseStamped(tfBuffer.lookupTransform("base", "tool0_controller", ros::Time(0)), currentUR5EPoseStamped);
                hasFoundUR5ETransform = true;
                // ROS_INFO_THROTTLE(1, "UR5e Cartesian Pose: %s", PoseToString(currentUR5EPose, 1).c_str());
            }
            catch (tf2::TransformException& exception)
            {
                ROS_WARN_THROTTLE(1, "%s", exception.what());
                ROS_WARN_THROTTLE(1, "Unable to find UR5e base to TCP transform");
            }

            // ROS_INFO_THROTTLE(1, "Actual UR5e Pose: %s", PoseToString(currentUR5EPoseStamped.pose, 1).c_str());

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
