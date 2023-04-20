#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "omni_msgs/OmniState.h"
#include "omni_msgs/OmniButtonEvent.h"
#include <memory>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosbag/bag.h>
#include "../include/ManipulatorModule.hpp"
#include "../include/Pose.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include "motor_angles_msg/MotorAngles.h"

// Touch position coordinates are output in millimetres, must convert to metres
const double TOUCH_POSITION_UNIT_SCALE_FACTOR = 1e-3;

// Factor by which Touch rotational movements are scaled before being sent to the manipulator
const double MANIPULATOR_ROTATION_SCALE_FACTOR = 0.2;

// Maximum joint velocity for a single update, in metres per second
const double JOINT_UPDATE_MAX_VELOCITY = 0.001;

// Maximum error in end-effector pose, in metres
const double MAX_POSE_ERROR = 0.002;

// Diameter of the manipulator, in metres
const double MANIPULATOR_DIAMETER = 0.004;

// Offset of end-effector tip from the centre of the final joint
const double END_EFFECTOR_TIP_OFFSET = 0.00244;

// Radius of the actuator pulley wheels, in metres
const double PULLEY_WHEEL_RADIUS = 0.01125;

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

typedef std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > TransformVector;

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

void PrintMatrix(const Eigen::MatrixXd& matrix, const char* title)
{
    std::stringstream output;

    output << matrix;

    ROS_INFO("%s:\n%s", title, output.str().c_str());
}

Eigen::Matrix3d xRotationMatrix(double theta)
{
    Eigen::Matrix3d rotation;

    rotation << 1, 0, 0,
                0, cos(theta), -sin(theta),
                0, sin(theta), cos(theta);

    return rotation;
}

Eigen::Matrix3d yRotationMatrix(double theta)
{
    Eigen::Matrix3d rotation;

    rotation << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

    return rotation;
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

Eigen::Matrix4d TransformFromPosition(const Eigen::Vector3d& position)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    transform.block(0, 3, 3, 1) = position;

    return transform;
}

Eigen::Matrix4d TransformFromRotation(const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    transform.block(0, 0, 3, 3) = rotation;

    return transform;
}

Eigen::Matrix4d TransformFromPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    transform.block(0, 0, 3, 3) = rotation;
    transform.block(0, 3, 3, 1) = position;

    return transform;
}

// Extract the position vector from a homogeneous transform
Eigen::Vector3d GetTransformPosition(const Eigen::Matrix4d& transform)
{
    Eigen::Vector3d position;
    position << transform.block(0, 3, 3, 1);

    return position;
}

Eigen::Matrix3d GetTransformRotation(const Eigen::Matrix4d& transform)
{
    Eigen::Matrix3d rotation;
    rotation = transform.block(0, 0, 3, 3);

    return rotation;
}

TransformVector GetJointTransforms(const Eigen::Matrix4d& baseTransform, const Eigen::Matrix4d toolTransform, const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    std::vector<ManipulatorModule> modules {proximal, distal};

    Eigen::Matrix4d moduleInterfaceAngularOffset = zRotationTransform(-M_PI_4);

    Eigen::Matrix4d endTransform = baseTransform;

    Eigen::Matrix4d panJointTransform = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d tiltJointTransform = Eigen::Matrix4d::Zero();

    TransformVector jointTransforms;

    for (int i = 0; i < modules.size(); i++)
    {
        panJointTransform = modules[i].GetPanJointTransform();
        tiltJointTransform = modules[i].GetTiltJointTransform();

        // PrintMatrix(panJointTransform, "Pan Joint Transform");
        // PrintMatrix(tiltJointTransform, "Tilt Joint Transform");

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (j % 2 != 0))
            {
                endTransform *= panJointTransform;
            }
            else
            {
                endTransform *= tiltJointTransform;
            }

            // PrintMatrix(endTransform, "Pushing back matrix: ");

            Eigen::Matrix4d jointTransform = endTransform;

            jointTransforms.push_back(jointTransform);
        }

        // Modules are offset by a 45-degree rotation at their interface
        if (i != modules.size() - 1)
        {
            endTransform *= moduleInterfaceAngularOffset;
            // PrintMatrix(endTransform, "Rotated end transform");
        }
    }

    endTransform *= toolTransform;

    jointTransforms.push_back(endTransform);

    // PrintMatrix(endTransform, "End Transform");

    return jointTransforms;
}

Eigen::MatrixXd GetJacobian(const TransformVector& jointTransforms, const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    std::vector<ManipulatorModule> modules {proximal, distal};

    Eigen::Vector3d xAxis;
    xAxis << 1, 0, 0;

    Eigen::Vector3d yAxis;
    yAxis << 0, 1, 0;

    // The final Jacobian will have 6 rows and 2 * {num. modules} columns
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 2 * modules.size());

    Eigen::VectorXd jointJacobianColumn(6);
    Eigen::Vector3d jointCrossProduct;

    Eigen::VectorXd modulePanJacobianColumn;
    Eigen::VectorXd moduleTiltJacobianColumn;

    Eigen::Vector3d endPosition = GetTransformPosition(jointTransforms.back());

    // The number of joint transforms applied to the Jacobian
    int numJointTransformsProcessed = 0;

    for (int i = 0; i < modules.size(); i++)
    {
        modulePanJacobianColumn = Eigen::VectorXd::Zero(6);
        moduleTiltJacobianColumn = Eigen::VectorXd::Zero(6);

        for (int j = 0; j < modules[i].GetNumJoints(); j++)
        {
            Eigen::Matrix4d jointTransform = jointTransforms[numJointTransformsProcessed + j];
            // PrintMatrix(jointTransform, "Joint Transform");
            // Current joint is a pan-type if the first joint in the module is pan and the joint index is even, or if the inverse is true
            if (modules[i].IsFirstJointPan() ^ (j % 2 != 0))
            {
                Eigen::Vector3d actuationAxis = GetTransformRotation(jointTransform) * xAxis;
                jointCrossProduct = actuationAxis.cross(endPosition - GetTransformPosition(jointTransform));

                // PrintMatrix(jointCrossProduct, "Joint cross product");

                jointJacobianColumn << jointCrossProduct, actuationAxis;

                modulePanJacobianColumn += jointJacobianColumn;
            }
            else
            {
                Eigen::Vector3d actuationAxis = GetTransformRotation(jointTransform) * yAxis;
                jointCrossProduct = actuationAxis.cross(endPosition - GetTransformPosition(jointTransform));

                // PrintMatrix(jointCrossProduct, "Joint cross product");

                jointJacobianColumn << jointCrossProduct, actuationAxis;

                moduleTiltJacobianColumn += jointJacobianColumn;
            }
        }

        modulePanJacobianColumn /= modules[i].GetNumPanJoints();
        moduleTiltJacobianColumn /= modules[i].GetNumTiltJoints();

        // PrintMatrix(modulePanJacobianColumn, "Pan Jacobian column post-division");
        // PrintMatrix(moduleTiltJacobianColumn, "Tilt Jacobian column post-division");

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

        numJointTransformsProcessed += modules[i].GetNumJoints();
    }

    return jacobian;
}

// Obtain an approximation of the inverse Jacobian using the damped least squares method
Eigen::MatrixXd DampedLeastSquares(Eigen::MatrixXd jacobian, double leastSquaresDampingFactor)
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

// Obtain an approximation of the inverse Jacobian using the damped least squares method
Eigen::MatrixXd GetInverseJacobian(Eigen::MatrixXd& jacobian, double leastSquaresDampingFactor)
{
    Eigen::MatrixXd inverseJacobian(jacobian.cols(), jacobian.rows());

    inverseJacobian = (jacobian.transpose() * jacobian + Eigen::Matrix4d::Identity()).colPivHouseholderQr().solve(jacobian.transpose());

    return inverseJacobian;
}

// Obtain an approximation of the inverse Jacobian using the damped least squares method
Eigen::MatrixXd GetInverseJacobianDamped(
    Eigen::MatrixXd& jacobian,
    const Eigen::Vector4d& jointPositions,
    const Eigen::Vector4d& minJointAngles,
    const Eigen::Vector4d& maxJointAngles,
    double leastSquaresDampingFactor
)
{
    Eigen::Matrix4d D = Eigen::Matrix4d::Identity();

    double minAngle, maxAngle;

    for (int i = 0; i < jointPositions.size(); i++)
    {
        minAngle = minJointAngles(i);
        maxAngle = maxJointAngles(i);

        D(i, i) = pow((2 * jointPositions(i) - maxAngle - minAngle) / (maxAngle - minAngle), 2) + 1;
    }

    Eigen::MatrixXd inverseJacobian(jacobian.cols(), jacobian.rows());

    inverseJacobian = (jacobian.transpose() * jacobian + D.pow(2)).colPivHouseholderQr().solve(jacobian.transpose());

    return inverseJacobian;
}

Eigen::Vector4d GetMotorPositionsFromJointPositions(const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    Eigen::Vector4d motorPositions = Eigen::Vector4d::Zero();

    double test = 2 * proximal.GetCurvatureRadius() * (
        proximal.GetNumPanJoints() * (cos(proximal.GetHalfCurvatureAngle()) - cos(proximal.GetHalfCurvatureAngle() + (-1 * (int)true) * proximal.GetPanJointAngle() / 2)) +
        proximal.GetNumTiltJoints() * (1 - cos(proximal.GetTiltJointAngle() / 2))
    );

    // ROS_INFO("test: %.6f", test);

    double proximalPanLengthDelta = proximal.GetIsolatedPanLengthDelta();
    double proximalTiltLengthDelta = proximal.GetIsolatedTiltLengthDelta();

    // ROS_INFO("prox. pan: %.6f, prox. tilt: %.6f", proximalPanLengthDelta, proximalTiltLengthDelta);

    motorPositions(0) = proximalPanLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    motorPositions(1) = proximalTiltLengthDelta * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    double proximalCurvatureRadius = proximal.GetCurvatureRadius();
    double proximalHalfCurvatureAngle = proximal.GetHalfCurvatureAngle();

    // The difference in tendon length change resulting from the angular offset of the distal tendons on the proximal joints
    double distalTendonProximalJointOffset = (2 - sqrt(2)) * proximal.GetCurvatureRadius() * sin(proximal.GetHalfCurvatureAngle()) * (
        proximal.GetNumPanJoints() * sin(proximal.GetPanJointAngle() / 2) +
        proximal.GetNumTiltJoints() * sin(proximal.GetTiltJointAngle() / 2)
    );

    // The contribution of the proximal joint positions to the length delta of the distal tendons
    double distalLengthDeltaDueToProximalJoints = proximalPanLengthDelta + proximalTiltLengthDelta + distalTendonProximalJointOffset;

    // ROS_INFO("dLengthDelta: %.6f", distalLengthDeltaDueToProximalJoints);

    motorPositions(2) = (distal.GetIsolatedPanLengthDelta() + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;
    motorPositions(3) = (distal.GetIsolatedTiltLengthDelta() + distalLengthDeltaDueToProximalJoints) * LENGTH_DELTA_TO_PULLEY_ROTATION_DEGREES;

    // ROS_INFO("dist. pan: %.6f, dist. tilt: %.6f", motorPositions(0), motorPositions(1));

    return motorPositions;
}

Eigen::Vector4d GetJointPositionsFromMotorPositions(const Eigen::Vector4d& motorStates, const ManipulatorModule& proximal, const ManipulatorModule& distal)
{
    Eigen::Vector4d jointPositions = Eigen::Vector4d::Zero();

    // Ratio of proximal curvature radius to pulley radius
    double proximalRadiusRatio = proximal.GetCurvatureRadius() / PULLEY_WHEEL_RADIUS;
    double proximalHalfPanJointAngle = proximal.GetPanJointAngle() / 2;
    double proximalHalfTiltJointAngle = proximal.GetTiltJointAngle() / 2;

    // Construct proximal actuator Jacobian
    Eigen::Matrix2d proximalJacobian;
    proximalJacobian << -proximalRadiusRatio * sin(proximal.GetHalfCurvatureAngle() - proximalHalfPanJointAngle), proximalRadiusRatio * sin(proximalHalfTiltJointAngle),
                         proximalRadiusRatio * sin(proximalHalfPanJointAngle), -proximalRadiusRatio * sin(proximal.GetHalfCurvatureAngle() - proximalHalfTiltJointAngle);

    // PrintMatrix(proximalJacobian, "Proximal Actuator Jacobian");

    double distalRadiusRatio = distal.GetCurvatureRadius() / PULLEY_WHEEL_RADIUS;
    double distalHalfPanJointAngle = distal.GetPanJointAngle() / 2;
    double distalHalfTiltJointAngle = distal.GetTiltJointAngle() / 2;

    // Constant coefficient of partial derivatives of distal tendon lengths w.r.t. proximal joint angles
    double derivativeCoefficient = 1 - 1 / sqrt(2);

    // Construct distal actuator Jacobian
    Eigen::MatrixXd distalJacobian(2, 4);
    distalJacobian <<  proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfPanJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfTiltJointAngle),
                       -distalRadiusRatio * sin(distal.GetHalfCurvatureAngle() - distalHalfPanJointAngle),
                       distalRadiusRatio * sin(distalHalfTiltJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfPanJointAngle),
                       proximalRadiusRatio * derivativeCoefficient * sin(proximal.GetHalfCurvatureAngle()) * cos(proximalHalfTiltJointAngle),
                       distalRadiusRatio * sin(distalHalfPanJointAngle),
                       -distalRadiusRatio * sin(distal.GetHalfCurvatureAngle() - distalHalfTiltJointAngle);

    // PrintMatrix(distalJacobian, "Distal Actuator Jacobian");

    // Combine proximal and distal Jacobians. Note that the distal motor angles have no effect on the proximal joint positions
    Eigen::Matrix4d combinedJacobian = Eigen::Matrix4d::Zero();
    combinedJacobian.block(0, 0, 2, 2) = proximalJacobian;
    combinedJacobian.block(2, 0, 2, 4) = distalJacobian;

    // PrintMatrix(combinedJacobian, "Combined Jacobian");

    jointPositions = combinedJacobian * motorStates;

    return jointPositions;
}

Eigen::Vector3d SkewSymmetricMatrixToVector(const Eigen::Matrix3d& matrix)
{
    // Creates a vector v from a skew-symmetric matrix, assuming the following matrix layout:
    // |   0 -vz  vy |
    // |  vz   0 -vx |
    // | -vy  vx   0 |
    Eigen::Vector3d vector;
    vector << -matrix(1, 2), matrix(0, 2), -matrix(0, 1);
    return vector;
}

Eigen::VectorXd GetPoseDelta(const Eigen::Vector3d& currentPosition, const Eigen::Vector3d& desiredPosition, const Eigen::Matrix3d& currentRotation, const Eigen::Matrix3d& desiredRotation)
{
    Eigen::VectorXd poseDelta(6);
    poseDelta << (desiredPosition - currentPosition), SkewSymmetricMatrixToVector(desiredRotation * currentRotation.inverse() - Eigen::Matrix3d::Identity());

    return poseDelta;
}

Eigen::VectorXd CapVectorMagnitude(const Eigen::VectorXd& vector, const double maxMagnitude)
{
    Eigen::VectorXd cappedVector = maxMagnitude * vector.normalized();

    return cappedVector;
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
    Pose& currentTouchPose,
    Pose& touchOriginPose
)
{
    // Update control origin poses if white button was just pressed
    if (omniButtonStates->grey_button && !isGreyButtonPressed)
    {
        touchOriginPose.position = currentTouchPose.position;
        touchOriginPose.orientation = currentTouchPose.orientation;
    }

    isGreyButtonPressed = (bool)omniButtonStates->grey_button;
    isWhiteButtonPressed = (bool)omniButtonStates->white_button;
}

tf2::Quaternion EigenRotationMatrixToTF2Quaternion(const Eigen::Matrix3d& rotation)
{
    Eigen::Quaterniond eigenQuat(rotation);
    tf2::Quaternion tf2Quat = tf2::Quaternion(eigenQuat.x(), eigenQuat.y(), eigenQuat.z(), eigenQuat.w());

    return tf2Quat;
}

Eigen::Matrix3d tf2QuaternionToEigenRotationMatrix(const tf2::Quaternion& quaternion)
{
    Eigen::Quaterniond eigenQuat(quaternion.getW(), quaternion.getX(), quaternion.getY(), quaternion.getZ());

    Eigen::Matrix3d rotationMatrix = eigenQuat.toRotationMatrix();

    return rotationMatrix;
}

void motorStatesCallback(const motor_angles_msg::MotorAngles::ConstPtr& motorStates, Eigen::VectorXd& motorStatesCache, Eigen::VectorXd& motorCommands, bool& hasReceivedMotorStates)
{
    if (!hasReceivedMotorStates)
    {
        motorCommands = motorStatesCache;
    }

    hasReceivedMotorStates = true;
    motorStatesCache << motorStates->proximal_pan_angle, motorStates->proximal_tilt_angle, motorStates->distal_pan_angle, motorStates->distal_tilt_angle;
}

int main(int argc, char **argv)
{
    Pose touchOriginPose;
    Eigen::Matrix3d endOriginRotation;

    Eigen::Matrix3d endRotation;
    Pose currentTouchPose;
    geometry_msgs::PoseStamped currentUR5EPoseStamped;

    Pose touchPoseDelta;

    ros::init(argc, argv, "micro_module_control");
    ros::NodeHandle nodeHandle;

    ros::Subscriber touchStateSubscriber = nodeHandle.subscribe<omni_msgs::OmniState>(
        "/phantom/state",
        1,
        boost::bind(
            &touchStateCallback,
            _1,
            boost::ref(currentTouchPose)
        )
    );

    bool isGreyButtonPressed = false;
    bool isWhiteButtonPressed = false;

    bool hasReceivedMotorStates = false;

    Eigen::VectorXd motorStates(4);
    Eigen::VectorXd motorCommands(4);

    ros::Subscriber touchButtonSubscriber = nodeHandle.subscribe<omni_msgs::OmniButtonEvent>(
        "/phantom/button",
        1,
        boost::bind(
            &touchButtonCallback,
            _1,
            boost::ref(isGreyButtonPressed),
            boost::ref(isWhiteButtonPressed),
            boost::ref(currentTouchPose),
            boost::ref(touchOriginPose)
        )
    );

    ros::Subscriber motorStatesSubscriber = nodeHandle.subscribe<motor_angles_msg::MotorAngles>(
        "micro_module_motor_states",
        1,
        boost::bind(
            &motorStatesCallback,
            _1,
            boost::ref(motorStates),
            boost::ref(motorCommands),
            boost::ref(hasReceivedMotorStates)
        )
    );

    ros::Publisher motorCommandsPublisher = nodeHandle.advertise<motor_angles_msg::MotorAngles>("micro_module_motor_command", 1);

    ManipulatorModule proximal(
        true,
        MANIPULATOR_DIAMETER,
        PROXIMAL_HALF_CURVATURE_ANGLE,
        PROXIMAL_NUM_JOINTS,
        PROXIMAL_JOINT_SEPARATION
    );

    ManipulatorModule distal(
        true,
        MANIPULATOR_DIAMETER,
        DISTAL_HALF_CURVATURE_ANGLE,
        DISTAL_NUM_JOINTS,
        DISTAL_JOINT_SEPARATION
    );

    // Offset of end effector tip from centre of final manipulator joint
    Eigen::Vector3d endEffectorTipPositionOffset;
    endEffectorTipPositionOffset << 0, 0, END_EFFECTOR_TIP_OFFSET;

    Eigen::Matrix4d endEffectorTransform = TransformFromPosition(endEffectorTipPositionOffset);

    PrintMatrix(endEffectorTransform, "End Effector");

    proximal.SetTotalPanAngle(0.0);
    proximal.SetTotalTiltAngle(0.0);
    distal.SetTotalPanAngle(0.0);
    distal.SetTotalTiltAngle(0.0);

    ROS_INFO("Proximal parameters:\n\tcurvature radius: %.6f\n\tnum pan joints: %d\n\thalf curvature angle: %.6f\n\tpan joint angle: %.6f\n\tnum tilt joints: %d\n\ttilt joint angle: %.6f\n\tpan central separation: %.6f\n\ttilt central separation: %.6f\n\tjoint separation distance: %.6f\n\tisolated pan length delta: %.6f\n\tisolated tilt length delta: %.6f",
        proximal.GetCurvatureRadius(),
        proximal.GetNumPanJoints(),
        proximal.GetHalfCurvatureAngle(),
        proximal.GetPanJointAngle(),
        proximal.GetNumTiltJoints(),
        proximal.GetTiltJointAngle(),
        proximal.GetPanCentralSeparation(),
        proximal.GetTiltCentralSeparation(),
        proximal.GetJointSeparationDistance(),
        proximal.GetIsolatedPanLengthDelta(),
        proximal.GetIsolatedTiltLengthDelta()
    );

    ROS_INFO("Distal parameters:\n\tcurvature radius: %.6f\n\tnum pan joints: %d\n\thalf curvature angle: %.6f\n\tpan joint angle: %.6f\n\tnum tilt joints: %d\n\ttilt joint angle: %.6f\n\tpan central separation: %.6f\n\ttilt central separation: %.6f\n\tjoint separation distance: %.6f\n\tisolated pan length delta: %.6f\n\tisolated tilt length delta: %.6f",
        distal.GetCurvatureRadius(),
        distal.GetNumPanJoints(),
        distal.GetHalfCurvatureAngle(),
        distal.GetPanJointAngle(),
        distal.GetNumTiltJoints(),
        distal.GetTiltJointAngle(),
        distal.GetPanCentralSeparation(),
        distal.GetTiltCentralSeparation(),
        distal.GetJointSeparationDistance(),
        proximal.GetIsolatedPanLengthDelta(),
        proximal.GetIsolatedTiltLengthDelta()
    );

    Eigen::Matrix4d baseTransform = Eigen::Matrix4d::Identity();

    // Fixed transformation quaternion between touch and manipulator frames
    tf2::Quaternion touchToManipulatorRotation;
    touchToManipulatorRotation.setRPY(0, M_PI, 0);

    Eigen::Vector3d endPosition = Eigen::Vector3d::Zero();

    tf2::Quaternion manipulatorOrientation;
    tf2::Quaternion manipulatorOriginOrientation;
    tf2::Quaternion manipulatorOrientationDelta;

    tf2::Quaternion movementOrientationDelta;

    Eigen::Matrix3d movementRotation;

    Eigen::Vector4d minJointAngles;

    minJointAngles << -0.1, -0.1, -0.05, -0.05;

    Eigen::Vector4d maxJointAngles;

    maxJointAngles << 0.1, 0.1, -0.05, -0.05;

    Eigen::Vector4d jointPositions;
    jointPositions << proximal.GetPanJointAngle(), proximal.GetTiltJointAngle(), distal.GetPanJointAngle(), distal.GetTiltJointAngle();

    ros::Rate rate(2);

    while (ros::ok())
    {
        // Enable manipulator teleoperation if user is pressing the grey button on the Touch
        if (isGreyButtonPressed && hasReceivedMotorStates)
        {
            // Update Touch orientation delta (in manipulator coordinate frame)
            touchPoseDelta.orientation = (touchToManipulatorRotation * currentTouchPose.orientation) * (touchToManipulatorRotation * touchOriginPose.orientation).inverse();

            ROS_INFO_THROTTLE(1, "Touch orientation delta: %s", QuaternionToString(touchPoseDelta.orientation).c_str());

            ROS_INFO_THROTTLE(
                1,
                "Current Maniulator Orientation: %s, Manipulator origin orientation: %s",
                QuaternionToString(manipulatorOrientation).c_str(),
                QuaternionToString(manipulatorOriginOrientation).c_str()
            );

            manipulatorOrientation = EigenRotationMatrixToTF2Quaternion(endRotation);
            manipulatorOriginOrientation = EigenRotationMatrixToTF2Quaternion(endOriginRotation);
            manipulatorOrientationDelta = manipulatorOrientation * manipulatorOriginOrientation.inverse();

            ROS_INFO_THROTTLE(1, "Manipulator orientation delta: %s", QuaternionToString(manipulatorOrientationDelta).c_str());

            // Final movement orientation delta is difference between Touch and manipulator orientation deltas
            movementOrientationDelta = touchPoseDelta.orientation * manipulatorOrientationDelta.inverse();

            movementRotation = tf2QuaternionToEigenRotationMatrix(movementOrientationDelta);

            TransformVector jointTransforms = GetJointTransforms(baseTransform, endEffectorTransform, proximal, distal);

            endPosition = GetTransformPosition(jointTransforms.back());
            endRotation = GetTransformRotation(jointTransforms.back());

            Eigen::MatrixXd jacobian = GetJacobian(jointTransforms, proximal, distal);

            // PrintMatrix(jacobian, "Jacobian");

            jointPositions << proximal.GetPanJointAngle(), proximal.GetTiltJointAngle(), distal.GetPanJointAngle(), distal.GetTiltJointAngle();

            Eigen::MatrixXd inverseJacobian = GetInverseJacobianDamped(jacobian, jointPositions, minJointAngles, maxJointAngles, LEAST_SQUARES_DAMPING_FACTOR);

            // PrintMatrix(inverseJacobian, "Inverse Jacobian");

            Eigen::Vector3d desiredPos = endPosition;

            Eigen::Matrix3d desiredRot = movementRotation * endRotation;

            Eigen::VectorXd poseDelta = GetPoseDelta(endPosition, desiredPos, endRotation, desiredRot);

            // PrintMatrix(poseDelta, "Pose delta");

            if (poseDelta.norm() > JOINT_UPDATE_MAX_VELOCITY)
            {
                poseDelta = CapVectorMagnitude(poseDelta, JOINT_UPDATE_MAX_VELOCITY).eval();
            }

            // PrintMatrix(poseDelta, "Capped pose delta");

            Eigen::Vector4d jointDeltas = inverseJacobian * poseDelta;

            // PrintMatrix(jointDeltas, "Final joint deltas");

            proximal.ApplyPanAngleDelta(jointDeltas(0));
            proximal.ApplyTiltAngleDelta(jointDeltas(1));

            distal.ApplyPanAngleDelta(jointDeltas(2));
            distal.ApplyTiltAngleDelta(jointDeltas(3));

            Eigen::Vector4d stateDelta = GetMotorPositionsFromJointPositions(proximal, distal);

            motorCommands += stateDelta;

            ROS_INFO("Sending motor commands: %.6f %.6f %.6f %.6f", motorCommands(0), motorCommands(1), motorCommands(2), motorCommands(3));

            motor_angles_msg::MotorAngles motorCommandsMsg;
            motorCommandsMsg.proximal_pan_angle = motorCommands(0);
            motorCommandsMsg.proximal_tilt_angle = motorCommands(1);
            motorCommandsMsg.distal_pan_angle = motorCommands(2);
            motorCommandsMsg.distal_tilt_angle = motorCommands(3);

            motorCommandsPublisher.publish(motorCommandsMsg);

            rate.sleep();
        }

        ros::spinOnce();
    }

    return 0;
}
