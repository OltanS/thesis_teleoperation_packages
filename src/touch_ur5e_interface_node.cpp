#include <math.h>

#include "ros/ros.h"
#include "omni_msgs/OmniButtonEvent.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "moveit/move_group_interface/move_group_interface.h"

constexpr char const* MY_NODE_NAME = "touch_ur5e_interface_node";
constexpr char const* BUTTON_TOPIC_NAME = "/phantom/button";
constexpr char const* TOUCH_POSE_TOPIC_NAME = "/phantom/pose";
const double MOVEMENT_SCALE = 2.0;
const std::vector<double> HOME{ 0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0 };

class TouchUR5eInterface
{
    geometry_msgs::TransformStamped prevUR5ePose;
    geometry_msgs::PoseStamped prevTouchPose;
    tf2_ros::Buffer tfBufferTool;
    tf2_ros::Buffer tfBufferTouchToUR5e;
    tf2_ros::TransformListener tfListenerTool;
    tf2_ros::TransformListener tfListenerTouchToUR5e;
    


    moveit::planning_interface::MoveGroupInterface UR5eArmInterface;
    moveit::planning_interface::MoveGroupInterface::Plan myPlanArm;

    bool movementActive;
    bool movingHome;

public:
    TouchUR5eInterface() : tfListenerTool(tfBufferTool), tfListenerTouchToUR5e(tfBufferTouchToUR5e), UR5eArmInterface("manipulator")
    {   
        this->prevUR5ePose = geometry_msgs::TransformStamped();
        this->prevTouchPose = geometry_msgs::PoseStamped();

        this->movementActive = false;
        this->movingHome = false;
        
        ROS_INFO("Available Planning Groups:");
        std::copy(UR5eArmInterface.getJointModelGroupNames().begin(), UR5eArmInterface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    }

    void buttonCallback(const omni_msgs::OmniButtonEvent::ConstPtr& data);
    void touchMovementCallback(const geometry_msgs::PoseStamped::ConstPtr& data);
};

void TouchUR5eInterface::buttonCallback(const omni_msgs::OmniButtonEvent::ConstPtr& data) {
    if (data->white_button) {
        this->movingHome = true;
        ROS_WARN_THROTTLE(1, "Moving to Home position. All other movement is disabled until this movement is complete!");
        UR5eArmInterface.setJointValueTarget(HOME);
        bool success = (UR5eArmInterface.plan(myPlanArm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("Home Reset", "Reset to Home position plan %s", success ? "CREATED" : "FAILED");
        UR5eArmInterface.move();
        ROS_INFO_NAMED("Home Reset", "Home reset complete!");
        this->movingHome = false;
    }
    
    auto prevActiveState = this->movementActive;
    this->movementActive = (data->grey_button) ? true : false;
    if (prevActiveState != movementActive) {
        ROS_INFO("Movement is now %s", data->grey_button ? "ACTIVE" : "DISABLED");
    }
}

void TouchUR5eInterface::touchMovementCallback(const geometry_msgs::PoseStamped::ConstPtr& data) {
    auto currTime = data->header.stamp;
    auto currPose = data->pose;
    auto prevTime = prevTouchPose.header.stamp;
    auto timeDiff = currTime - prevTime;

    auto prevTouchPosePosition = this->prevTouchPose.pose.position;

    // Update the old information with the new
    prevTouchPose = *data;
    ROS_INFO_THROTTLE(2, "FREQUENCY CHECK: Last callback was %.4fs ago.", timeDiff.nsec * pow(10, -9));

    if (this->movingHome) {
        ROS_WARN_THROTTLE(1, "Moving to Home position. All other movement is disabled until this movement is complete!");
        return;
    }

    if (timeDiff > ros::Duration(0.5)) {
        ROS_INFO("Time difference between last position and new one was too large, resetting previous position to current position.");
        return;
    }
    if (!this->movementActive) {
        ROS_INFO_THROTTLE(5, "Movement is not active, so touch movement callbacks are being ignored.");
        return;
    }

    /*
    Replaced by the MoveIt lookups.
    geometry_msgs::TransformStamped tooltipTransform;
    try {
        tooltipTransform = tfBufferTool.lookupTransform("base", "tool0", ros::Time(0));
        auto tr = tooltipTransform.transform.translation;
        ROS_INFO_THROTTLE(2, "Location of tool flange in base frame: [x:%.3f y:%.3f z:%.3f]", tr.x, tr.y, tr.z);
    }

    catch (tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
        ROS_WARN("Was unable to perform lookup for the position of the tool flange! Not attempting movement.");
        return;
    }
    */

    ROS_INFO_THROTTLE(0.5, "Atttempting movement now.");
    // Calculate difference in current Touch Pose and Last Touch Pose
    geometry_msgs::Point poseDiff;
    poseDiff.x = MOVEMENT_SCALE * (currPose.position.x - prevTouchPosePosition.x);
    poseDiff.y = MOVEMENT_SCALE * (currPose.position.y - prevTouchPosePosition.y);
    poseDiff.z = MOVEMENT_SCALE * (currPose.position.z - prevTouchPosePosition.z);
    ROS_INFO("Scaled difference in pose between previous and current Touch position: [x:%.5f y:%.5f z:%.5f]", poseDiff.x, poseDiff.y, poseDiff.z);
    // Convert to the UR5e frame using the calculated transformation matrix
    geometry_msgs::TransformStamped touchToUR5eTransform;
    touchToUR5eTransform = tfBufferTouchToUR5e.lookupTransform("world", "touchToUR5e", ros::Time(0));
    tf2::doTransform(poseDiff, poseDiff, touchToUR5eTransform);
    ROS_INFO("The difference after transformation [x:%.5f y:%.5f z:%.5f]", poseDiff.x, poseDiff.y, poseDiff.z);
    // Read current position of tool
    // TODO: Currently assuming pose w.r.t base frame, test assumption.
    geometry_msgs::PoseStamped currentUR5ePose;
    currentUR5ePose = UR5eArmInterface.getCurrentPose();
    ROS_INFO("Current UR5e tool pose: [x:%.5f y:%.5f z:%.5f]", currentUR5ePose.pose.position.x, currentUR5ePose.pose.position.y, currentUR5ePose.pose.position.z);
    // Set as a goal with MoveIt (with a timeout)
    geometry_msgs::Pose targetUR5ePose;
    // TODO: manipulate orientation too.
    targetUR5ePose.orientation = currentUR5ePose.pose.orientation;
    targetUR5ePose.position.x = currentUR5ePose.pose.position.x + poseDiff.x;
    targetUR5ePose.position.y = currentUR5ePose.pose.position.y + poseDiff.y;
    targetUR5ePose.position.z = currentUR5ePose.pose.position.z + poseDiff.z;
    ROS_INFO("Target UR5e tool pose: [x:%.5f y:%.5f z:%.5f]", targetUR5ePose.position.x, targetUR5ePose.position.y, targetUR5ePose.position.z);
    UR5eArmInterface.setPoseTarget(targetUR5ePose);
    bool success = (UR5eArmInterface.plan(myPlanArm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Movement", "Visualizing movement %s", success ? "SUCCESS" : "FAILED");
    UR5eArmInterface.move();
}

// TODO: Consider setting a rate and using a spinonce in a while loop for moderation
int main(int argc, char** argv)
{
    // Initalize the ROS node
    ROS_INFO("Initalizizing the ROS node.");
    ros::init(argc, argv, MY_NODE_NAME);
    ros::NodeHandle n;
    
    ROS_INFO("Starting EXPERIMENTAL async spinner.");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO("Initalizing the interface class.");
    TouchUR5eInterface t;

    ROS_INFO("Creating the button and touch movement Subscribers.");
    ros::Subscriber buttonSubscriber = n.subscribe(BUTTON_TOPIC_NAME, 1, &TouchUR5eInterface::buttonCallback, &t);
    ros::Subscriber touchMovementSubscriber = n.subscribe(TOUCH_POSE_TOPIC_NAME, 1, &TouchUR5eInterface::touchMovementCallback, &t);
    
    // set transform broadcaster from Touch to UR5e
    ROS_INFO("Creating static broadcaster from the Touch frame to UR5e frame.");
    static tf2_ros::StaticTransformBroadcaster touchToUR5e;
    geometry_msgs::TransformStamped static_transformStamped;

    // TODO Change how this works so it doesn't cause two separate trees
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "touchToUR5e";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    // fixed axis, 90 deg CCW around x+, 90 deg ccw around y+
    quat.setRPY(0, 0, M_PI_2);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    touchToUR5e.sendTransform(static_transformStamped);

    ROS_INFO("Initialization Complete!");
    // ros::spin();
    ros::waitForShutdown();
    return 0;
}