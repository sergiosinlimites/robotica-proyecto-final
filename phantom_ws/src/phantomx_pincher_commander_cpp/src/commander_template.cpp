#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <phantomx_pincher_interfaces/msg/pose_command.hpp>
#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
  #include <moveit/move_group_interface/move_group_interface.hpp>
#else
  #include <moveit/move_group_interface/move_group_interface.h>
#endif
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = phantomx_pincher_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        // Prefer elbow-up solutions (can be disabled via parameter)
        prefer_elbow_up_ = node_->declare_parameter<bool>("prefer_elbow_up", true);
        // HOME: prefer IK to the HOME pose; fallback to joints=0 only if enabled.
        home_joint_fallback_ = node_->declare_parameter<bool>("home_joint_fallback", true);
        // Use a reentrant callback group for general subscriptions
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        // Use a separate MutuallyExclusiveCallbackGroup for pose commands
        // This ensures only ONE pose command runs at a time (preventing race conditions)
        // but allows it to run in parallel with the default group (used by MoveGroup for joint_states)
        pose_cmd_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        auto pose_sub_opt = rclcpp::SubscriptionOptions();
        pose_sub_opt.callback_group = pose_cmd_group_;

        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        arm_->setPlanningTime(5.0); // Increase planning time
        arm_->setNumPlanningAttempts(10); // Increase attempts
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        RCLCPP_INFO(
            node_->get_logger(),
            "Arm planning frame: %s",
            arm_->getPlanningFrame().c_str()
        );
    
        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::openGripperCallback, this, _1), sub_opt);

        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1), sub_opt);

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1), pose_sub_opt);
    }

    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, 
                        double roll, double pitch, double yaw, bool cartesian_path=false)
    {
        const bool is_home_pose = (std::abs(x) < 0.05 && std::abs(y) < 0.05 && z > 0.35);

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = arm_->getPlanningFrame();  // <--- use MoveIt’s frame
        target_pose.header.stamp = node_->now();   
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if (!cartesian_path) {
            // Restore approximate IK logic which was removed by user
            // For 4-DOF robots, setPoseTarget is often too strict.
            arm_->setGoalPositionTolerance(0.001);
            arm_->setGoalOrientationTolerance(3.14159); // Allow any orientation

            // Soft preference: keep elbow >= 0 (avoid elbow-down)
            if (prefer_elbow_up_) {
                moveit_msgs::msg::Constraints constraints;
                moveit_msgs::msg::JointConstraint elbow_c;
                elbow_c.joint_name = "phantomx_pincher_arm_elbow_flex_joint";
                elbow_c.position = 1.2;          // center around ~69°
                elbow_c.tolerance_below = 1.2;   // min ~0
                elbow_c.tolerance_above = 2.0;   // max ~3.2 (covers up to pi)
                elbow_c.weight = 1.0;
                constraints.joint_constraints.push_back(elbow_c);
                arm_->setPathConstraints(constraints);
            }

            if (arm_->setApproximateJointValueTarget(target_pose, "")) {
                planAndExecute(arm_);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Could not find approximate IK solution for requested pose.");
                if (is_home_pose && home_joint_fallback_) {
                    RCLCPP_INFO(node_->get_logger(), "HOME fallback: moving joints to zeros.");
                    std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0};
                    goToJointTarget(home_joints);
                }
            }

            if (prefer_elbow_up_) {
                arm_->clearPathConstraints();
            }
        }
        else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            
            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;

            double fraction = arm_->computeCartesianPath(
                waypoints,
                0.01,          // eef_step
                0.0,           // jump_threshold
                trajectory,
                true,          // avoid_collisions
                &error_code    // optional error code output
            );

            if (fraction == 1) {
                arm_->execute(trajectory);
            }
        }
    }

    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }


private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            interface->execute(plan);
        }
    }

    void openGripperCallback(const Bool &msg)
    {
        if (msg.data) {
            openGripper();
        }
        else {
            closeGripper();
        }
    }

    void jointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;

        // accept [q1..q4] or [q1..q6] (we only use arm joints)
        if (joints.size() >= 4) {
            std::vector<double> arm_joints = {joints[0], joints[1], joints[2], joints[3]};
            goToJointTarget(arm_joints);
        }
    }

    void poseCmdCallback(const PoseCmd &msg)
    {
        goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);   
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    bool prefer_elbow_up_{true};
    bool home_joint_fallback_{true};

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr pose_cmd_group_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    
    // Use MultiThreadedExecutor to allow MoveGroupInterface to process callbacks 
    // (like joint_states) while we are blocking in a callback (like poseCmdCallback).
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    auto commander = std::make_shared<Commander>(node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}