#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>  // For sleep functionality

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "robot_direct_planner",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("robot_direct_planner");

    // Define MoveGroup interface for controlling the robot
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "mycobot_pro_600_arm");

    // Set the planning frame to 'env_world' 
    move_group_interface.setPoseReferenceFrame("env_world");

    move_group_interface.setStartStateToCurrentState();

    // Wait for the robot to be ready and set the planning time
    move_group_interface.setPlanningTime(20.0);

    // Set Position A based on user input
    geometry_msgs::msg::PoseStamped target_pose_a;
    target_pose_a.header.frame_id = "env_world";  // Specify the frame as 'env_world'
    std::cout << "Enter coordinates for Position A (x y z Orientation x y z w): ";
    std::cin >> target_pose_a.pose.position.x >> target_pose_a.pose.position.y >> target_pose_a.pose.position.z
             >> target_pose_a.pose.orientation.x >> target_pose_a.pose.orientation.y >> target_pose_a.pose.orientation.z >> target_pose_a.pose.orientation.w;

    // Move to Position A
    move_group_interface.setPoseTarget(target_pose_a);
    moveit::planning_interface::MoveGroupInterface::Plan plan_to_a;
    bool success_a = (move_group_interface.plan(plan_to_a) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success_a) {
        move_group_interface.move();
        std::cout << "Moved to Position A successfully.\n";
    } else {
        RCLCPP_ERROR(logger, "Failed to move to Position A.");
        return 1;
    }

    // Wait for a few seconds at Position A
    std::cout << "Waiting for 5 seconds at Position A...\n";
    auto joint_values = move_group_interface.getCurrentJointValues();
    std::cout << "Current Joint Values: ";
    // Printing joint values of the Robot at Position A
    for (const auto &value : joint_values) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
    rclcpp::sleep_for(std::chrono::seconds(5));  // Sleep for 5 seconds

    // Set Position B based on user input
    geometry_msgs::msg::PoseStamped target_pose_b;
    target_pose_b.header.frame_id = "env_world";
    std::cout << "Enter coordinates for Position B (x y z Orientation x y z w): ";
    std::cin >> target_pose_b.pose.position.x >> target_pose_b.pose.position.y >> target_pose_b.pose.position.z
             >> target_pose_b.pose.orientation.x >> target_pose_b.pose.orientation.y >> target_pose_b.pose.orientation.z >> target_pose_b.pose.orientation.w;

    // Move to Position B
    move_group_interface.setPoseTarget(target_pose_b);
    moveit::planning_interface::MoveGroupInterface::Plan plan_to_b;
    bool success_b = (move_group_interface.plan(plan_to_b) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success_b) {
        move_group_interface.move();
        std::cout << "Moved to Position B successfully.\n";
    } else {
        RCLCPP_ERROR(logger, "Failed to move to Position B.");
        return 1;
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
