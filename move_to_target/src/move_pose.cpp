#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>  // For RPY to quaternion conversion
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("interactive_move_to_pose_rpy");
    auto const logger = rclcpp::get_logger("interactive_move_to_pose_rpy");

    // Set up MoveGroupInterface for the robot arm
    static const std::string PLANNING_GROUP = "mycobot_pro_600_arm";  // Replace with your planning group
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End-effector link: %s", move_group.getEndEffectorLink().c_str());

    while (rclcpp::ok()) {
        // Read target position and orientation from user
        double x, y, z, roll, pitch, yaw;
        std::cout << "Enter target position (x, y, z) and orientation (roll, pitch, yaw in radians), or type 'exit' to quit:\n";

        std::string input;
        std::cin >> input;

        if (input == "exit") {
            std::cout << "Exiting program.\n";
            break;
        }

        try {
            x = std::stod(input);
            std::cin >> y >> z >> roll >> pitch >> yaw;
        } catch (const std::exception& e) {
            std::cout << "Invalid input. Please enter valid numbers for x, y, z, roll, pitch, yaw or type 'exit' to quit.\n";
            continue;
        }

        // Convert RPY to quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);

        // Set target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = quaternion.x();
        target_pose.orientation.y = quaternion.y();
        target_pose.orientation.z = quaternion.z();
        target_pose.orientation.w = quaternion.w();

        move_group.setPoseTarget(target_pose);

        // Plan and execute the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(logger, "Plan successful. Executing...");
            move_group.execute(plan);
            std::cout << "Successfully moved to position (" << x << ", " << y << ", " << z 
                      << ") with orientation (roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << ")\n";
        } else {
            RCLCPP_WARN(logger, "Planning to target pose failed!");
            std::cout << "Failed to move to position (" << x << ", " << y << ", " << z 
                      << ") with orientation (roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << ")\n";
        }

        // Wait briefly before the next iteration
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
