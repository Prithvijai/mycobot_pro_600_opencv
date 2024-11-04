#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "moveit_example_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a logger
    auto logger = rclcpp::get_logger("moveit_example");

    // Create the MoveIt MoveGroup Interface for the robotic arm
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "cobot_arm");

    // Define the target positions (you can adjust these coordinates)
    std::vector<geometry_msgs::msg::Pose> target_poses(2);

    // Initial pose (if you want to set a specific starting pose)
    target_poses[0].position.x = 0.0;   // Set pos1 x
    target_poses[0].position.y = 0.18;  // Set pos1 y
    target_poses[0].position.z = 0.817; // Set pos1 z
    target_poses[0].orientation.w = 1.0; // Set orientation (quaternion)

    target_poses[1].position.x = 0.2;   // Set pos2 x
    target_poses[1].position.y = 0.0;    // Set pos2 y
    target_poses[1].position.z = 0.817;  // Set pos2 z
    target_poses[1].orientation.w = 1.0; // Set orientation (quaternion)

    // Move to the first target pose
    for (const auto &target_pose : target_poses) {
        move_group_interface.setPoseTarget(target_pose);

        // Plan and execute the movement
        auto [success, plan] = move_group_interface.plan();
        if (success) {
            move_group_interface.execute(plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed for the target pose!");
            return 1; // Exit if planning fails
        }

        // Print the current position
        auto current_pose = move_group_interface.getCurrentPose();
        printf("Current pose of the arm:\n");
        printf("Position - x: %f, y: %f, z: %f\n", 
               current_pose.pose.position.x, 
               current_pose.pose.position.y, 
               current_pose.pose.position.z);
        printf("Orientation - x: %f, y: %f, z: %f, w: %f\n", 
               current_pose.pose.orientation.x, 
               current_pose.pose.orientation.y, 
               current_pose.pose.orientation.z, 
               current_pose.pose.orientation.w);

        // Optionally, you can wait for a while before moving to the next position
        std::this_thread::sleep_for(2s); // Wait for 2 seconds
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
