#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "robot_cartesian_planner",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("robot_cartesian_planner");

    // Get user input for positions A and B, including orientations
    geometry_msgs::msg::Pose target_pose_a, target_pose_b;

    std::cout << "Enter coordinates for Position A (x y z): ";
    std::cin >> target_pose_a.position.x >> target_pose_a.position.y >> target_pose_a.position.z;

    std::cout << "Enter coordinates for Position B (x y z ): ";
    std::cin >> target_pose_b.position.x >> target_pose_b.position.y >> target_pose_b.position.z;

    // Define MoveGroup interface for controlling the robot
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "cobot_arm");

    // Ensure the robot's state is available before moving
    rclcpp::sleep_for(std::chrono::seconds(1));  // Wait for joint states to be published

    // Fetch the current robot state (including orientation)
    auto current_state = move_group_interface.getCurrentState();
    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to fetch the current robot state.");
        return 1;
    }

    // Use the current orientation as the target orientation (the robot's initial state)
    target_pose_a.orientation = current_state->getGlobalLinkTransform("link6").rotation(); // Assuming "link6" is the end-effector

    // Move to initial position (Point A)
    move_group_interface.setPoseTarget(target_pose_a);
    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;

    // Set a longer planning time (10 seconds)
    move_group_interface.setPlanningTime(10.0);
    bool success = (move_group_interface.plan(initial_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group_interface.move();
        std::cout << "Moved to Position A successfully.\n";
    } else {
        RCLCPP_ERROR(logger, "Failed to move to initial pose A.");
        return 1;
    }

    // Define waypoints for linear interpolation between A and B
    std::vector<geometry_msgs::msg::Pose> waypoints;
    int num_waypoints = 5;
    for (int i = 0; i <= num_waypoints; ++i) {
        geometry_msgs::msg::Pose waypoint;
        waypoint.position.x = target_pose_a.position.x + i * (target_pose_b.position.x - target_pose_a.position.x) / num_waypoints;
        waypoint.position.y = target_pose_a.position.y + i * (target_pose_b.position.y - target_pose_a.position.y) / num_waypoints;
        waypoint.position.z = target_pose_a.position.z + i * (target_pose_b.position.z - target_pose_a.position.z) / num_waypoints;

        // Keep the orientation constant as it is at Position A
        waypoint.orientation = target_pose_a.orientation;

        waypoints.push_back(waypoint);
    }

    // Plan a Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.02;                                             
    const double jump_threshold = 0.0;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // Execute the Cartesian path if planning is successful
    if (fraction > 0.95) {
        std::cout << "Executing Cartesian path to target pose B...\n";
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        move_group_interface.execute(cartesian_plan);
        std::cout << "Successfully moved to Position B.\n";
    } else {
        RCLCPP_ERROR(logger, "Failed to compute Cartesian path.");
    }

    // Clear path constraints after motion
    move_group_interface.clearPathConstraints();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
