#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/bool.hpp>
#include <chrono> // For sleep functionality

struct Waypoint {
    double x, y, z, roll, pitch, yaw;
};

std::vector<Waypoint> readWaypointsFromFile(const std::string &file_path) {
    std::vector<Waypoint> waypoints;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + file_path);
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines or lines with just whitespace
        if (line.empty() || line.find_first_not_of(" \t\n\r") == std::string::npos) {
            continue;
        }

        std::istringstream iss(line);
        Waypoint waypoint;
        // Check if the line has enough data to form a waypoint
        if (iss >> waypoint.x >> waypoint.y >> waypoint.z >> waypoint.roll >> waypoint.pitch >> waypoint.yaw) {
            waypoints.push_back(waypoint);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("robot_wavepoint_planner"), "Skipping invalid line: %s", line.c_str());
        }
    }

    file.close();
    return waypoints;
}

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "robot_wavepoint_planner",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("robot_wavepoint_planner");

    // Publisher to notify about the waypoint movement status
    auto publisher = node->create_publisher<std_msgs::msg::Bool>("waypoint_reached", 10);

    // Check if file path is provided as an argument
    if (argc < 2) {
        RCLCPP_ERROR(logger, "Usage: %s <file_path>", argv[0]);
        return 1;
    }

    std::string file_path = argv[1];

    // Read waypoints from the file
    std::vector<Waypoint> waypoints;
    try {
        waypoints = readWaypointsFromFile(file_path);
        RCLCPP_INFO(logger, "Successfully loaded %lu waypoints from file.", waypoints.size());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger, "Error reading file: %s", e.what());
        return 1;
    }

    // Define MoveGroup interface for controlling the robot
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "mycobot_pro_600_arm");

    // Set the planning frame to 'base'
    move_group_interface.setPoseReferenceFrame("base");

    move_group_interface.setStartStateToCurrentState();

    move_group_interface.setPlanningTime(20.0);
    // Define boundaries for the robot's workspace

    for (const auto &waypoint : waypoints) {
        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(waypoint.roll, waypoint.pitch, waypoint.yaw);

        // Create target pose
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base";
        target_pose.pose.position.x = waypoint.x;
        target_pose.pose.position.y = waypoint.y;
        target_pose.pose.position.z = waypoint.z;
        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        target_pose.pose.orientation.w = q.w();

        // Command robot to move to the waypoint
        move_group_interface.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_interface.move();
            std::cout << "Moved to waypoint (" << waypoint.x << ", " << waypoint.y << ", " << waypoint.z << ") successfully.\n";

            // Publish that the waypoint was reached
            std_msgs::msg::Bool msg;
            msg.data = true;
            publisher->publish(msg);
        } else {
            RCLCPP_ERROR(logger, "Failed to move to waypoint (%f, %f, %f). Stopping execution.", waypoint.x, waypoint.y, waypoint.z);
            break; // Exit the loop on failure
        }

        // Wait for 5 seconds between movements
        std::cout << "Waiting for 1 seconds...\n";
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}