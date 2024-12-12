#include <memory>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath> // For the conversion from radians to degrees
#include <chrono> // For generating a timestamp
#include <filesystem>

namespace fs = std::filesystem;

class ReadJointStatesNode : public rclcpp::Node {
public:
    ReadJointStatesNode() : Node("read_jointstates_node"), waypoint_reached_(false) {
        // Subscriber to listen for the waypoint reached message
        waypoint_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "waypoint_reached", 10, std::bind(&ReadJointStatesNode::waypointCallback, this, std::placeholders::_1)
        );

        // Subscriber to read joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ReadJointStatesNode::jointStateCallback, this, std::placeholders::_1)
        );
        
        // Ensure the joint_states folder exists
        std::string directory_path = "/home/rakshith/mycobot_pro_600_opencv/Joint_States";
        if (!fs::exists(directory_path)) {
            fs::create_directory(directory_path);  // Create the folder if it doesn't exist
        }

        // Create a unique file name with a timestamp
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *tm_now = std::localtime(&now_c);

        // Format the timestamp as YYYY-MM-DD_HH-MM-SS
        char filename[100];
        std::strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S", tm_now);
        
        // Construct the full file path
        std::string file_path = directory_path + "/joint_states_log_" + std::string(filename) + ".txt";

        // Open the file in append mode
        output_file_.open(file_path, std::ios::app);
        if (!output_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the file for writing joint states.");
        } else {
            RCLCPP_INFO(this->get_logger(), "File successfully opened for writing joint states: %s", file_path.c_str());
        }
    }

    ~ReadJointStatesNode() {
        if (output_file_.is_open()) {
            output_file_.close();
            RCLCPP_INFO(this->get_logger(), "File closed.");
        }
    }

private:
    bool waypoint_reached_;
    std::ofstream output_file_;

    void waypointCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !waypoint_reached_) {
            std::cout << "Waypoint reached, reading joint states...\n";
            waypoint_reached_ = true;  // Set the flag to true
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (waypoint_reached_ && !msg->position.empty()) {
            // Print joint positions and write them to the file in degrees
            std::cout << "Joint positions (degrees): ";
            for (const auto &position : msg->position) {
                double position_deg = position * (180.0 / M_PI); // Convert radians to degrees
                std::cout << position_deg << " ";
                
                // Write to file
                if (output_file_.is_open()) {
                    output_file_ << position_deg << " ";
                }
            }
            std::cout << std::endl;

            // Write a newline after each set of joint positions
            if (output_file_.is_open()) {
                output_file_ << std::endl;
            }

            // Reset the flag to false after reading joint states
            waypoint_reached_ = false;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_reached_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadJointStatesNode>());
    rclcpp::shutdown();
    return 0;
}
