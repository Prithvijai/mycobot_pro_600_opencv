#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit_ik",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit_ik");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "cobot_arm");

  // Function to set target pose with quaternion orientation
  auto setTargetPose = [](double x, double y, double z, double roll, double pitch, double yaw) {
    geometry_msgs::msg::Pose msg;
    
    // Create a quaternion from roll, pitch, yaw
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);  // Set rotation around x (roll), y (pitch), z (yaw)
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    
    return msg;
  };

  // Function to move to a target pose using IK
  auto moveToPoseUsingIK = [&move_group_interface, &logger](const geometry_msgs::msg::Pose& target_pose) {
    move_group_interface.setPoseTarget(target_pose);

    // Check if IK is successful and get joint solution
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(logger, "IK Solution found, planning trajectory...");
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "IK solution failed!");
    }
  };

  // Prompt user for Position A (First target pose)
  double x_a, y_a, z_a, roll_a, pitch_a, yaw_a;
  std::cout << "Enter Position A (x, y, z, roll, pitch, yaw): ";
  std::cin >> x_a >> y_a >> z_a >> roll_a >> pitch_a >> yaw_a;
  auto target_pose_A = setTargetPose(x_a, y_a, z_a, roll_a, pitch_a, yaw_a);

  // Move to Position A using IK
  moveToPoseUsingIK(target_pose_A);

  // Prompt user for Position B (Second target pose)
  double x_b, y_b, z_b, roll_b, pitch_b, yaw_b;
  std::cout << "Enter Position B (x, y, z, roll, pitch, yaw): ";
  std::cin >> x_b >> y_b >> z_b >> roll_b >> pitch_b >> yaw_b;
  auto target_pose_B = setTargetPose(x_b, y_b, z_b, roll_b, pitch_b, yaw_b);

  // Move to Position B using IK
  moveToPoseUsingIK(target_pose_B);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}





// #include <memory>
// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <tf2/LinearMath/Quaternion.h>

// int main(int argc, char * argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "cobot_arm");

//   // Function to set target pose with quaternion orientation
//   auto setTargetPose = [](double x, double y, double z, double roll, double pitch, double yaw) {
//     geometry_msgs::msg::Pose msg;
    
//     // Create a quaternion from roll, pitch, yaw
//     tf2::Quaternion q;
//     q.setRPY(roll, pitch, yaw);  // Set rotation around x (roll), y (pitch), z (yaw)
//     msg.orientation.x = q.x();
//     msg.orientation.y = q.y();
//     msg.orientation.z = q.z();
//     msg.orientation.w = q.w();
    
//     msg.position.x = x;
//     msg.position.y = y;
//     msg.position.z = z;
    
//     return msg;
//   };

//   // Prompt user for Position A (First target pose)
//   double x_a, y_a, z_a, roll_a, pitch_a, yaw_a;
//   std::cout << "Enter Position A (x, y, z, roll, pitch, yaw): ";
//   std::cin >> x_a >> y_a >> z_a >> roll_a >> pitch_a >> yaw_a;
//   auto target_pose_A = setTargetPose(x_a, y_a, z_a, roll_a, pitch_a, yaw_a);

//   // Set target pose for Position A
//   move_group_interface.setPoseTarget(target_pose_A);

//   // Plan and execute movement to Position A
//   auto const [success_A, plan_A] = [&move_group_interface] {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   if (success_A) {
//     RCLCPP_INFO(logger, "Moving to Position A...");
//     move_group_interface.execute(plan_A);
//   } else {
//     RCLCPP_ERROR(logger, "Planning failed for Position A!");
//     return 1; // Exit if planning failed for Position A
//   }

//   // Prompt user for Position B (Second target pose)
//   double x_b, y_b, z_b, roll_b, pitch_b, yaw_b;
//   std::cout << "Enter Position B (x, y, z, roll, pitch, yaw): ";
//   std::cin >> x_b >> y_b >> z_b >> roll_b >> pitch_b >> yaw_b;
//   auto target_pose_B = setTargetPose(x_b, y_b, z_b, roll_b, pitch_b, yaw_b);

//   // Set target pose for Position B
//   move_group_interface.setPoseTarget(target_pose_B);

//   // Plan and execute movement to Position B
//   auto const [success_B, plan_B] = [&move_group_interface] {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   if (success_B) {
//     RCLCPP_INFO(logger, "Moving to Position B...");
//     move_group_interface.execute(plan_B);
//   } else {
//     RCLCPP_ERROR(logger, "Planning failed for Position B!");
//     return 1; // Exit if planning failed for Position B
//   }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }




