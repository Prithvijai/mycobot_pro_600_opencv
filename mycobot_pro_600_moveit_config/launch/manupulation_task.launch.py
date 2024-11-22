import os
from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Set use_sim_time globally if running in simulation
    use_sim_time = SetLaunchConfiguration('use_sim_time', 'true')

    # Load MoveIt config for your robot
    moveit_config = (
        MoveItConfigsBuilder("mycobot_pro_600")
        .robot_description(file_path="config/mycobot_600.urdf.xacro")
        .robot_description_semantic(file_path="config/mycobot_600.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Start RViz with MoveIt configuration
    rviz_config_file = os.path.join(
        get_package_share_directory("mycobot_pro_600_moveit_config"),
        "launch/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Start robot_state_publisher to publish the robot's state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
        ],
    )

    # Start move_group for motion planning
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            "/home/rakshith/mycobot_pro_600_opencv/src/mycobot_pro_600_moveit_config/config/moveit_controllers.yaml"
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("mycobot_pro_600_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "mycobot_pro_600_arm",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Launch the nodes
    return LaunchDescription([
        use_sim_time,
        rviz_node,
        robot_state_publisher_node,
        move_group,
        ros2_control_node,
    ] + load_controllers)
