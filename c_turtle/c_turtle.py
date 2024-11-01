from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
import launch.conditions as conditions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare("c_turtle")

    world_path = LaunchConfiguration("world_path")
    update_rate = LaunchConfiguration("update_rate")
    step_size = LaunchConfiguration("step_size")
    show_viz = LaunchConfiguration("show_viz")
    viz_pub_rate = LaunchConfiguration("viz_pub_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot1_ns = LaunchConfiguration("robot1_ns")
    robot2_ns = LaunchConfiguration("robot2_ns")
    robot1_x = LaunchConfiguration("robot1_x")
    robot1_y = LaunchConfiguration("robot1_y")
    robot1_yaw = LaunchConfiguration("robot1_yaw")
    robot2_x = LaunchConfiguration("robot2_x")
    robot2_y = LaunchConfiguration("robot2_y")
    robot2_yaw = LaunchConfiguration("robot2_yaw")

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution([pkg_share, "c_world/world.yaml"]),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="100.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.01"),
            DeclareLaunchArgument(name="show_viz", default_value="true"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            DeclareLaunchArgument(name="use_sim_time", default_value="true"),
            DeclareLaunchArgument(name="robot1_ns", default_value="robot1"),
            DeclareLaunchArgument(name="robot2_ns", default_value="robot2"),
            DeclareLaunchArgument(name="robot1_x", default_value="0.0"),
            DeclareLaunchArgument(name="robot1_y", default_value="0.0"),
            DeclareLaunchArgument(name="robot1_yaw", default_value="0.0"),
            DeclareLaunchArgument(name="robot2_x", default_value="-10.0"),
            DeclareLaunchArgument(name="robot2_y", default_value="0.0"),
            DeclareLaunchArgument(name="robot2_yaw", default_value="0.0"),

            SetEnvironmentVariable(name="ROSCONSOLE_FORMAT", value="[${severity} ${time} ${logger}]: ${message}"),

            Node(
                name="flatland_server",
                package="flatland_server",
                executable="flatland_server",
                output="screen",
                parameters=[
                    {"world_path": world_path},
                    {"update_rate": update_rate},
                    {"step_size": step_size},
                    {"show_viz": show_viz},
                    {"viz_pub_rate": viz_pub_rate},
                    {"use_sim_time": use_sim_time},
                ],
            ),

            Node(
                name="c_turtle_1",
                namespace=robot1_ns,
                package="c_turtle",
                executable="c_turtle",
                output="screen",
                parameters=[
                    {"initial_pose_x": robot1_x},
                    {"initial_pose_y": robot1_y},
                    {"initial_pose_yaw": robot1_yaw},
                ],
            ),

            Node(
                name="c_turtle_2",
                namespace=robot2_ns,
                package="c_turtle",
                executable="c_turtle",
                output="screen",
                parameters=[
                    {"initial_pose_x": robot2_x},
                    {"initial_pose_y": robot2_y},
                    {"initial_pose_yaw": robot2_yaw},
                ],
            ),

            Node(
                name="tf",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),

            Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_share, "rviz/robot_navigation.rviz"])],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=conditions.IfCondition(show_viz),
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()