"""Gazebo Fortress + diffbot + ros_gz bridge + robot_state_publisher (+ RViz)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("mpc_diffdrive")
    xacro_file = os.path.join(pkg, "urdf", "diffbot.urdf.xacro")
    world = os.path.join(pkg, "worlds", "empty.sdf")
    robot_desc = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [LaunchConfiguration("gz_extra"), f" -r {world}"]}.items())

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "diffbot", "-z", "0.08"])

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_desc, "use_sim_time": True}])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=[
                      "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                      "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                      "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                      "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                      "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                  ])

    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", os.path.join(pkg, "rviz", "mpc.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        # pass gz_extra:=-s to run Gazebo headless (no GUI)
        DeclareLaunchArgument("gz_extra", default_value=""),
        gz, rsp, spawn, bridge, rviz,
    ])
