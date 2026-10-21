"""Run a tracker (mpc | pure_pursuit) + metrics logger. Start sim.launch.py first.

  ros2 launch mpc_diffdrive track.launch.py controller:=mpc path:=figure_eight
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("mpc_diffdrive"), "config", "mpc.yaml")
    path = LaunchConfiguration("path")
    ctrl = LaunchConfiguration("controller")
    logger = Node(package="mpc_diffdrive", executable="metrics_node", name="metrics_logger",
                  output="screen",
                  parameters=[cfg, {"path": path, "tag": ctrl,
                                    "out_dir": LaunchConfiguration("out_dir")}])
    return LaunchDescription([
        DeclareLaunchArgument("out_dir", default_value="/tmp/mpc_logs"),
        DeclareLaunchArgument("controller", default_value="mpc"),
        DeclareLaunchArgument("path", default_value="figure_eight"),
        Node(package="mpc_diffdrive", executable="mpc_node", name="mpc_controller",
             output="screen", parameters=[cfg, {"path": path}],
             condition=LaunchConfigurationEquals("controller", "mpc")),
        Node(package="mpc_diffdrive", executable="pure_pursuit_node", name="pure_pursuit",
             output="screen", parameters=[cfg, {"path": path}],
             condition=LaunchConfigurationEquals("controller", "pure_pursuit")),
        logger,
        # metrics node exits when the lap is done -> stop everything
        RegisterEventHandler(OnProcessExit(target_action=logger,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
