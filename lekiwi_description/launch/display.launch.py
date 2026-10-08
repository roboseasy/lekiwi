"""Display the classroom model with synthetic joint states, without hardware."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("lekiwi_description")
    model = PathJoinSubstitution([description_share, "urdf", "lekiwi.urdf.xacro"])
    rviz_config = PathJoinSubstitution([description_share, "rviz", "model.rviz"])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model]), value_type=str
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="Model preview closed"))],
        )),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            # Matches classroom teleop_bridge.DEFAULT_ARM_OFFSETS_DEG.
            parameters=[{"rate": 30, "zeros.wrist_roll": -1.5707963267948966}],
            output="screen",
        ),
        rviz,
    ])
