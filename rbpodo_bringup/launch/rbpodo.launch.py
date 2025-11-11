import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    robot_ip_parameter_name = "robot_ip"
    model_id_parameter_name = "model_id"
    model_path_parameter_name = "model_path"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    cb_simulation_parameter_name = "cb_simulation"
    use_rviz_parameter_name = "use_rviz"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    model_id = LaunchConfiguration(model_id_parameter_name)
    model_path = LaunchConfiguration(model_path_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    cb_simulation = LaunchConfiguration(cb_simulation_parameter_name)

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            model_path,
            " cb_simulation:=",
            cb_simulation,
            " robot_ip:=",
            robot_ip,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
        ]
    )

    rviz_file = os.path.join(
        get_package_share_directory("rbpodo_description"), "rviz", "urdf.rviz"
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rbpodo_bringup"),
            "config",
            "controllers.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                cb_simulation_parameter_name,
                default_value="false",
                description="Select RB Control Box mode, Simulation or Real".format(
                    cb_simulation_parameter_name
                ),
            ),
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                default_value="10.0.2.7",
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                model_id_parameter_name,
                default_value="rb5_850e",
                description="Model ID for Rainbow Robotics Cobot",
            ),
            DeclareLaunchArgument(
                model_path_parameter_name,
                default_value=[
                    TextSubstitution(
                        text=os.path.join(
                            get_package_share_directory("rbpodo_description"),
                            "robots",
                            "",
                        )
                    ),
                    model_id,
                    TextSubstitution(text=".urdf.xacro"),
                ],
                description="Model path (xacro)",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
                description="Visualize the robot in Rviz",
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description="Use fake hardware",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description="Fake sensor commands. Only valid when '{}' is true".format(
                    use_fake_hardware_parameter_name
                ),
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_controllers],
                remappings=[
                    ("joint_states", "rbpodo/joint_states"),
                    ("~/robot_description", "/robot_description"),
                ],
                output="both",
                on_exit=Shutdown(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"source_list": ["rbpodo/joint_states"], "rate": 30}],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["position_controllers"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
