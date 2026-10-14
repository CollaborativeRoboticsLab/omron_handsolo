import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_robot_description(package_name, file_path):
    absolute_file_path = os.path.join(get_package_share_directory(package_name), file_path)
    return xacro.process_file(absolute_file_path).toxml()


def _is_enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _create_hardware_actions(context, tm_robot_ip, tm_use_simulation, robot_description_override):
    actions = []

    use_arm_enabled = _is_enabled(context, 'use_arm')
    use_base_enabled = _is_enabled(context, 'use_base')
    override_enabled = robot_description_override.perform(context).lower() == 'true'

    if use_arm_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('tm_driver'), 'launch', 'tm_bringup.launch.py')
                ),
                launch_arguments={
                    'tm_robot_ip': tm_robot_ip,
                    'tm_use_simulation': tm_use_simulation,
                }.items(),
            )
        )

    if use_base_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('amr_ros'), 'launch', 'ld250.launch.py')
                ),
                launch_arguments={
                    'robot_description_override': robot_description_override,
                    'extra_params_file': os.path.join(
                        get_package_share_directory('handsolo_ros'), 'config', 'handsolo-ld250.yaml'
                    ),
                    'rviz': 'false',
                }.items(),
            )
        )

    if override_enabled:
        robot_description = {
            'robot_description': load_robot_description('handsolo_description', 'xacro/ld250_tm12x.urdf.xacro')
        }
        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='log',
                parameters=[robot_description],
            )
        )

    return actions


def generate_launch_description():
    tm_robot_ip = LaunchConfiguration('tm_robot_ip')
    tm_use_simulation = LaunchConfiguration('tm_use_simulation')
    use_arm = LaunchConfiguration('use_arm')
    use_base = LaunchConfiguration('use_base')
    robot_description_override = LaunchConfiguration('robot_description_override')

    declare_robot_ip = DeclareLaunchArgument(
        'tm_robot_ip',
        default_value='192.168.1.2',
        description='Target robot IP address'
    )

    declare_use_simulation = DeclareLaunchArgument(
        'tm_use_simulation',
        default_value='false',
        description='Use simulation mode (true/false)'
    )

    declare_use_arm = DeclareLaunchArgument(
        'use_arm',
        default_value='true',
        description='Whether to start the TM arm hardware stack (true/false)'
    )

    declare_use_base = DeclareLaunchArgument(
        'use_base',
        default_value='true',
        description='Whether to start the AMR base hardware stack (true/false)'
    )

    declare_robot_description_override = DeclareLaunchArgument(
        'robot_description_override',
        default_value='true',
        description='Whether to override robot_description defined in amr_core.launch.py (true/false)'
    )

    return LaunchDescription([
        declare_robot_ip,
        declare_use_simulation,
        declare_use_arm,
        declare_use_base,
        declare_robot_description_override,
        OpaqueFunction(
            function=lambda context: _create_hardware_actions(
                context,
                tm_robot_ip,
                tm_use_simulation,
                robot_description_override,
            )
        ),
    ])