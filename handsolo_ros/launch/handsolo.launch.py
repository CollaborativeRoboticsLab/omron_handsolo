import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _is_enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _create_optional_launches(context, handsolo_ros_share):
    actions = []

    use_arm_enabled = _is_enabled(context, 'use_arm')
    use_base_enabled = _is_enabled(context, 'use_base')
    use_moveit_enabled = _is_enabled(context, 'use_moveit')
    use_nav2_enabled = _is_enabled(context, 'use_nav2')
    use_rviz_enabled = _is_enabled(context, 'use_rviz')

    if use_arm_enabled and use_moveit_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(handsolo_ros_share, 'launch', 'handsolo', 'handsolo.moveit.launch.py')
                )
            )
        )

    if use_base_enabled and use_nav2_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(handsolo_ros_share, 'launch', 'handsolo', 'handsolo.nav2.launch.py')
                )
            )
        )

    if use_rviz_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(handsolo_ros_share, 'launch', 'handsolo', 'handsolo.rviz.launch.py')
                ),
                launch_arguments={
                    'use_moveit': 'true' if use_arm_enabled and use_moveit_enabled else 'false',
                    'use_nav2': 'true' if use_base_enabled and use_nav2_enabled else 'false',
                }.items(),
            )
        )

    if use_base_enabled:
        actions.append(
            Node(
                package='handsolo_filter',
                executable='velocity_filter',
                name='velocity_filter',
                output='screen'
            )
        )

    return actions


def generate_launch_description():
    # Top-level launch args
    tm_use_simulation = LaunchConfiguration('tm_use_simulation')
    tm_robot_ip = LaunchConfiguration('tm_robot_ip')
    use_arm = LaunchConfiguration('use_arm')
    use_base = LaunchConfiguration('use_base')
    use_rviz = LaunchConfiguration('use_rviz')
    use_moveit = LaunchConfiguration('use_moveit')
    use_nav2 = LaunchConfiguration('use_nav2')

    declare_tm_use_simulation = DeclareLaunchArgument(
        'tm_use_simulation',
        default_value='false',
        description='Forwarded to hardware bringup to run TM robot in simulation (true/false)'
    )

    declare_tm_robot_ip = DeclareLaunchArgument(
        'tm_robot_ip',
        default_value='192.168.1.2',
        description='Target robot IP address'
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

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz (true/false)'
    )

    declare_use_moveit = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Whether to start MoveIt (true/false)'
    )

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Whether to start Nav2 (true/false)'
    )

    # Paths to included launch files
    handsolo_ros_share = get_package_share_directory('handsolo_ros')
    
    include_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(handsolo_ros_share, 'launch', 'handsolo', 'handsolo.hardware.launch.py')
        ),
        launch_arguments={
            'tm_use_simulation': tm_use_simulation,
            'tm_robot_ip': tm_robot_ip,
            'use_arm': use_arm,
            'use_base': use_base,
        }.items(),
    )

    return LaunchDescription([
        declare_tm_use_simulation,
        declare_tm_robot_ip,
        declare_use_arm,
        declare_use_base,
        declare_use_rviz,
        declare_use_moveit,
        declare_use_nav2,
        include_hardware,
        OpaqueFunction(function=lambda context: _create_optional_launches(context, handsolo_ros_share))
    ])