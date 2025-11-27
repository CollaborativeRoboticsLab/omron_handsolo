import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Top-level launch args
    tm_use_simulation = LaunchConfiguration('tm_use_simulation')
    tm_robot_ip = LaunchConfiguration('tm_robot_ip')
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

    # Paths to included launch files (within moma_ros)
    moma_ros_share = get_package_share_directory('moma_ros')
    handsolo_ros_share = get_package_share_directory('handsolo_ros')
    
    # Reuse ld250_tm12x.hardware.launch.py launch file as everything is same.
    include_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moma_ros_share, 'launch', 'ld250_tm12x', 'ld250_tm12x.hardware.launch.py')
        ),
        launch_arguments={
            'tm_use_simulation': tm_use_simulation,
            'tm_robot_ip': tm_robot_ip,
        }.items(),
    )

    include_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(handsolo_ros_share, 'launch', 'handsolo', 'handsolo.moveit.launch.py')
        ),
        condition=IfCondition(use_moveit)
    )

    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(handsolo_ros_share, 'launch', 'handsolo', 'handsolo.nav2.launch.py')
        ),
        condition=IfCondition(use_nav2)
    )

    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(handsolo_ros_share, 'launch', 'handsolo', 'handsolo.rviz.launch.py')
        ),
        condition=IfCondition(use_rviz)
    )

    # Velocity filter node to prevent amr_base from moving forward.
    filter_node = Node(
        package='handsolo_filter',
        executable='velocity_filter',
        name='velocity_filter',
        output='screen'
    )

    return LaunchDescription([
        declare_tm_use_simulation,
        declare_tm_robot_ip,
        declare_use_rviz,
        declare_use_moveit,
        declare_use_nav2,
        include_hardware,
        include_moveit,
        include_nav2,
        include_rviz,
        filter_node
    ])