import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    absolute_file_path = os.path.join(get_package_share_directory(package_name), file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def get_moveit_config():
    moveit_config = (
        MoveItConfigsBuilder('ld250_tm12x', package_name='tm12x_moveit_config')
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('handsolo_description'),
                'xacro',
                'ld250_tm12x.urdf.xacro',
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory('handsolo_ros'),
                'config',
                'ld250_tm12x.srdf',
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory('tm12x_moveit_config'),
                'config',
                'moveit2_controllers.yaml',
            )
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory('tm12x_moveit_config'),
                'config',
                'joint_limits.yaml',
            )
        )
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )
    moveit_config.planning_pipelines['ompl'] = load_yaml('tm12x_moveit_config', 'config/ompl_planning.yaml')
    return moveit_config

def generate_launch_description():
    use_moveit = LaunchConfiguration('use_moveit')
    use_nav2 = LaunchConfiguration('use_nav2')

    declare_use_moveit = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Whether to start RViz with the MoveIt configuration'
    )

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Whether to start RViz with the Nav2 configuration'
    )
    moveit_config = get_moveit_config()

    # RViz configurations
    moveit_rviz_config = PathJoinSubstitution([FindPackageShare('handsolo_ros'), 'rviz', 'handsolo-moveit.rviz'])
    nav2_rviz_cfg = PathJoinSubstitution([FindPackageShare('handsolo_ros'), 'rviz', 'handsolo-nav2.rviz'])

    # RViz
    moveit_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        emulate_tty=True,
        arguments=['-d', moveit_rviz_config],
        condition=IfCondition(use_moveit),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # RViz2
    nav2_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', nav2_rviz_cfg],
        condition=IfCondition(use_nav2),
        # parameters=[{'use_sim_time': use_sim_time}],
        output='log'
    )

    return LaunchDescription([
        declare_use_moveit,
        declare_use_nav2,
        moveit_rviz_node, 
        nav2_rviz_node
    ])