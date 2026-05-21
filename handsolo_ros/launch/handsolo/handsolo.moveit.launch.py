import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
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
    moveit_config = get_moveit_config()

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # Virtual Hand Solo to Base Link  Static TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'virtual_hand_solo/base_link', 'base']
    )


    return LaunchDescription([
        run_move_group_node,
        static_tf_node
        ])

    