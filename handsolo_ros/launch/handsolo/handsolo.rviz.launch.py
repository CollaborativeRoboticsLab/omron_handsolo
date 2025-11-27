import os
import sys
import yaml
import json
import xacro
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Configure robot_description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('handsolo_description'),
            'xacro',
            'handsolo.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # SRDF Configuration
    robot_description_semantic_config = load_file('tm12x_moveit_config'  , 'config/tm12x.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization 
                                   default_planner_request_adapters/FixWorkspaceBounds 
                                   default_planner_request_adapters/FixStartStateBounds 
                                   default_planner_request_adapters/FixStartStateCollision 
                                   default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }

    # Kinematics
    kinematics_yaml = load_yaml('tm12x_moveit_config'  , 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # RViz configurations
    moveit_rviz_config = PathJoinSubstitution([FindPackageShare('handsolo_ros'), 'rviz', 'handsolo-moveit.rviz'])
    nav2_rviz_cfg = PathJoinSubstitution([FindPackageShare('handsolo_ros'), 'rviz', 'handsolo-nav2.rviz'])

    # Joint limits
    joint_limits_yaml = {'robot_description_planning': load_yaml('tm12x_moveit_config', 'config/joint_limits.yaml')}

    # RViz
    moveit_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        emulate_tty=True,
        arguments=['-d', moveit_rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
        ],
    )

    # RViz2
    nav2_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz_cfg],
        # parameters=[{'use_sim_time': use_sim_time}],
        output='log'
    )

    return LaunchDescription([
        moveit_rviz_node, 
        nav2_rviz_node
    ])