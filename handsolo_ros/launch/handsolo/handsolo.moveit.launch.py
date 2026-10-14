import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('handsolo_description'),
            'xacro',
            'ld250_tm12x.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file('handsolo_ros', 'config/ld250_tm12x.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml('tm12x_moveit_config', 'config/kinematics.yaml')
    }

    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planning_request_adapters/AddTimeOptimalParameterization 
                                   default_planning_request_adapters/FixWorkspaceBounds 
                                   default_planning_request_adapters/FixStartStateBounds 
                                   default_planning_request_adapters/FixStartStateCollision 
                                   default_planning_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml('tm12x_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    controllers_yaml = load_yaml('tm12x_moveit_config', 'config/controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    joint_limits_yaml = {
        'robot_description_planning': load_yaml('tm12x_moveit_config', 'config/joint_limits.yaml')
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        run_move_group_node,
    ])

