import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

    # Configure robot_description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('handsolo_description'),
            'xacro',
            'ld250_tm12x.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # SRDF Configuration
    robot_description_semantic_config = load_file('handsolo_ros', 'config/ld250_tm12x.srdf')
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
        name='rviz2_moveit',
        output='log',
        emulate_tty=True,
        arguments=['-d', moveit_rviz_config],
        condition=IfCondition(use_moveit),
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