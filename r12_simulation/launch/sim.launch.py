import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('r12_description')
    pkg_moveit = get_package_share_directory('r12_moveit_config')

    xacro_file = os.path.join(pkg_description, 'urdf', 'r12.urdf')

    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': 'empty.sdf -v 4 -r',
            'on_exit_shutdown': 'true'
        }.items()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])},
        ]
    )
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=[ '-name', 'r12_arm', '-topic', 'robot_description', '-x', '1', '-y', '1'], 
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        parameters=[{"use_sim_time": True}],
    )
    spawn_controller_1 = Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_controller_joint_state_broadcaster",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )
    spawn_controller_2 = Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_controller_r12_arm_controller",
        arguments=["r12_arm_controller"],
        output="screen"
    )
    spawn_controller_3 = Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawn_controller_hand_controller",
        arguments=["hand_controller"],
        output="screen"
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument(
            'urdf_model',
            default_value=xacro_file,
            description='Full path to the Xacro file'
        ),
        ros_gz_bridge,
        moveit,
        rviz,
        spawn,
        robot_state_publisher,
        spawn_controller_1,
        spawn_controller_2,
        spawn_controller_3
    ])