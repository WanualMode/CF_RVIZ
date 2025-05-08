from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    manipulator_urdf_path = PathJoinSubstitution([
        FindPackageShare('ros_gz_example_description'),
        'models', 'manipulator', 'model.urdf'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='manipulator_state_publisher',
            parameters=[{
                'robot_description': Command(['cat ', manipulator_urdf_path])
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])

