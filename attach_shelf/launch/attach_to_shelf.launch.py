from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the command-line arguments for the parameters
    return LaunchDescription([
        DeclareLaunchArgument('degrees', default_value='-90', description='Number of degrees for the rotation of the robot after stopping'),
        DeclareLaunchArgument('obstacle', default_value='0.3', description='Distance (in meters) to the obstacle at which the robot will stop'),
        DeclareLaunchArgument('final_approach', default_value='true', description='value of the request of the service, attach_to_shelf'),

        Node(
            package='attach_shelf',  # Replace with your package name
            executable='pre_approach_v2_node',   # Replace with your executable name
            name='pre_approach_v2_node',
            parameters=[
                {'degrees': LaunchConfiguration('degrees')},
                {'obstacle': LaunchConfiguration('obstacle')},
                {'final_approach': LaunchConfiguration('final_approach')},
            ]
        )
    ])
