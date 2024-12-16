import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach',
                    parameters=[{'use_sim_time': True}]),
                # ComposableNode(
                #     package='my_components',
                #     plugin='my_components::AttachServer',
                #     name='attach_server',
                #     parameters=[{'use_sim_time': True}]),
            ],
            output='screen',
            # Keep running even if the PreApproach finishes
            emulate_tty=True  # Keep container alive
    )

    # Manual Composition Node
    manual_server_node = Node(
        package='my_components',
        executable='manual_composition',
        name='attach_server',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return launch.LaunchDescription([container, manual_server_node])