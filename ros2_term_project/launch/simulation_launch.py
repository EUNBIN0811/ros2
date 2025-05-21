from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_node = Node(
        package='ros2_term_project',
        namespace='',
        executable='cmd_publisher_node',
        
    )

    ld.add_action(controller_node)

    return ld
