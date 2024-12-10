from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    my_node = Node(
        package='finder_bots',
        executable='finderBots'
    )

    ld.add_action(my_node)
   

    return ld
