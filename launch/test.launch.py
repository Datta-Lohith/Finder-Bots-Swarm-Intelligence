from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_catch_ros2 import Catch2IntegrationTestNode, Catch2LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the argument for test duration
    test_duration_arg = DeclareLaunchArgument(
        name='test_duration',
        default_value='2.0',
        description='Max length of test in seconds.'
    )
    
    finder_bots_node = Node(
        package='finder_bots',
        executable='finderBots',
        name='finderBots',
        output='screen',
    )

    # Node for the integration test using the TestNode class
    test_talker_node = Catch2IntegrationTestNode(
        package='finder_bots',
        executable='catch_test_finderBots',
        name='catch_test_finderBots',
        parameters=[{'test_duration': LaunchConfiguration('test_duration')}],
    )

    return Catch2LaunchDescription([
        test_duration_arg,
        finder_bots_node,
        test_talker_node,
    ])
