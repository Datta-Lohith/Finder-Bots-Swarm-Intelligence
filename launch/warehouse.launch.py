from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package share directory
    pkg_share = get_package_share_directory('finder_bots')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # TurtleBot3 model
    TURTLEBOT3_MODEL = 'waffle_pi'
    os.environ['TURTLEBOT3_MODEL'] = TURTLEBOT3_MODEL

    # World file
    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.world')

    # Gazebo model path
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path

    # Gazebo launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        gzserver_cmd,
        gzclient_cmd,
    ])
