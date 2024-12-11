from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    # Robot description
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_' + TURTLEBOT3_MODEL,
        'model.sdf'
    )
    robot_desc = os.path.join( get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(robot_desc, 'r') as urdf_file:
        robot_desc_content = urdf_file.read()

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc_content, 'use_sim_time': use_sim_time}]
    )

    # Spawning multiple robots
    spawn_robot_nodes = []
    robot_pose = [(9.0, 28.0), (0.0, -1.0), (1.0, 20.0), (7.0, 2.0), (20.0, 3.0),
                  (23.0, -3.0), (3.0, 30.0), (20.0, 18.0), (22.0, 30.0), (10.0, -6.0),
                  (11.0, 12.0), (0.0, 12.0)
                  ]

    for i in range(len(robot_pose)):
        robot_name = f"robot_{i}"
        spawn_robot_nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', urdf_path,
                '-x', str(robot_pose[i][0]),
                '-y', str(robot_pose[i][1]),
                '-z', '0.01',
                '-robot_namespace', robot_name
            ],
            output='screen'
        ))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        *spawn_robot_nodes  # Add all robot spawn nodes to the LaunchDescription
    ])
