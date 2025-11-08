from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable


def generate_launch_description():
    # Args
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file = LaunchConfiguration('xacro_file', default='my_robot.urdf.xacro')

    # Rutas
    xacro_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        xacro_file
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'controllers.yaml'
    ])

    # robot_description desde xacro
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_path]),
        value_type=str
    )

    # Nodos
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    # ros2_control_node (controller_manager)
    # Pasamos explícitamente el parámetro 'robot_description' para que inicialice el ResourceManager.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_yaml]
    )

    # Spawners de controladores
    spawner_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawner_motor = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['motor_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('xacro_file', default_value='my_robot.urdf.xacro'),
        robot_state_publisher,
        ros2_control_node,
        spawner_broadcaster,
        spawner_motor,
    ])
