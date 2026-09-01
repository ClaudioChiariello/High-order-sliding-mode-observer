import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression, TextSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction, Shutdown
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():

    nodes = []
    truck_description_pkg_share = get_package_share_directory('truck_description')
    xacro_file = os.path.join(truck_description_pkg_share, 'urdf', 'piramide_truck.xacro') # Ensure it uses xacro file
 
    control_mode = LaunchConfiguration("control_mode")
    driven_wheels = LaunchConfiguration("driven_wheels")
    control_algorithm = LaunchConfiguration("control_algorithm")
    app_point_offset = LaunchConfiguration("app_point_offset")
    steering_limit = LaunchConfiguration("steering_limit")
    # Parameter that use the xacro to decide if activate mujoco or gz as system interface
    gz_sim = LaunchConfiguration("gz_sim")

    # Run the xacro command to convert the xacro file into an urdf to pass then to robot_description
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_file,
        " ",
        "control_mode:=",
        control_mode,
        " use_pid:=false",
        " use_mjcf_from_topic:=false",
        " headless:=false",
        " driven_wheels:=",
        driven_wheels,
        " app_point_offset:=",
        app_point_offset,
        " steering_limit:=",
        steering_limit,
        " gz_sim:=",
        gz_sim,
    ])

    truck_control_pkg_share = get_package_share_directory('truck_control')
    controller_manager_parameters_file = PathJoinSubstitution([truck_control_pkg_share, "config", "truck8x8_controllers.yaml"])


    # Start gz-sim
    gz_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    gz_sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world_file'), ' -r'],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # ros2_control node with MuJoCo. Nota che lo stesso eseguibile del pacchetto controller_manager. Per gz sim devi lanciarlo da xml
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(robot_description_content, value_type=str),
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }],
        ),
    
    )

    nodes.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "robot_description",
                "-name", "truck",
                "-allow_renaming", "false",
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.32",
                "-Y", "0.0"
            ]
        )
    )

    nodes.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/world/Truck/control@ros_gz_interfaces/srv/ControlWorld'
            ],
            output='screen'
        )
    )
  
    controllers_to_spawn = ["joint_state_broadcaster", "traction_controller", "steering_controller"]

    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controller_manager_parameters_file],
                output="both",
            )
        )

    nodes.append(
        Node(
            package='truck_control',
            executable=['truck_', LaunchConfiguration('control_algorithm')],
            name=['truck_', LaunchConfiguration('control_algorithm')],
            output='screen',
            parameters=[{
                'driven_wheels': LaunchConfiguration('driven_wheels'),
                'rear_wheel_separation': LaunchConfiguration('rear_wheel_separation'),
                'front_wheel_separation': LaunchConfiguration('front_wheel_separation'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'front_wheel_base': LaunchConfiguration('front_wheel_base'),
                'central_wheel_base': LaunchConfiguration('central_wheel_base'),
                'rear_wheel_base': LaunchConfiguration('rear_wheel_base'),
                'steering_limit': LaunchConfiguration('steering_limit'),
                'app_point_offset': LaunchConfiguration('app_point_offset'),
                'rear_centerline_pos': LaunchConfiguration('rear_centerline_pos'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        )
    )

    return LaunchDescription([

        # Declare args
        #QUesto chiama il ros2_gz_control
        DeclareLaunchArgument("control_mode", default_value="control"),
        DeclareLaunchArgument("driven_wheels", default_value="8"),
        DeclareLaunchArgument("control_algorithm", default_value="kinematic_control"),
        DeclareLaunchArgument("app_point_offset", default_value="0.0"),
        DeclareLaunchArgument("steering_limit", default_value="0.52"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument('rear_wheel_separation', default_value='2.088'),
        DeclareLaunchArgument('front_wheel_separation', default_value='2.131'),
        DeclareLaunchArgument('wheel_radius', default_value='0.5645'),
        DeclareLaunchArgument('front_wheel_base', default_value='1.9'),
        DeclareLaunchArgument('central_wheel_base', default_value='3.75'),
        DeclareLaunchArgument('rear_wheel_base', default_value='1.382'),
        DeclareLaunchArgument('rear_centerline_pos', default_value='2.6815'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('gz_sim', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='empty.sdf'),
        gz_sim_include,
        *nodes        
    ]
)



