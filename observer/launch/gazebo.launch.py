import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    pkg_name = get_package_share_directory("observer")

    # Launch arguments
    world_file = os.path.join(pkg_name, "worlds", "truck_world.sdf")

    controller_manager_config_file = PathJoinSubstitution([
        FindPackageShare("truck_controller_pkg"),
        "config",
        "truck8x4_controllers.yaml"
    ])
    xacro_file = PathJoinSubstitution([
        FindPackageShare("piramide_bringup"),
        "urdf",
        "piramide_truck.xacro"
    ])

    control_mode = LaunchConfiguration("control_mode")
    driven_wheels = LaunchConfiguration("driven_wheels")
    control_algorithm = LaunchConfiguration("control_algorithm")
    app_point_offset = LaunchConfiguration("app_point_offset")
    steering_limit = LaunchConfiguration("steering_limit")
 

    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
 

    robot_description = Command(["xacro ", xacro_file,
        " control_mode:=", control_mode,
        " driven_wheels:=", driven_wheels,
        " app_point_offset:=", app_point_offset,
        " steering_limit:=", steering_limit
    ])

 

    return LaunchDescription([

        # Declare args
        #QUesto chiama il ros2_gz_control
        DeclareLaunchArgument("control_mode", default_value="control"),
        DeclareLaunchArgument("driven_wheels", default_value="8"),
        DeclareLaunchArgument("control_algorithm", default_value="kinematic_control"),
        DeclareLaunchArgument("app_point_offset", default_value="0.0"),
        DeclareLaunchArgument("steering_limit", default_value="0.52"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument('rear_wheel_separation', default_value='2.088'),
        DeclareLaunchArgument('front_wheel_separation', default_value='2.131'),
        DeclareLaunchArgument('wheel_radius', default_value='0.5645'),
        DeclareLaunchArgument('front_wheel_base', default_value='1.9'),
        DeclareLaunchArgument('central_wheel_base', default_value='3.75'),
        DeclareLaunchArgument('rear_wheel_base', default_value='1.382'),
        DeclareLaunchArgument('rear_centerline_pos', default_value='2.6815'),

        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }]
        ),
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [world_file, ' -r'],
                'on_exit_shutdown': 'True'
            }.items(),
        ),

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
        ),

        # # control mode
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
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/world/Truck/control@ros_gz_interfaces/srv/ControlWorld'
            ],
            # remappings=[
            #     ('/model/truck/odometry', '/odom')
            # ],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager',
                        '/controller_manager'
                    ],
                    output='screen'
                )
            ]
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["traction_controller",
                '--controller-manager',
                '/controller_manager'
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=["steering_controller",
                '--controller-manager',
                '/controller_manager'
            ],
            output='screen'
        ),

        Node(
            package='truck_control',
            executable='truck_kinematic_control',
            name='truck_kinematic_control',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True
                }
            ]
        ),
        # DeclareLaunchArgument('scale_tanh', default_value='10'),
        # DeclareLaunchArgument('des_vel_x', default_value='10'),
        # DeclareLaunchArgument('des_omega_z', default_value='0.6'),
        # Node(
        #     package='observer',
        #     executable='observer',
        #     name='HOSMO',
        #     output='screen',
        #     parameters=[
        #         {   
        #             "scale_tanh": LaunchConfiguration('scale_tanh'),
        #             "des_vel_x": LaunchConfiguration('des_vel_x'),
        #             "des_omega_z": LaunchConfiguration('des_omega_z'),
        #             'use_sim_time': True
        #         }
        #     ]
        # ),
    ]
)