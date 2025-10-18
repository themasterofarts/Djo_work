from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration

def generate_launch_description():
    
    pkg_path = get_package_share_directory('my_pkg')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    
    pkg_gazebo= get_package_share_directory('ros_gz_sim')


    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo, 'launch', 'gz_sim.launch.py'),
    #     ),
    #     launch_arguments={'gz_args': [PathJoinSubstitution([
    #         pkg_path,
    #         'worlds',
    #         'djo_world.sdf'
    #     ]),
    #     TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
    #     #TextSubstitution(text=' -r -v -v1')],
    #     'on_exit_shutdown': 'true'}.items()
    #)


    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo, 'launch', 'gz_sim.launch.py'),
    #     ),
    #     launch_arguments={'gz_args': [PathJoinSubstitution([
    #         pkg_path,
    #         'worlds',
    #         'empty_gz.sdf'
    #     ]),
    #     TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
    #     'on_exit_shutdown': 'true'}.items()
    # )
    

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args":[ PathJoinSubstitution([pkg_path, "worlds", "djo_world.sdf"]),
             TextSubstitution(text=' -r -v -v1 --render-engine ogre')],

        }.items(),
    )
    
    

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),


        Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True,
             'camera.image.compressed.jpeg_quality': 75},
        ],
        ),
        
         
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     parameters=[{
        #         'config_file': os.path.join(pkg_path, 'configuration', 'bridge.yaml'),
        #         'qos_overrides./tf_static.publisher.durability': 'transient_local',
        #     }],
        #     output='screen'
        # ),

       ### noeud du pont entre gazebo et ros pour la communication #####
        Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            '/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "imu@sensor_msgs/msg/Imu@gz.msgs.IMU",

        ],
        output="screen",
        parameters=[{'use_sim_time': True    }    ]
        ),
        
        ##  Faire apparaitre mon robot dans gazebo   ######
        Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-name",
                    "bot",
                    "-allow_renaming",
                    "true",  # permet de faire le robot dans gz sim
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.1",
                    "-R",
                    "0.0",
                    "-P",
                    "0.0",
                ],
            ),
        gz_sim,
    ])
