import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'image_rosbag',
            default_value='',
            description='Path to the rosbag file containing image data'
        ),
        OpaqueFunction(function=lambda context: launch_setup(context)),
        launch_ros.actions.Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            arguments=['--ros-args', '--log-level', 'WARN'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='relbot_vision',
            executable='ball_detector',
            parameters=[
                {'image_topic': '/output/moving_camera'},
                {'method': 'BLOB'},
                {'hue': 20.0},
                {'debug': True}
            ],
            arguments=['--ros-args', '--log-level', 'WARN'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='relbot_control',
            executable='velocity_controller',
            parameters=[
                {'target_size': 100.0},
                {'target_position': 80.0},
                {'linear_gain': 0.001},
                {'angular_gain': 0.01},
                {'debug': True}
            ],
            arguments=['--ros-args', '--log-level', 'INFO'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='relbot_control',
            executable='differential_controller',
            arguments=['--ros-args', '--log-level', 'WARN'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='image_tools',
            executable='showimage',
            remappings=[
                ('/image', '/velocity_controller/debug_image')
            ],
            arguments=['--ros-args', '--log-level', 'WARN'],
            output='log'
        )
    ])

def launch_setup(context):
    image_rosbag = LaunchConfiguration('image_rosbag').perform(context)
    actions = []
    if image_rosbag:
        actions.append(
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', image_rosbag, '--loop'],
                output='log'
            )
        )
    return actions