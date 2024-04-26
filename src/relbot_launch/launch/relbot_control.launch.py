import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    linear_gain_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        'linear_gain',
        default_value='0.01',
        description='Gain of size error to linear velocity'
    )
    angular_gain_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        'angular_gain',
        default_value='0.005',
        description='Gain of position error to angular velocity'
    )
    hue_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        'hue',
        default_value='150.0',
        description='Hue of the ball'
    )
    method_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        'method',
        default_value='NOCV',
        description='Detection method to detect the ball'
    )

    return launch.LaunchDescription([
        linear_gain_arg,
        angular_gain_arg,
        hue_arg,
        method_arg,
        launch_ros.actions.Node(
            package='image_tools',
            executable='cam2image',
            arguments=['--ros-args', '--log-level', 'WARN'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='relbot_vision',
            executable='ball_detector',
            parameters=[
                {'image_topic': '/image'},
                {'method': LaunchConfiguration('method')},
                {'hue': LaunchConfiguration('hue')},
                {'debug': True}
            ],
            arguments=['--ros-args', '--log-level', 'WARN'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='relbot_control',
            executable='velocity_controller',
            parameters=[
                {'target_size': 80.0},
                {'target_position': 160.0},
                {'linear_gain': LaunchConfiguration('linear_gain')},
                {'angular_gain': LaunchConfiguration('angular_gain')},
                {'debug': True}
            ],
            arguments=['--ros-args', '--log-level', 'INFO'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='relbot_control',
            executable='differential_controller',
            arguments=['--ros-args', '--log-level', 'WARN'],
            remappings=[
                ('/input/left_motor/setpoint_vel', '/left_motor/setpoint_vel'),
                ('/input/right_motor/setpoint_vel', '/right_motor/setpoint_vel')
            ],
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