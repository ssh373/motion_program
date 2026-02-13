from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('ns')
    model = LaunchConfiguration('model')
    device = LaunchConfiguration('device')
    rate = LaunchConfiguration('rate')
    clip_action = LaunchConfiguration('clip_action')
    max_delta = LaunchConfiguration('max_delta')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='robot1'),
        DeclareLaunchArgument(
            'model',
            default_value='/home/user/booster_deploy/tasks/locomotion/models/k1_walk.pt',
            description='Path to TorchScript model'
        ),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('rate', default_value='50.0'),
        DeclareLaunchArgument('clip_action', default_value='0.0'),
        DeclareLaunchArgument('max_delta', default_value='0.0'),

        #  BT → cmd_vel bridge
        Node(
            package='motion',
            executable='loco_bridge',
            namespace=ns,
            output='screen'
        ),

        # Walking policy node
        Node(
            package='motion',
            executable='walking_policy_node',
            namespace=ns,
            output='screen',
            arguments=[
                '--model', model,
                '--device', device,
                '--rate', rate,
                '--clip_action', clip_action,
                '--max_delta_per_step', max_delta,
            ]
        ),
    ])
