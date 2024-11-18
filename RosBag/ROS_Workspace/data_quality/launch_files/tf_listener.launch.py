import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_quality',
            executable='tf_listener',
            name='tf_listener',
            output='screen',
        ),
    ])