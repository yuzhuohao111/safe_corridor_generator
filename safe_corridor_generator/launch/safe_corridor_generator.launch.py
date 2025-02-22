from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='safe_corridor_generator',
            executable='safe_corridor_generator_node',
            name='safe_corridor_generator_node',
            namespace='',  # 添加 namespace 参数
            output='screen',
            emulate_tty=True,
        ),
    ])