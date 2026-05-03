from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. GÖZLER (Perception)
        Node(
            package='teknofest_ika',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),

        # 2. DUYULAR (Sensors)
        Node(
            package='teknofest_ika',
            executable='sensor_node',
            name='sensor_node',
            output='screen'
        ),

        # 3. KASLAR (Motors)
        Node(
            package='teknofest_ika',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),

        # 4. BEYİN (Brain)
        Node(
            package='teknofest_ika',
            executable='brain_node',
            name='brain_node',
            output='screen'
        ),


        #arayüz
        # Node(
        #     package='teknofest_ika',
        #     executable='gui_node',
        #     name='gui_node',
        #     output='screen'
        # ),
    ])