from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='lane_info_extractor_node',
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        # Node(
        #     package='serial_communication_pkg',
        #     executable='serial_sender_node',
        #     name='serial_sender_node',
        #     output='screen'
        # ),
    ])