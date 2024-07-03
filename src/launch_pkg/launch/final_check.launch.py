from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# ros2 launch launch_pkg final_check.launch.py # NOTE

def generate_launch_description():
    
    rqt_graph = ExecuteProcess(
    	cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
    	output="screen",
    )
    
    image_publisher_node = Node(
        package="camera_perception_pkg",
        executable="image_publisher_node",
        parameters=[{
            "cam_num": 0,
            "data_source": "camera",
            "logger": False
        }]
    )
    
    yolov8_node = Node(
        package="camera_perception_pkg",
        executable="yolov8_node",
    )
    
    lane_info_extractor_node = Node(
        package="camera_perception_pkg",
        executable="lane_info_extractor_node",
        parameters=[{
            'logger': False
        }]
    )
    
    traffic_light_detector_node = Node(
        package="camera_perception_pkg",
        executable="traffic_light_detector_node",
    )
    
    debug_node = Node(
        package="camera_perception_pkg",
        executable="debug_node",
    )
    
    lidar_publisher_node = Node(
        package="lidar_perception_pkg",
        executable="lidar_pub_node",
    )
    
    lidar_processor_node = Node(
        package="lidar_perception_pkg",
        executable="lidar_processor_node",
    )
    
    lidar_obstacle_detector_node = Node(
        package="lidar_perception_pkg",
        executable="lidar_obstacle_detector_node",
    )
    
    motion_planner_node = Node(
        package="decision_making_pkg",
        executable="motion_planner_node",
    )
    
    serial_protocol_converter_node = Node(
        package="serial_communication_pkg",
        executable="serial_protocol_converter_node",
    )
    
    serial_sender_node = Node(
        package="serial_communication_pkg",
        executable="serial_sender_node",
    )
    
    return LaunchDescription([
        rqt_graph, 
        serial_sender_node,
        serial_protocol_converter_node,
        motion_planner_node,
        lidar_obstacle_detector_node,
        lidar_processor_node,
        lidar_publisher_node,
        debug_node,
        traffic_light_detector_node,
        lane_info_extractor_node,
        yolov8_node,
        image_publisher_node
        
    ])