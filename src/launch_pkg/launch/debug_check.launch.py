from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# ros2 launch launch_pkg debug_check.launch.py
# ros2 launch launch_pkg debug_check.launch.py cam_num:=0 logger:=True           # NOTE

def generate_launch_description():
    
    rqt_graph = ExecuteProcess(
    	cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
    	output="screen",
    )
    
    cam_num_launch_arg = DeclareLaunchArgument("cam_num",  default_value=TextSubstitution(text="2"))
    
    data_source_launch_arg = DeclareLaunchArgument("data_source", default_value=TextSubstitution(text="camera"))

    logger_launch_arg = DeclareLaunchArgument(              
        "logger", default_value=TextSubstitution(text="False")     
    )
        
    image_publisher_node = Node(
        package="camera_perception_pkg",
        executable="image_publisher_node",
        parameters=[{
            "cam_num": LaunchConfiguration('cam_num'), 
            "data_source": LaunchConfiguration('data_source'),
            "logger": LaunchConfiguration('logger'), 
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
            "logger": LaunchConfiguration('logger'), 
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
      
    return LaunchDescription([
        rqt_graph,
        
        cam_num_launch_arg,  
        data_source_launch_arg, 
        logger_launch_arg,
        
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