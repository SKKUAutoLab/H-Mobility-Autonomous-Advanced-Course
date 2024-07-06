from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# ros2 launch launch_pkg camera_check.launch.py
# ros2 launch launch_pkg camera_check.launch.py cam_num:=0 logger:=True          # NOTE

def generate_launch_description():
    
    rqt_graph = ExecuteProcess(
    	cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
    	output="screen",
    )
    
    cam_num_launch_arg = DeclareLaunchArgument(              
        "cam_num", default_value=TextSubstitution(text="2")     
    )
    data_source_launch_arg = DeclareLaunchArgument("data_source", default_value=TextSubstitution(text="camera"))

    logger_launch_arg = DeclareLaunchArgument(              
        "logger", default_value=TextSubstitution(text="False")     
    )
    
    return LaunchDescription([
        rqt_graph,
        
        cam_num_launch_arg,   
        data_source_launch_arg,
        logger_launch_arg,
        
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            parameters=[{
                "cam_num": LaunchConfiguration('cam_num'), 
                "data_source": LaunchConfiguration('data_source'),
                "logger": LaunchConfiguration('logger'), 
            }]
        ),
        
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
        ),
        
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='lane_info_extractor_node',
            parameters=[{
                "logger": LaunchConfiguration('logger'), 
            }]
        ),
        
        Node(
            package='camera_perception_pkg',
            executable='traffic_light_detector_node',
            name='traffic_light_detector_node',
        )
        
    ])