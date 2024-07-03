from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# ros2 launch launch_pkg lidar_check.launch.py

def generate_launch_description():   
    
    rqt_graph = ExecuteProcess(
    	cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
    	output="screen",
    )
    
    return LaunchDescription([
            
        rqt_graph,
        
        Node(
            package='lidar_perception_pkg',
            executable='lidar_pub_node',
            name='lidar_publisher_node',
        ),
        
        Node(
            package='lidar_perception_pkg',
            executable='lidar_processor_node',
            name='lidar_processor_node',
        ),
        
        Node(
            package='lidar_perception_pkg',
            executable='lidar_obstacle_detector_node',
            name='lidar_obstacle_detector_node',
        )
        
    ])