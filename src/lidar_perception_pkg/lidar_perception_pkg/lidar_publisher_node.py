import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import sys
import os
import tf2_ros
import geometry_msgs.msg
# from .lib.rplidar import RPLidar, RPLidarException
from .lib import lidar_perception_func_lib as LPFL
import numpy as np

#---------------Variable Setting---------------
# Publish할 토픽 이름
PUB_TOPIC_NAME = 'lidar_raw' 

# 라이다 장치 번호 (ls /dev/ttyUSB* 명령을 터미널 창에 입력하여 확인)
LIDAR_PORT = '/dev/ttyUSB0'
#----------------------------------------------

class LidarSensorDataPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.publisher_ = self.create_publisher(LaserScan, PUB_TOPIC_NAME, self.qos_profile)
        self.lidar = None
        self.lidar_sensor_data_generator = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Set up a timer to call publish_lidar_data at a regular interval
        self.timer = self.create_timer(0.1, self.publish_lidar_data)
        self.initialize_lidar()

    def initialize_lidar(self):
        """Initialize the RPLidar."""
        try:
            self.lidar = LPFL.RPLidar(LIDAR_PORT)
            self.lidar_sensor_data_generator = self.lidar.iter_scans()
        except LPFL.RPLidarException as e:
            self.get_logger().error(f'Failed to initialize LIDAR: {e}')
            self.destroy_node()
            rclpy.shutdown()
    
    def reset_lidar(self):
        """Reset the LIDAR connection and data generator."""
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except LPFL.RPLidarException as e:
            self.get_logger().error(f'Failed to reset LIDAR: {e}')
        
        self.initialize_lidar()

    def publish_lidar_data(self):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_frame'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        self.tf_broadcaster.sendTransform(transform)

        try:
            scan = next(self.lidar_sensor_data_generator)
            print('Got %d measurements' % len(scan))
            scan = np.array(scan)
            # Create LaserScan message
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'laser_frame'  # frame id of your lidar sensor
            msg.angle_min = 0.0  # Minimum angle of the scan [rad]
            msg.angle_max = 2 * np.pi  # Maximum angle of the scan [rad]
            msg.angle_increment = 2 * np.pi / 360.0  # Angular distance between measurements [rad]
            msg.time_increment = 0.0  # Time between measurements [seconds]
            msg.scan_time = 0.1  # Time between scans [seconds]
            msg.range_min = 0.15  # Minimum range value [m]
            msg.range_max = 12.0  # Maximum range value [m]

            ranges = [float('inf')] * int((msg.angle_max - msg.angle_min) / msg.angle_increment)
            intensities = [0.0] * int((msg.angle_max - msg.angle_min) / msg.angle_increment)
            
            for measurement in scan:
                angle = np.radians(measurement[1])  # Convert to radians
                if msg.angle_min <= angle <= msg.angle_max:
                    index = int((angle - msg.angle_min) / msg.angle_increment)
                    if 0 <= index < len(ranges):
                        ranges[index] = measurement[2] / 1000.0  # Distance measurement
                        intensities[index] = measurement[0]  # Intensity measurement
            
            msg.ranges = ranges
            msg.intensities = intensities

            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % PUB_TOPIC_NAME)

        except StopIteration:
            self.get_logger().error('Failed to get lidar scan')
            return
        except LPFL.RPLidarException as e:
            self.get_logger().error(f'RPLidar exception: {e}')
            self.reset_lidar()
        except ValueError as e:
            self.get_logger().error(f'ValueError: {e}')
            self.reset_lidar()

    def __del__(self):
        """Destructor to ensure LIDAR is properly shut down."""
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except LPFL.RPLidarException as e:
            self.get_logger().error(f'Failed to properly shutdown LIDAR: {e}')

def main(args=None):
    rclpy.init(args=args)
    lidar_publisher = LidarSensorDataPublisher()
    try:
        rclpy.spin(lidar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
