import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib import lidar_perception_func_lib as LPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = 'lidar_raw'

# Publish할 토픽 이름
PUB_TOPIC_NAME = 'lidar_processed'
#----------------------------------------------

class LidarSensorDataProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            LaserScan,
            SUB_TOPIC_NAME,
            self.lidar_raw_cb,
            self.qos_profile)  
        
        self.publisher = self.create_publisher(
            LaserScan,
            PUB_TOPIC_NAME,
            self.qos_profile) 

    def lidar_raw_cb(self, msg):
        # 이 함수는 Lidar 데이터를 수신할 때마다 호출 됨.
        ranges = msg.ranges
        intensities = msg.intensities

        msg = LPFL.rotate_lidar_data(msg, offset = 0) # offset은 0부터 359까지의 값을 입력
        msg = LPFL.flip_lidar_data(msg, pivot_angle = 0) # pivot_angle은 0부터 359까지의 값을 입력
        self.publisher.publish(msg)
        self.get_logger().info(f'Received scan with {len(ranges)} ranges and {len(intensities)} intensities')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarSensorDataProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
