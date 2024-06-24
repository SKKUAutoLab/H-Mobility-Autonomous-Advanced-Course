import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib import lidar_perception_func_lib as LPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = 'lidar_processed'  # 구독할 토픽 이름

# Publish할 토픽 이름
PUB_TOPIC_NAME = 'lidar_obstacle_info'  # 물체 감지 여부를 퍼블리시할 토픽 이름
#----------------------------------------------


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(LaserScan, SUB_TOPIC_NAME, self.lidar_callback, self.qos_profile)
        self.publisher = self.create_publisher(Bool, PUB_TOPIC_NAME, self.qos_profile) 

        self.detection_checker = LPFL.StabilityDetector(consec_count=5) # 연속적으로 몇 번 감지 여부를 확인할지 설정




    def lidar_callback(self, msg):
         
        start_angle = 0  # 원하는 각도 범위의 시작 값
        end_angle = 30  # 원하는 각도 범위의 끝 값
        
        range_min = 0.5  # 원하는 거리 범위의 최소값 [m]
        range_max = 2.0  # 원하는 거리 범위의 최대값 [m]

        ranges = msg.ranges


        detected = LPFL.detect_object(ranges=ranges, start_angle=start_angle, end_angle=end_angle, range_min=range_min, range_max=range_max)
        
        # ranges는 라이다 센서값 입력                                                        

        # 각도 범위 지정
        # 예시 1) 
        # start_angle을 355도로, end_angle을 4도로 설정하면, 
        # 355도에서 4도까지의 모든 각도(355, 356, 357, 358, 359, 0, 1, 2, 3, 4도)가 포함.
        # 
        # 예시 2)
        # start_angle을 0도로, end_angle을 30도로 설정하면, 
        # 0도에서 30도까지의 모든 각도(0, 1, 2, ..., 30도)가 포함.
        # 
        # 예시 3)
        # start_angle을 180도로, end_angle을 190도로 설정하면, 
        # 180도에서 190도까지의 모든 각도(180, 181, 182, ..., 190도)가 포함. 

        # 거리범위 지정 
        # range_min보다 크거나 같고, range_max보다 작거나 같은 거리값을 포함.

        # 각도범위 및 거리범위를 둘 다 만족하는 범위에 라이다 센서값이 존재하면 True, 아니면 False 리턴. 


        detection_result = self.detection_checker.check_consecutive_detections(detected)

        detection_msg = Bool()
        detection_msg.data = detection_result
        self.publisher.publish(detection_msg)

        self.get_logger().info(f'Lidar Obstacle detected: {detection_result}')

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetection()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
