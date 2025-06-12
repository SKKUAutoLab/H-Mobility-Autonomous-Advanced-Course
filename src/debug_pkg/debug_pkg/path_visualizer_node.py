import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import PathPlanningResult
import cv2
import numpy as np
from cv_bridge import CvBridge

#---------------Variable Setting---------------
SUB_ROI_IMAGE_TOPIC = "roi_image"        # ROI 이미지 토픽
SUB_SPLINE_PATH_TOPIC = "path_planning_result"  # 경로 계획 결과 토픽
PUB_TOPIC_NAME = "path_visualized_img"      # 시각화된 이미지 퍼블리시 토픽

#----------------------------------------------
class PathVisualizerNode(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')

        # 파라미터 선언
        self.sub_roi_image_topic = self.declare_parameter('sub_roi_image_topic', SUB_ROI_IMAGE_TOPIC).value
        self.sub_spline_path_topic = self.declare_parameter('sub_spline_path_topic', SUB_SPLINE_PATH_TOPIC).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # CvBridge 초기화
        self.cv_bridge = CvBridge()

        # 구독자 설정 (이미지 및 경로 구독)
        self.roi_image_sub = self.create_subscription(
            Image, self.sub_roi_image_topic, self.roi_image_callback, self.qos_profile)
        
        self.spline_path_sub = self.create_subscription(
            PathPlanningResult, self.sub_spline_path_topic, self.spline_path_callback, self.qos_profile)

        # 퍼블리셔 설정 (시각화된 이미지 퍼블리시)
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)

        # 이미지와 경로 데이터를 저장하기 위한 변수
        self.roi_image = None
        self.spline_path = None

    def roi_image_callback(self, msg: Image):
        try:
            self.roi_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROI image: {str(e)}")

    def spline_path_callback(self, msg: PathPlanningResult):
        # 경로 데이터를 받아오기
        self.spline_path = list(zip(msg.x_points, msg.y_points))

        # ROI 이미지와 경로가 모두 준비되었을 때 시각화
        if self.roi_image is not None and self.spline_path is not None:
            self.visualize_path()

    def visualize_path(self):
        # 경로 점들을 이미지 위에 그리기
        for (x, y) in self.spline_path:
            # OpenCV에서 좌표는 (x, y) 순서이므로 그대로 사용
            cv2.circle(self.roi_image, (int(x), int(y)), 5, (0, 0, 255), -1)

        # 시각화된 이미지를 ROS 메시지로 변환하여 퍼블리시
        try:
            output_msg = self.cv_bridge.cv2_to_imgmsg(self.roi_image, encoding='bgr8')
            self.publisher.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for publishing: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
