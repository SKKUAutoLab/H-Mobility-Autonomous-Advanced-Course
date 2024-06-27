import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import sys
import cv2
import os

#---------------Variable Setting---------------
# Publish할 토픽 이름
PUB_TOPIC_NAME = 'image_raw'

# 데이터 입력 소스: 'camera', 'image', 또는 'video' 중 택1하여 입력
DATA_SOURCE = 'video'

# 카메라(웹캠) 장치 번호 (ls /dev/video* 명령을 터미널 창에 입력하여 확인)
CAM_NUM = 0

# 이미지 데이터가 들어있는 디렉토리의 경로를 입력
IMAGE_DIRECTORY_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/sample_dataset'

# 비디오 데이터 파일의 경로를 입력
VIDEO_FILE_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4'

# 화면에 publish하는 이미지를 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True

# 이미지 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.03
#----------------------------------------------

class ImagePublisherNode(Node):
    def __init__(self, data_source=DATA_SOURCE, cam_num=CAM_NUM, img_dir=IMAGE_DIRECTORY_PATH, video_path=VIDEO_FILE_PATH, pub_topic=PUB_TOPIC_NAME, logger=SHOW_IMAGE, timer=TIMER):
        super().__init__('image_publisher_node')
        self.declare_parameter('data_source', data_source)
        self.declare_parameter('cam_num', cam_num)
        self.declare_parameter('img_dir', img_dir)
        self.declare_parameter('video_path', video_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        
        self.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        self.cam_num = self.get_parameter('cam_num').get_parameter_value().integer_value
        self.img_dir = self.get_parameter('img_dir').get_parameter_value().string_value
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.br = CvBridge()
        
        if self.data_source == 'camera':
            self.cap = cv2.VideoCapture(self.cam_num)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        elif self.data_source == 'video':
            self.cap = cv2.VideoCapture(self.video_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not self.cap.isOpened():
                self.get_logger().error('Cannot open video file: %s' % self.video_path)
                rclpy.shutdown()
                sys.exit(1)
        elif self.data_source == 'image':
            if os.path.isdir(self.img_dir):
                self.img_list = sorted(os.listdir(self.img_dir))
                self.img_num = 0
            else:
                self.get_logger().error('Not a directory file: %s' % self.img_dir)
                rclpy.shutdown()
                sys.exit(1)
        else:
            self.get_logger().error("Wrong data source: %s \nCheck that the DATA_SOURCE variable is either 'camera', 'image', or 'video'." % self.data_source)
            rclpy.shutdown()
            sys.exit(1)
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
    def timer_callback(self):
        if self.data_source == 'camera':
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (640, 480))
                image_msg = self.br.cv2_to_imgmsg(frame)
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame' 
                self.publisher.publish(self.br.cv2_to_imgmsg(frame))
                if self.logger:
                    cv2.imshow('Camera Image', frame)
                    cv2.waitKey(1)
        elif self.data_source == 'image':
            while self.img_num < len(self.img_list):
                img_file = self.img_list[self.img_num]
                img_path = os.path.join(self.img_dir, img_file)
                img = cv2.imread(img_path)
                if img is None:
                    self.get_logger().warn('Skipping non-image file: %s' % img_file)
                else:
                    img = cv2.resize(img, (640, 480))
                    image_msg = self.br.cv2_to_imgmsg(img)
                    image_msg.header = Header()
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = 'image_frame'
                    self.publisher.publish(self.br.cv2_to_imgmsg(img))
                    if self.logger:
                        self.get_logger().info('Published image: %s' % img_file)
                        cv2.imshow('Saved Image', img)
                        cv2.waitKey(1)
                
                self.img_num += 1
                break
            else:
                self.img_num = 0
        elif self.data_source == 'video':
            ret, img = self.cap.read()
            if ret:
                img = cv2.resize(img, (640, 480))
                image_msg = self.br.cv2_to_imgmsg(img)
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame'
                self.publisher.publish(image_msg)
                print(image_msg.header)
                if self.logger:
                    cv2.imshow('Video Frame', img)
                    cv2.waitKey(1)
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset video to the first frame
    
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        pass
    node.destroy_node()
    if node.cap.isOpened():
        node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
