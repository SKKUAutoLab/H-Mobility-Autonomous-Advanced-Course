import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from interfaces_pkg.msg import MotionCommand

from .lib import convert_arduino_msg as PROTOCOL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "serial_msg"
#----------------------------------------------

class ConvertProtocol():
  def __init__(self):
    pass
  
  def process(self, motor):
    steer = motor.steering
    left_speed = motor.left_speed
    right_speed = motor.right_speed 
    
    message = PROTOCOL.protocol_with_differential(steer, left_speed, right_speed)
    
    print(message)
    return message
  
class ConvertProtocolNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic=PUB_TOPIC_NAME):
    super().__init__('serial_protocol_converter_node')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('pub_topic', pub_topic)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
    
    self.is_running = False
    self.convert = ConvertProtocol()
    
    self.qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
    
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, self.qos_profile)
    self.publisher_ = self.create_publisher(String, self.pub_topic , self.qos_profile)
    
      
  def data_callback(self, motor):
    protocol_msg = self.convert.process(motor)
    
    msg = String()
    msg.data = protocol_msg
    self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConvertProtocolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        pass
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()