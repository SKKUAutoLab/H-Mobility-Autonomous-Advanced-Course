import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String

from .lib import control_motor as CONTROL
from .lib import convert_arduino_msg as PROTOCOL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "serial_msg"

# 아두이노 장치 이름 (ls /dev/ttyACM* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/ttyACM0'

# 가변저항 입력 핀
VARIBLE_RESISTOR_INPUT_PIN = 'A2'

# 조향모터에 연결된 핀
STEERING_PIN1 = 10
STEERING_PIN2 = 12

# 좌측 뒷바퀴에 연결된 핀
LEFT_REAR_PIN1 = 3
LEFT_REAR_PIN2 = 5

# 우측 뒷바퀴에 연결된 핀
RIGHT_REAR_PIN1 = 6
RIGHT_REAR_PIN2 = 8

# 가변저항 최대 좌측값 & 최대 우측값
VARIBLE_RESISTOR_MOST_LEFT = 461 
VARIBLE_RESISTOR_MOST_RIGHT = 346

#----------------------------------------------

ser = serial.Serial(PORT, 9600, timeout=1)
time.sleep(1)

class SendSignal():
  def __init__(self):
    CONTROL.arduino_pinsetting(ser,
                        VARIBLE_RESISTOR_INPUT_PIN,
                        STEERING_PIN1,
                        STEERING_PIN2,
                        LEFT_REAR_PIN1,
                        LEFT_REAR_PIN2,
                        RIGHT_REAR_PIN1,
                        RIGHT_REAR_PIN2,
                        VARIBLE_RESISTOR_MOST_LEFT,
                        VARIBLE_RESISTOR_MOST_RIGHT)    
    time.sleep(1)

  def process(self, protocol):
    message = protocol.data
    ser.write(message.encode())
    return

class MotorControlNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic="topic_send_signal"):
    super().__init__('serial_sender_node')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('pub_topic', pub_topic)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
    
    self.send_serial = SendSignal()
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=1)
    self.subscription = self.create_subscription(String, self.sub_topic, self.data_callback, qos_profile)

  def data_callback(self, msg):
    self.send_serial.process(msg)

def main(args=None):
  rclpy.init(args=args)
  node = MotorControlNode()
  try:
      rclpy.spin(node)
      
  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      message = PROTOCOL.protocol_with_differential(0, 0, 0)
      ser.write(message.encode())
      pass
    
  finally:
    ser.close()
    print('closed')
    
  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
