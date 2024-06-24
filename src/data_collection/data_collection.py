'''
키 조작

w : 속도 +10
s : 속도 -10
a : 회전 -1     (- : 좌회전, + : 우회전)
d : 회전 +1

c: 카메라 프레임 캡쳐

r : 일시정지
f : 종료
'''

import cv2
import string
import time
import sys
import os
import marshal
import types

real_path = os.path.dirname(os.path.realpath(__file__))
pyc = open((real_path)+'/data_collection_func.cpython-310.pyc', 'rb').read()
code = marshal.loads(pyc[16:])
module = types.ModuleType('module_name')
exec(code, module.__dict__)


# TODO ----------------------------------------------------
# video 번호 입력
CAM_NUM = 2 

# 아두이노 포트번호 입력
PORT='/dev/ttyACM0' 

# 데이터 수집 경로 입력
DATA_COLLECTION_PATH= os.path.dirname(real_path) + '/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets' 

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
VARIBLE_RESISTOR_MOST_LEFT = 450 
VARIBLE_RESISTOR_MOST_RIGHT = 335

# ----------------------------------------------------




def main():
    arduino_init_info = [VARIBLE_RESISTOR_INPUT_PIN,
                        STEERING_PIN1,
                        STEERING_PIN2,
                        LEFT_REAR_PIN1,
                        LEFT_REAR_PIN2,
                        RIGHT_REAR_PIN1,
                        RIGHT_REAR_PIN2,
                        VARIBLE_RESISTOR_MOST_LEFT,
                        VARIBLE_RESISTOR_MOST_RIGHT
                        ]
    data_collector = module.Data_Collect(path=DATA_COLLECTION_PATH, cam_num=CAM_NUM, ser_port=PORT, arduino_info=arduino_init_info)
    
    try:
        data_collector.process()
        
    except KeyboardInterrupt:
        data_collector.interrupt_process()


    data_collector.ser.close()

    if data_collector.cap.isOpened():
        data_collector.cap.release()


    cv2.destroyAllWindows()
    print('program finish')
    sys.exit(0)

if __name__ == '__main__':
    main()

