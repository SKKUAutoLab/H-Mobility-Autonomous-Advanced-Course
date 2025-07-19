import serial
import marshal
import types
import os
import time

real_path = os.path.dirname(os.path.realpath(__file__))
pyc = open((real_path)+'/data_collection_func_lib.cpython-310.pyc', 'rb').read()
code = marshal.loads(pyc[16:])
module = types.ModuleType('module_name')
exec(code, module.__dict__)

def main():
    DATA_PATH= os.path.dirname(real_path) + '/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets' 
    CAMERA_NUM = 0
    SERIAL_PORT = "/dev/ttyACM0"
    MAX_STEERING = 7  # 사용자 정의 최대 조향 단계

    print(DATA_PATH)

    # 데이터 수집 객체 초기화
    data_collector = module.Data_Collect(path=DATA_PATH, cam_num=CAMERA_NUM, max_steering=MAX_STEERING)
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(1)
    try:
        # 숨겨진 코드 프로세스 시작
        while True:
            # 한 번의 키보드 입력 처리
            result = data_collector.process()

            # 프로세스 종료 플래그 확인
            if result["exit"]:
                steering = 0
                left_speed = 0
                right_speed = 0
                message = f"s{steering}l{left_speed}r{right_speed}\n"
                ser.write(message.encode())
                break

            # 현재 제어 값 가져오기
            control_values = data_collector.get_control_values()

            # 시리얼 송신
            message = f"s{control_values['steering']}l{control_values['left_speed']}r{control_values['right_speed']}\n"
            ser.write(message.encode())

            # 디버깅용 출력
            print(f"Sent: {message.strip()}")

    except KeyboardInterrupt:
        steering = 0
        left_speed = 0
        right_speed = 0
        message = f"s{steering}l{left_speed}r{right_speed}\n"
        ser.write(message.encode())
        print("Program interrupted.")
    finally:
        ser.close()
        data_collector.cleanup()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
