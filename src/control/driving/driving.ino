// 최대 입력 문자 수
const unsigned int MAX_INPUT = 15;

// 핀 번호 변수
const int STEERING_1 = 2;
const int STEERING_2 = 3;
const int FORWARD_RIGHT_1 = 4;
const int FORWARD_RIGHT_2 = 5;
const int FORWARD_LEFT_1 = 6;
const int FORWARD_LEFT_2 =7;
const int POT = A2;

// 조향 속도 상수
const int STEERING_SPEED = 128;

// 가변저항 값 범위
const int resistance_most_left = 460;
const int resistance_most_right = 352;

// 조향 최대 단계 수 (한 쪽 기준)
const int MAX_STEERING_STEP = 7;

// 제어 상태 변수
int angle = 0, resistance = 0, mapped_resistance = 0;
int left_speed = 0, right_speed = 0;

// 명령 주기 제한 변수
unsigned long lastCommandTime = 0; // 마지막 명령 처리 시간
const unsigned int COMMAND_INTERVAL = 50; // 명령 처리 간 최소 대기 시간(ms)

// 함수 선언
void steerRight();
void steerLeft();
void maintainSteering();
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);
void processIncomingByte(const byte inByte);
void processData(const char *data);

void setup() {
    Serial.begin(115200);

    // 핀 모드 설정
    pinMode(POT, INPUT);
    pinMode(STEERING_1, OUTPUT);
    pinMode(STEERING_2, OUTPUT);
    pinMode(FORWARD_RIGHT_1, OUTPUT);
    pinMode(FORWARD_RIGHT_2, OUTPUT);
    pinMode(FORWARD_LEFT_1, OUTPUT);
    pinMode(FORWARD_LEFT_2, OUTPUT);
}

void loop() {
    // 현재 시간 가져오기
    unsigned long currentTime = millis();

    // 직렬 데이터 처리
    while (Serial.available() > 0) {
        processIncomingByte(Serial.read());
    }

    // 일정 시간 간격으로만 제어 명령 실행
    if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {
        // 포텐셔미터 값을 읽어 조향 계산
        resistance = analogRead(POT);
        mapped_resistance = map(resistance, resistance_most_left, resistance_most_right, -MAX_STEERING_STEP, MAX_STEERING_STEP + 1);

        // 조향 상태에 따라 동작 제어
        if (mapped_resistance == angle) {
            maintainSteering();
        } else if (mapped_resistance > angle) {
            steerLeft();
        } else {
            steerRight();
        }

        // 모터 속도 설정
        setLeftMotorSpeed(left_speed);
        setRightMotorSpeed(right_speed);

        // 마지막 명령 시간 갱신
        lastCommandTime = currentTime;
    }
}

// 조향 제어 함수
void steerRight() {
    analogWrite(STEERING_1, STEERING_SPEED);
    analogWrite(STEERING_2, LOW);
}

void steerLeft() {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, STEERING_SPEED);
}

void maintainSteering() {
    analogWrite(STEERING_1, LOW);
    analogWrite(STEERING_2, LOW);
}

// 모터 속도 설정 함수
void setLeftMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_LEFT_1, speed);
        analogWrite(FORWARD_LEFT_2, LOW);
    } else {
        analogWrite(FORWARD_LEFT_1, LOW);
        analogWrite(FORWARD_LEFT_2, (-1) * speed);
    }
}

void setRightMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(FORWARD_RIGHT_1, speed);
        analogWrite(FORWARD_RIGHT_2, LOW);
    } else {
        analogWrite(FORWARD_RIGHT_1, LOW);
        analogWrite(FORWARD_RIGHT_2, (-1) * speed);
    }
}

// 직렬 데이터 처리
void processIncomingByte(const byte inByte) {
    static char input_line[MAX_INPUT];
    static unsigned int input_pos = 0;

    switch (inByte) {
        case '\n':
            input_line[input_pos] = 0; // 종료 문자 추가
            processData(input_line);  // 데이터 처리
            input_pos = 0; // 버퍼 초기화
            break;

        case '\r':
            break; // 캐리지 리턴 무시

        default:
            if (input_pos < (MAX_INPUT - 1)) {
                input_line[input_pos++] = inByte;
            }
            break;
    }
}

// 데이터 패킷 처리
void processData(const char *data) {
    int sIndex = -1, lIndex = -1, rIndex = -1;

    // 명령 파싱
    for (int i = 0; data[i] != '\0'; i++) {
        if (data[i] == 's') sIndex = i;
        else if (data[i] == 'l') lIndex = i;
        else if (data[i] == 'r') rIndex = i;
    }

    if (sIndex != -1 && lIndex != -1 && rIndex != -1) {
        int newAngle = atoi(data + sIndex + 1);
        int newLeftSpeed = atoi(data + lIndex + 1);
        int newRightSpeed = atoi(data + rIndex + 1);

        // 명령 값 업데이트 (중복 명령 무시)
        if (newAngle != angle || newLeftSpeed != left_speed || newRightSpeed != right_speed) {
            angle = newAngle;
            left_speed = newLeftSpeed;
            right_speed = newRightSpeed;

            // 조향 값 제한
            if (angle > MAX_STEERING_STEP) angle = MAX_STEERING_STEP;
            else if (angle < -MAX_STEERING_STEP) angle = -MAX_STEERING_STEP;
        }
    }
}
