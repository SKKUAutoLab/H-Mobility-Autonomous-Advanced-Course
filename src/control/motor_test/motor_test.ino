//모터 제어 코드

//모터의 회전 속도 입력 (-255 ~ 255)
//양수는 전진, 음수는 후진
//절댓값이 클수록 모터가 빠르게 회전 
static int target_speed = 50;

//모터 드라이버 IN1,IN2에 연결하는 아두이노 핀번호 입력 (모터가 반대로 동작하면 순서를 IN2,IN1 순서로 입력)
static int test_motor1 = 3;
static int test_motor2 = 4;


//여기서부터는 건드릴 필요 없음.
//---------------------------------------------------------------------------------------------------------
void motor_control(int m1, int m2, int speed){
  if (speed < 0){
    analogWrite(m1, 0);
    analogWrite(m2, speed);
  }
  else{
    analogWrite(m1, speed);
    analogWrite(m2, 0);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(test_motor1, OUTPUT);
  pinMode(test_motor2, OUTPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  motor_control(test_motor1, test_motor2, target_speed);
}