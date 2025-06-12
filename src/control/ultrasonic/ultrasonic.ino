#define TRIG 3  // 초음파 송신 핀 설정
#define ECHO 2  // 초음파 수신 핀 설정

void setup() {
  Serial.begin(9600); // 센서 데이터를 PC에서 확인할 수 있도록 시리얼 통신 시작

  // 핀 모드 설정: 초음파 송신은 출력, 수신은 입력
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  long duration, distance;

  // 초음파 송신 전에 LOW로 초기화
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  // 10마이크로초 동안 HIGH 신호를 보내 초음파를 발사
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // 초음파가 물체에 반사되어 돌아오는 데 걸린 시간을 측정
  duration = pulseIn(ECHO, HIGH);

  // 측정 시간(duration)을 바탕으로 거리(cm) 계산
  // 소리의 속도는 약 340m/s이므로, 공식은 다음과 같음:
  // 거리 = 시간 * (34000 / 2) / 1,000,000  => 거리(cm) = 시간 * 17 / 1000
  distance = duration * 17 / 1000;

  // 측정된 시간과 거리 값을 시리얼 모니터에 출력
  Serial.println(duration);
  Serial.print("\nDistance : ");
  Serial.print(distance);
  Serial.println(" cm");

  // 1초 간격으로 측정 반복
  delay(1000);
}
