int sensorPin = A2;

void setup() {

  Serial.begin(9600);

  pinMode(sensorPin, INPUT);

}

 

void loop() {

  int value = analogRead(sensorPin);

  Serial.println(value);

  delay(100);

}
