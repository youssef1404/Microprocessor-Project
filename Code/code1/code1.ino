#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12

//Right motor
#define ENA 6
#define IN1 7
#define IN2 8

//Left motor
#define ENB 5
#define IN3 9
#define IN4 10

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  Serial.begin(9600);
}

void loop() {
  int leftSensorValue = digitalRead(IR_SENSOR_LEFT);
  int rightSensorValue = digitalRead(IR_SENSOR_RIGHT);

  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print("  Right Sensor: ");
  Serial.println(rightSensorValue);

  if (leftSensorValue == LOW && rightSensorValue == LOW) {
    // Both sensors on the line - move forward
    moveForward();
  } else if (leftSensorValue == HIGH && rightSensorValue == LOW) {
    // Left sensor off the line - turn left
    turnRight();
  } else if (leftSensorValue == LOW && rightSensorValue == HIGH) {
    // Right sensor off the line - turn right
    turnLeft();
  } else {
    // Lost the line - stop
    stopMotors();
  }
}

void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 60); 

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 60); 
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); 

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 40); 
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 40); 

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0); 
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}







