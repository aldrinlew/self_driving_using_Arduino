#include <Arduino.h>

// Motor connections
const int MOTOR1_PIN1 = 2;
const int MOTOR1_PIN2 = 3;
const int MOTOR2_PIN1 = 4;
const int MOTOR2_PIN2 = 5;

// Sensor connections
const int LEFT_IR_SENSOR = A3;
const int RIGHT_IR_SENSOR = A2;
const int PIR_SENSOR = A1;
const int BUZZER_PIN = A5;
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;



void setup() {
  // Initialize motor pins as output
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  // Initialize sensor pins
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(PIR_SENSOR, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
}

void loop() {
  int pirState = digitalRead(PIR_SENSOR);
  int leftIRState = digitalRead(LEFT_IR_SENSOR);
  int rightIRState = digitalRead(RIGHT_IR_SENSOR);

  if (pirState == HIGH) {
    stop();
  } else if (leftIRState == LOW) {
    turnRight();
  } else if (rightIRState == LOW) {
    turnLeft();
  } else {
    int distance = measureDistance();
    if (distance > 15) {
      forward();
    }
     else {
      stop();
      while (true)
      {
        buzzon();
        delay(5000);
        buzzoff();
      }
      
    }
  }
}

void forward() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void stop() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnRight() {
  digitalWrite(MOTOR1_PIN1, HIGH);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
}

void turnLeft() {
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, HIGH);
  digitalWrite(MOTOR2_PIN2, LOW);
}

int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void buzzon() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzoff() {
  digitalWrite(BUZZER_PIN, LOW);
}
