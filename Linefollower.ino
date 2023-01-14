#include <QTRSensors.h>
// Motors pins
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

float kp = 9;
float ki = 0;
float kd = 3;



float p = 1;
float i = 0;
float d = 0;

float error = 0;
float lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;

const int baseSpeed = 200;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};
void setup() {

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  delay(500);

  calibrateQTR();
}

void calibrateQTR() {

  // Calibrating the QTR
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  int speed = 160;

  int turns = 0;
  int state = 1;
  Serial.begin(9600);

  while (turns < 12) {
    qtr.calibrate();
    qtr.readLineBlack(sensorValues);

    switch (state) {
      case 1:
        setMotorSpeed(160, -160);
        
        if (sensorValues[0] > 800 && sensorValues[5] < 800) {
          // turn right
          state = -1;
          turns++;
        }
        break;
      case -1:
        setMotorSpeed(-160, 160);
        if (sensorValues[5] > 800 && sensorValues[0] < 800) {
          // turn left
          state = 1;
          turns++;
        }
        break;
    }
  }
  // Turn the LED off when the calibration is finished
  digitalWrite(LED_BUILTIN, LOW);

}

int PIDcontrol(float error) {

  p = error;
  i = i + error;
  d = error - lastError;

  lastError = error;
  int motorSpeed = kp * p + ki * i + kd * d;
  return motorSpeed;

}

void loop() {
 float error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);


  int motorSpeed = PIDcontrol(error);
  
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeed;
  }
  else if (error > 0) {
    m2Speed -= motorSpeed;
  }
  
  m1Speed = constrain(m1Speed, -maxSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, -maxSpeed, maxSpeed);


  setMotorSpeed(m1Speed, m2Speed);

}


// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {

  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  }
  else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  }
  else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    } 
  }
}

