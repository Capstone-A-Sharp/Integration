#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <math.h>

// ====== 모터 드라이버 핀 설정 ======
const int motorA1 = 8;
const int motorA2 = 9;
const int motorPW_A = 3;

const int motorB1 = 10;
const int motorB2 = 11;
const int motorPW_B = 5;

const int switchPin = 2;

// ====== 압력센서 핀 및 설정 ======
const int numRows = 16;
const int numCols = 16;
const int delay_clr = 10;
const int delay_clc = 50;
const int clrPin = 5;
const int clkPin = 6;
const int sigPin1 = A0;
const int sigPin2 = A1;
const int colIdx[16] = {1,3,5,7,9,11,13,15,14,12,10,8,6,4,2,0};
int pressureData[numRows][numCols * 2] = { {0,}, };

// ====== 자이로 센서 설정 ======
MPU9250_asukiaaa mySensor;

// 초기 설정 -----------------------------------------------------
void setup() {
  Serial.begin(115200);

  // 모터 설정
  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT); pinMode(motorPW_A, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT); pinMode(motorPW_B, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);

  // 압력센서 설정
  pinMode(clrPin, OUTPUT);
  pinMode(clkPin, OUTPUT);
  resetFSRcounter();

  // 자이로 센서 설정
  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  delay(1000);
}

// 루프 --------------------------------------------------------
void loop() {
  // 자이로 측정 및 출력
  printIncline();

  // 압력센서 데이터 수집 및 출력
  readPressureData();
  sendJsonStart();
  sendPressureDataAsJson();
  sendJsonEnd();

  // 모터 제어
  if (Serial.available()) {
    int speed = Serial.parseInt();
    int switchState = digitalRead(switchPin);
    controlMotors(speed, switchState);
  }

  delay(200);
}

// 모터 제어 함수들 --------------------------------------------------------
void controlMotors(int speed, int switchState) {
  if (speed <= 10) {
    stopMotors(); return;
  }

  if (switchState == LOW) forwardMotors(speed);
  else backwardMotors(speed);
}

void stopMotors() {
  digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); digitalWrite(motorB2, LOW);
  analogWrite(motorPW_A, 0); analogWrite(motorPW_B, 0);
}

void forwardMotors(int speed) {
  digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW);
  analogWrite(motorPW_A, speed);
  digitalWrite(motorB1, HIGH); digitalWrite(motorB2, LOW);
  analogWrite(motorPW_B, speed);
}

void backwardMotors(int speed) {
  digitalWrite(motorA1, LOW); digitalWrite(motorA2, HIGH);
  analogWrite(motorPW_A, speed);
  digitalWrite(motorB1, LOW); digitalWrite(motorB2, HIGH);
  analogWrite(motorPW_B, speed);
}

// 압력 센서 관련 함수 --------------------------------------------------------
void resetFSRcounter(){
  digitalWrite(clrPin, LOW);
  delayMicroseconds(delay_clr);
  digitalWrite(clrPin, HIGH);
  delayMicroseconds(delay_clr);
  digitalWrite(clkPin, LOW);
}

void readPressureData(){
  digitalWrite(clrPin, HIGH);
  delayMicroseconds(delay_clr);
  digitalWrite(clrPin, LOW);

  for (int col = 0; col < numCols; col++) {
    for (int row = 0; row < numRows; row++) {
      int val1 = 1023 - analogRead(sigPin1);
      pressureData[colIdx[col]][row] = val1;
      int val2 = 1023 - analogRead(sigPin2);
      pressureData[colIdx[col]][row + numCols] = val2;

      digitalWrite(clkPin, HIGH);
      delayMicroseconds(delay_clc);
      digitalWrite(clkPin, LOW);
      delayMicroseconds(delay_clc);
    }
  }
}

void sendJsonStart() {
  Serial.println("{");
}

void sendJsonEnd() {
  Serial.println("}");
  Serial.println("END");
}

void sendPressureDataAsJson() {
  Serial.println("\"FSR\": {");
  for (int row = 0; row < numRows; row++) {
    Serial.print("\"row");
    Serial.print(row);
    Serial.print("\": [");
    for (int col = 0; col < numCols * 2; col++) {
      Serial.print(pressureData[row][col]);
      if (col < numCols * 2 - 1) Serial.print(",");
    }
    Serial.print("]");
    if (row < numRows - 1) Serial.println(",");
    else Serial.println("");
  }
  Serial.println("}");
}

// 자이로 센서 기울기 판단 함수 --------------------------------------------------------
void printIncline() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("° → ");
  if (pitch > 10) Serial.println("오르막");
  else if (pitch < -10) Serial.println("내리막");
  else Serial.println("평지");
}
