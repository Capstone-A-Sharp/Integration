#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <math.h>
#include <SoftwareSerial.h>

// === 자이로센서 값 변수 선언 ===
float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;
float mx = 0.0, my = 0.0, mz = 0.0;
float roll = 0.0, pitch = 0.0, yaw = 0.0;

MPU9250_asukiaaa MPU9250;

// === 압력센서 설정 ===
const int numRows = 16;
const int numCols = 16;
const int delay_clr = 10;
const int delay_clc = 50;
const int colIdx[16] = {1, 3, 5, 7, 9, 11, 13, 15, 14, 12, 10, 8, 6, 4, 2, 0};
int pressureData[numRows][numCols * 2] = {{0,},};

// 압력센서 핀 정의
const int clrPin = 5;     // 공통 CLR 핀
const int clkPin = 6;     // 공통 CLK 핀
const int sigPin1 = A0;    // 센서 출력 입력 핀
const int sigPin2 = A1;

// calib스위치 핀 정의  
const int calibSwitchInputPin = 4;
int calib_switch = 0;

//motor driver 핀 정의
const int motorA1 = 8;
const int motorA2 = 7;
const int motorPW_A = 9;
const int motorB1 = 10;
const int motorB2 = 12;
const int motorPW_B = 11;

const int motorswitch = A2;
int motor_switch=0;

// SoftwareSerial 핀 설정---------------------------------------------------------
#define SW_RX 2  // 소프트웨어 시리얼 수신핀
#define SW_TX 3  // 소프트웨어 시리얼 송신핀 (여기선 사용 안함)
SoftwareSerial SoftSerial(SW_RX, SW_TX);

unsigned int bufferNum = 0;
unsigned int speedL = 0;
unsigned int speedR = 0;


//MPU9250------------------------------------------------------------------------
void setMPU9250(){
  MPU9250.setWire(&Wire);
  MPU9250.beginAccel();
  MPU9250.beginGyro();
  MPU9250.beginMag();  // 자기장 사용
  delay(1000);
}

void sensingMPU9250(){
  MPU9250.accelUpdate();
  MPU9250.gyroUpdate();
  MPU9250.magUpdate();

  // 센서 값 읽기
  ax = MPU9250.accelX(); ay = MPU9250.accelY(); az = MPU9250.accelZ();
  gx = MPU9250.gyroX(); gy = MPU9250.gyroY(); gz = MPU9250.gyroZ();
  mx = MPU9250.magX(); my = MPU9250.magY(); mz = MPU9250.magZ();

  roll  = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  yaw   = atan2(my, mx) * 180.0 / PI;
  if (yaw < 0) yaw += 360.0;  // 음수 방지
}

void sendMPU9250DataAsJson(){
  // JSON 전송
  Serial.print("\"MPU9250\":{");

  //Serial.print("\"accel\":{");
  //Serial.print("\"x\":"); Serial.print(ax, 3); Serial.print(",");
  //Serial.print("\"y\":"); Serial.print(ay, 3); Serial.print(",");
  //Serial.print("\"z\":"); Serial.print(az, 3); Serial.print("},");

  //Serial.print("\"gyro\":{");
  //Serial.print("\"x\":"); Serial.print(gx, 3); Serial.print(",");
  //Serial.print("\"y\":"); Serial.print(gy, 3); Serial.print(",");
  //Serial.print("\"z\":"); Serial.print(gz, 3); Serial.print("},");

  //Serial.print("\"mag\":{");
  //Serial.print("\"x\":"); Serial.print(mx, 3); Serial.print(",");
  //Serial.print("\"y\":"); Serial.print(my, 3); Serial.print(",");
  //Serial.print("\"z\":"); Serial.print(mz, 3); Serial.print("},");

  //Serial.print("\"roll\":"); Serial.print(roll, 2); Serial.print(",");
  Serial.print("\"pitch\":"); Serial.print(pitch, 2);
  //Serial.print("\"yaw\":"); Serial.print(yaw, 2);

  Serial.println("}");
}


//FSR=============================================================================
void setFSRgpio(){
  // 핀 모드 설정
  pinMode(clrPin, OUTPUT);
  pinMode(clkPin, OUTPUT);
}

void resetFSRcounter(){
  // 초기화: 클리어 한번만 수행
  digitalWrite(clrPin, LOW);
  delayMicroseconds(delay_clr);
  digitalWrite(clrPin, HIGH);
  delayMicroseconds(delay_clr);
  digitalWrite(clkPin, LOW);
}

void readPressureData(){
  // 카운터를 다시 0으로 초기화
  digitalWrite(clrPin, HIGH);
  delayMicroseconds(delay_clr);
  digitalWrite(clrPin, LOW);

  // 열 루프 (MUX 쪽 출력 0~3)
  for (int col = 0; col < numCols; col++) {
    // 행 루프 (DEMUX 쪽 출력 4~7)
    for (int row = 0; row < numRows; row++) {
      // 센서값 읽기
      int sensorValue1 = 1023 - analogRead(sigPin1);
      pressureData[colIdx[col]][row] = sensorValue1;
      int sensorValue2 = 1023 - analogRead(sigPin2);
      pressureData[colIdx[col]][row + numCols] = sensorValue2;

      // CLK 펄스 (행 증가)
      digitalWrite(clkPin, HIGH);
      delayMicroseconds(delay_clc);
      digitalWrite(clkPin, LOW);
      delayMicroseconds(delay_clc);
    }
  }
}

void sendPressureDataAsJson() {
  Serial.println("\"FSR\": {");

  int left_sum = 0;
  int right_sum = 0;

  for (int row = 0; row < numRows; row++) {
    for (int col = 0; col < numCols * 2; col++) {
      int value = pressureData[row][col];
      if (col < 16) left_sum += value;
      else          right_sum += value;
    }
  }

  Serial.print("\"left_sum\":");
  Serial.print(left_sum);
  Serial.print(",");
  Serial.print("\"right_sum\":");
  Serial.println(right_sum);

  Serial.println("}");  // FSR 닫기
}


// calibswitch ------------------------------------------------------------
void setCalibswitch(){
  pinMode(calibSwitchInputPin, INPUT_PULLUP);
}

void sendCalibswitchJson(){
  calib_switch = (digitalRead(calibSwitchInputPin) == LOW) ? 1 : 0;
  Serial.print("\"calib_switch\":");
  Serial.println(calib_switch);
}

// motordriver -------------------------------------------------------------------
void setMotordriver(){
  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT); pinMode(motorPW_A, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT); pinMode(motorPW_B, OUTPUT);
  pinMode(motorswitch, INPUT_PULLUP);
}

void readMotorinfo(){
  // 모터 제어
  if (Serial.available()) {
    int speed = Serial.parseInt();
    int motorswitchState = digitalRead(motorswitch);
  //Serial.print("SwitchState: ");
  //Serial.println(motorswitchState);
    //int motorswitchState=LOW;
    controlMotors(speed, motorswitchState);
  }
}

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

void sendForBackJson(){
  motor_switch = (digitalRead(motorswitch) == LOW) ? 1 : 0;
  Serial.print("\"motor_switch\":");
  Serial.println(motor_switch);
}

//Wheel-----------------------------------------------------------------------
void readWheelSpeed(){
  static String buffer = "";  // 수신된 문자열 임시 저장

  // 소프트웨어 시리얼에 수신 데이터가 있을 경우
  while (SoftSerial.available()) {
    char ch = SoftSerial.read();

    if (isDigit(ch)) {
      buffer += ch; // 숫자면 버퍼에 추가
    } else if (ch == '\n' || ch == '\r') {
      // 엔터(개행 문자)가 들어오면 문자열 → 숫자 변환
      if (buffer.length() > 0) {
        bufferNum = buffer.toInt(); // 정수로 변환
        
        if(bufferNum >> 8)speedR = bufferNum & 0xFF;
        else             {speedL = bufferNum & 0xFF;}

        buffer = ""; // 버퍼 초기화
      }
    } else {
      // 숫자도 아니고 엔터도 아니면 무시
    }
  }
}

void sendWheelSpeedDataAsJson(){
  Serial.print("\"Wheel_Speed\": {\"L\":");
  Serial.print(speedL);
  Serial.print(", \"R\":");
  Serial.print(speedR);
  Serial.println("}");
}


// JSON 포맷 데이터 설정 함수들 ---------------------------------------------------------
void sendJsonStart(){
  Serial.println("{");
}

void sendJsonEnd(){
  Serial.println("}");  // 전체 JSON 닫기
  Serial.println("END");  // JSON 한 세트의 끝 표시
}

void sendComma4Field2Field(){
  Serial.print(",");
}

//------------------------------------------------------------------------------
void setup() {
  // 하드웨어 UART 시작
  Serial.begin(115200);
  Serial.setTimeout(10); // 10ms만 대기


    // 소프트웨어 UART 시작
  SoftSerial.begin(9600);

  Wire.begin();
  setMPU9250();

  setFSRgpio();
  resetFSRcounter();

  setCalibswitch();

  setMotordriver();
}

void loop() {
  sensingMPU9250();
  readPressureData();
  readWheelSpeed();

  sendJsonStart();
  sendMPU9250DataAsJson();
  sendComma4Field2Field();
  sendPressureDataAsJson();
  sendComma4Field2Field();
  sendCalibswitchJson();
  sendComma4Field2Field();
  sendWheelSpeedDataAsJson();
  sendComma4Field2Field();
  sendForBackJson();
  sendJsonEnd();

  readMotorinfo();

  delay(100);  // 10Hz 주기
}
