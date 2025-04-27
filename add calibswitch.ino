#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <math.h>

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

float mx = 0.0;
float my = 0.0;
float mz = 0.0;

float roll  = 0.0;
float pitch = 0.0;
float yaw   = 0.0;

MPU9250_asukiaaa MPU9250;

const int numRows = 16;
const int numCols = 16;

const int delay_clr = 10;
const int delay_clc = 50;

const int colIdx[16] = {1, 3, 5, 7, 9, 11, 13, 15, 14, 12, 10, 8, 6, 4, 2, 0};

// 압력 데이터 저장 배열
int pressureData[numRows][numCols * 2] = {
  {0,},
};

// 핀 정의
const int clrPin = 5;     // 공통 CLR 핀
const int clkPin = 6;     // 공통 CLK 핀
const int sigPin1 = A0;    // 센서 출력 입력 핀
const int sigPin2 = A1;

const int calibSwitchInputPin = 10;
const int calibSwitchLowPin = 11;  // GND 대용 핀 (항상 LOW로 설정)
int calib_switch = 0;


void setMPU9250(){
  MPU9250.setWire(&Wire);
  MPU9250.beginAccel();
  MPU9250.beginGyro();
  MPU9250.beginMag();  // 자기장 사용
  delay(1000);

}

void setCalibswitch(){
  // 스위치 핀 설정
  pinMode(calibSwitchInputPin, INPUT_PULLUP);
  pinMode(calibSwitchLowPin, OUTPUT);
  digitalWrite(calibSwitchLowPin, LOW);  // 항상 LOW 유지
}

void sensingMPU9250(){
  MPU9250.accelUpdate();
  MPU9250.gyroUpdate();
  MPU9250.magUpdate();

  // 센서 값 읽기
  ax = MPU9250.accelX();
  ay = MPU9250.accelY();
  az = MPU9250.accelZ();

  gx = MPU9250.gyroX();
  gy = MPU9250.gyroY();
  gz = MPU9250.gyroZ();

  mx = MPU9250.magX();
  my = MPU9250.magY();
  mz = MPU9250.magZ();
  
  roll  = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  yaw = atan2(my, mx) * 180.0 / PI;
  if (yaw < 0) yaw += 360.0;  // 음수 방지
}

void sendMPU9250DataAsJson(){
  // JSON 전송
  Serial.print("\"MPU9250\":{");

  Serial.print("\"accel\":{");
  Serial.print("\"x\":"); Serial.print(ax, 3); Serial.print(",");
  Serial.print("\"y\":"); Serial.print(ay, 3); Serial.print(",");
  Serial.print("\"z\":"); Serial.print(az, 3); Serial.print("},");

  Serial.print("\"gyro\":{");
  Serial.print("\"x\":"); Serial.print(gx, 3); Serial.print(",");
  Serial.print("\"y\":"); Serial.print(gy, 3); Serial.print(",");
  Serial.print("\"z\":"); Serial.print(gz, 3); Serial.print("},");

  Serial.print("\"mag\":{");
  Serial.print("\"x\":"); Serial.print(mx, 3); Serial.print(",");
  Serial.print("\"y\":"); Serial.print(my, 3); Serial.print(",");
  Serial.print("\"z\":"); Serial.print(mz, 3); Serial.print("},");

  Serial.print("\"roll\":"); Serial.print(roll, 2); Serial.print(",");
  Serial.print("\"pitch\":"); Serial.print(pitch, 2); Serial.print(",");
  Serial.print("\"yaw\":"); Serial.print(yaw, 2);

  Serial.println("}");
}








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

  for (int row = 0; row < numRows; row++) {
    Serial.print("\"row");
    Serial.print(row);
    Serial.print("\": [");

    for (int col = 0; col < numCols * 2; col++) {
      Serial.print(pressureData[row][col]);
      if (col < numCols * 2 - 1) Serial.print(",");
    }

    Serial.print("]");
    if (row < numRows - 1) Serial.println(",");  // 마지막 줄엔 쉼표 없음
    else Serial.println("");
  }

  Serial.println("}");  // FSR 닫기
}


void sendCalibswitchJson(){
  // loop() 안 또는 그 전에 있어야 함
  calib_switch = (digitalRead(calibSwitchInputPin) == LOW) ? 1 : 0;
  Serial.print("\"calib_switch\":");
  Serial.println(calib_switch);
}





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

void setup() {

  Serial.begin(115200);
  Wire.begin();
  setMPU9250();

  setFSRgpio();

  setCalibswitch();
  resetFSRcounter();

}

void loop() {
  sensingMPU9250();
  readPressureData();

  sendJsonStart();
  sendMPU9250DataAsJson();
  sendComma4Field2Field();
  sendPressureDataAsJson();
  sendComma4Field2Field();
  sendCalibswitchJson();
  sendJsonEnd();
  delay(100);  // 10Hz 주기
}
