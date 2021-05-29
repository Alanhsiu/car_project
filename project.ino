#include <NewPing.h>        //for the Ultrasonic sensor function library.
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
//#include "RFID.h"
//#include "bluetooth.h"
//#include "track.h"
// for RFID
#include <SPI.h>
#include <MFRC522.h>


#define MotorR_I1     7 //定義 I1 接腳（右）
#define MotorR_I2     8 //定義 I2 接腳（右）8
#define MotorL_I3     4 //定義 I3 接腳（左）4
#define MotorL_I4     2 //定義 I4 接腳（左） 2
#define MotorL_PWML    6 //定義 ENA (PWM調速) 接腳
#define MotorR_PWMR    5 //定義 ENB (PWM調速) 接腳 5
#define trig_pin1 A0 //left
#define echo_pin1 A1
#define trig_pin2 A2 //right
#define echo_pin2 A3
#define trig_pin3 A4 //front
#define echo_pin3 A5
#define trig_pin4 A6 //back
#define echo_pin4 A7
#define RST_PIN      A0        // 讀卡機的重置腳位
#define SS_PIN       10       // 晶片選擇腳位
#define buzzer 3
#define maximum_distance 200

NewPing sonar_left(trig_pin1, echo_pin1, maximum_distance);
NewPing sonar_right(trig_pin2, echo_pin2, maximum_distance);
NewPing sonar_front(trig_pin3, echo_pin3, maximum_distance);
NewPing sonar_back(trig_pin4, echo_pin4, maximum_distance);
MFRC522 mfrc522(SS_PIN, RST_PIN);  // 建立MFRC522物件
SoftwareSerial BT (9, 10);

int ld = 100;  //left_distance
int rd = 100;  //right_distance
int fd = 100;  //front_distance
int bd = 100;  //back_distance
int v0 = 100;
void setup() {

  BT.begin(9600);
  Serial.begin(9600);
  SPI.begin();
//  mfrc522.PCD_Init();
  pinMode(MotorR_I1,   OUTPUT);
  pinMode(MotorR_I2,   OUTPUT);
  pinMode(MotorL_I3,   OUTPUT);
  pinMode(MotorL_I4,   OUTPUT);
  pinMode(MotorL_PWML, OUTPUT);
  pinMode(MotorR_PWMR, OUTPUT);
  pinMode(buzzer, OUTPUT);

}

void loop() {

  //  byte idSize = mfrc522.uid.size;
  //  byte* rf = rfid(idSize);
  //  if (rf != NULL) {
  //    send_byte(rf, idSize);
  //    Serial.println(*rf);
  //  }

  ld = read_ld();
  rd = read_rd();
  fd = read_fd();
  bd = read_bd();

  if (fd >= 50) {
    double error = ld - rd;
    if (abs(error) < 20) move_forward();
    else MotorWriting(v0 - error / 2, v0 + error / 2);
  }
  else {
    while (read_fd() < 50) {
      move_stop();
      alarm();
    }
  }
}

int read_ld() {
  delay(30);
  int cm = sonar_left.ping_cm();
  if (cm == 0) cm = 250;
  //  Serial.println(cm);
  return cm;
}
int read_rd() {
  delay(30);
  int cm = sonar_right.ping_cm();
  if (cm == 0) cm = 250;
  //  Serial.println(cm);
  return cm;
}
int read_fd() {
  delay(30);
  int cm = sonar_front.ping_cm();
  if (cm == 0) cm = 250;
  //  Serial.println(cm);
  return cm;
}
int read_bd() {
  delay(30);
  int cm = sonar_back.ping_cm();
  if (cm == 0) cm = 250;
  //  Serial.println(cm);
  return cm;
}

void move_stop() {
  MotorWriting(0, 0);
  delay(1000);
}

void move_forward() {
  MotorWriting(150, 150);
}

void move_backward() {
  MotorWriting(-150, -150);
}

void alarm() {
  digitalWrite(7, HIGH);
  delay(1000);
  digitalWrite(7, LOW);
  delay(1000);
}
void MotorWriting(double vL, double vR) {
  // TODO: use L298N to control motor voltage & direction
  if (vR > 255) vR = 255;
  if (vL > 255) vL = 255;
  if (vR < -255) vR = -255;
  if (vL < -255) vL = -255;
  if (vL < 0) {
    digitalWrite(MotorR_I1, LOW);
    digitalWrite(MotorR_I2, HIGH);
    vL = -vL;
  }
  else{
    digitalWrite(MotorR_I1, HIGH);
    digitalWrite(MotorR_I2, LOW);
  }
  if (vR < 0) {
    digitalWrite(MotorL_I3, HIGH);
    digitalWrite(MotorL_I4, LOW);
    vR = -vR;
  }
  else{
    digitalWrite(MotorL_I3, LOW);
    digitalWrite(MotorL_I4, HIGH);
  }
  analogWrite(MotorL_PWML, vL);
  analogWrite(MotorR_PWMR, vR);
}// MotorWriting
