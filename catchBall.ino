int MOTOR[4][2] = {{2, 5}, {4, 3}, {6, 9}, {8, 7}};
int yaw, yawold, up, kp, kd, t, a = 0, sign, white, b;
int grey_sens[16] = {513, 489, 559, 511, 517, 519, 446, 465, 519, 437, 466, 526, 581, 552, 548, 502};
int num_sens[16] = {9, 11, 5, 8, 10, 12, 6, 7, 3, 2, 13, 0, 1, 4, 14, 15};
int sens[16];
byte motorL, motorR;

#include <FastLED.h>
#include <Wire.h>
#include "GyroControl.h"

byte DirectA  = 9;
byte RDirectA = 6;
byte powerA   = 21;
byte DirectB  = 8;
byte RDirectB = 7;
byte powerB   = 20;
byte DirectC  = 35;
byte RDirectC = 36;
byte powerC   = 22;
byte DirectD  = 30;
byte RDirectD = 31;
byte powerD   = 23;
byte Mpower   = 150;
#define PinD1 39
#define PinD2 38
#define PinD3 37
#define PinA1 A22
#define PinA2 A21
#define NUM_LEDS 8
#define DATA_PIN 16
GyroControl gyro;
elapsedMillis timer;
CRGB leds[NUM_LEDS];


void MotorA(int speed) {
  if (speed > 255) { //Проверка на выход за мин.; макс.
    speed = 255;
  } else if (speed < -255) {
    speed = -255;
  }
  if (speed > 0) { // Определение направления езды
    digitalWrite(DirectA, HIGH);
    digitalWrite(RDirectA, LOW);
    analogWrite(powerA, speed);
  } else {
    digitalWrite(RDirectA, HIGH);
    digitalWrite(DirectA, LOW);
    analogWrite(powerA, abs(speed));
  }
}

void MotorB(int speed) {
  if (speed > 255) { //Проверка на выход за мин.; макс.
    speed = 255;
  } else if (speed < -255) {
    speed = -255;
  }
  if (speed > 0) { // Определение направления езды
    digitalWrite(DirectB, LOW);
    digitalWrite(RDirectB, HIGH);
    analogWrite(powerB, speed);
  } else {
    digitalWrite(RDirectB, LOW);
    digitalWrite(DirectB, HIGH);
    analogWrite(powerB, abs(speed));
  }
}

void MotorC(int speed) {
  if (speed > 255) { //Проверка на выход за мин.; макс.
    speed = 255;
  } else if (speed < -255) {
    speed = -255;
  }
  if (speed > 0) { // Определение направления езды
    digitalWrite(DirectC, HIGH);
    digitalWrite(RDirectC, LOW);
    analogWrite(powerC, speed);
  } else {
    digitalWrite(RDirectC, HIGH);
    digitalWrite(DirectC, LOW);
    analogWrite(powerC, abs(speed));
  }
}

void MotorD(int speed) {
  if (speed > 255) { //Проверка на выход за мин.; макс.
    speed = 255;
  } else if (speed < -255) {
    speed = -255;
  }
  if (speed > 0) { // Определение направления езды
    digitalWrite(DirectD, HIGH);
    digitalWrite(RDirectD, LOW);
    analogWrite(powerD, speed);
  } else {
    digitalWrite(RDirectD, HIGH);
    digitalWrite(DirectD, LOW);
    analogWrite(powerD, abs(speed));
  }
}

void MControl(int speedA, int speedB, int speedC, int speedD) {
  MotorA(speedA);
  MotorB(speedB);
  MotorC(speedC);
  MotorD(speedD);
}

void Move_deg(int angle,int power){
    up = yaw * kp + (yaw - yawold) * kd;
    MotorA(cos(radians(45 + angle)) * power + up);
    MotorB(cos(radians(135 + angle)) * -power + up);
    MotorC(cos(radians(-45 + angle)) * power - up);
    MotorD(cos(radians(-135 + angle)) * -power - up);
    yawold = yaw;
}

struct InfraredResult
{
  byte Direction;
  byte Strength;
};

class InfraredSeeker
{
  public:
    static void Initialize();
    static boolean Test();
    static void ReadACRaw(byte* buffer);
    static void ReadDCRaw(byte* buffer);
    static InfraredResult ReadAC();
    static InfraredResult ReadDC();
    static int DirectionAngle(byte Direction);
  private:
    static InfraredResult PopulateValues(byte* buffer);
    static void ReadValues(byte OffsetAddress, byte* buffer);
    static const int Address = 0x08; //Divide by two as 8bit-I2C address is provided
};

void InfraredSeeker::Initialize()
{
  Wire.begin();
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(0x00);
  Wire.endTransmission();
  while (Wire.available() > 0)
    Wire.read();
}

boolean InfraredSeeker::Test()
{
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.requestFrom(InfraredSeeker::Address, 16);
  char Manufacturer_Model[16];
  while (Wire.available() < 16);
  for (byte i = 0; i < 16; i++)
  {
    Manufacturer_Model[i] = Wire.read();
  }
  while (Wire.available() > 0)
    Wire.read();
  return strncmp(Manufacturer_Model, "HiTechncNewIRDir", 16) == 0;
}

void InfraredSeeker::ReadValues(byte OffsetAddress, byte* buffer)
{
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(OffsetAddress);
  Wire.endTransmission();
  Wire.requestFrom(InfraredSeeker::Address, 6);
  while (Wire.available() < 6);
  for (byte i = 0; i < 6; i++)
  {
    buffer[i] = Wire.read();
  }
  while (Wire.available() > 0)
    Wire.read();
}

void InfraredSeeker::ReadACRaw(byte* buffer)
{
  ReadValues(0x49, buffer);
}

void InfraredSeeker::ReadDCRaw(byte* buffer)
{
  ReadValues(0x42, buffer);
}

InfraredResult InfraredSeeker::PopulateValues(byte* buffer)
{
  InfraredResult Data;
  Data.Direction = buffer[0];
  if (buffer[0] != 0)
  {
    if (buffer[0] % 2 == 0)
    {
      Data.Strength = (buffer[buffer[0] / 2] + buffer[buffer[0] / 2 + 1]) / 2;
    }
    else
    {
      Data.Strength = buffer[buffer[0] / 2 + 1];
    }
  }
  else
  {
    Data.Strength = 0;
  }
  return Data;
}

InfraredResult InfraredSeeker::ReadAC()
{
  byte buffer[6];
  ReadACRaw(buffer);
  return PopulateValues(buffer);
}

InfraredResult InfraredSeeker::ReadDC()
{
  byte buffer[6];
  ReadDCRaw(buffer);
  return PopulateValues(buffer);
}

void setup()
{
  kp = 2;
  kd = 10;

  InfraredSeeker::Initialize();
  pinMode(21, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setSDA(33);
  Wire.setSCL(34);
  Serial.begin(9600);
  pinMode(39, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(37, OUTPUT);
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  gyro.start();
  delay(2000);
}



void loop()
{
  for (int i = 0; i < 8; i++){
    digitalWrite(PinD1, i % 2);
    digitalWrite(PinD2, (i % 4) / 2);
    digitalWrite(PinD3, i / 4);
    sens[num_sens[i]] = analogRead(PinA1);
    sens[num_sens[i + 8]] = analogRead(PinA2);
  }
  InfraredResult InfraredBall = InfraredSeeker::ReadAC();
  gyro.read();
  yaw = gyro.heading;
  white = 0;
  for (int i = 0; i < 16; i++){
    if (sens[i] < grey_sens[i]){
      white = 1;
    }
  }
  if (white == 0){
    if (InfraredBall.Direction == 0 or InfraredBall.Direction == 9){
      Move_deg(150, 150);
      leds[0] = CRGB::Black;
    }
    else if ((InfraredBall.Direction == 5 or InfraredBall.Direction == 6) and InfraredBall.Strength > 175){
      Move_deg(15, 255);
      leds[0] = CRGB::Red;
    }
    else{
      leds[0] = CRGB::Black;
      Move_deg((InfraredBall.Direction - 5) * 35, 150);
    }
  }
  else{
    b = InfraredBall.Direction - 5;
    sign = (b * 35) / abs((b * 35));
    if (sign == 0){
      sign = -1;
    }
    t = timer;
    if (InfraredBall.Direction == 0 or InfraredBall.Direction == 9 or (InfraredBall.Direction == 8 and InfraredBall.Strength < 100) or InfraredBall.Direction == 1){
      while (t - timer < 500){
        Move_deg(150, 150);
      }
      leds[0] = CRGB::Yellow;
    }
    else{
      while (t - timer < 500){
        Move_deg((-180 * sign) + (b * 35), 150);
      }
      leds[0] = CRGB::Yellow;
    }
  }/*
  white = 0;
  while (true){
    t = timer;
    while (timer < t + 10){
      Move_deg(white, 150);
    }
    white += 1;
  }*/
  FastLED.show();
  Serial.println(String(InfraredBall.Strength) + ' ' + String(InfraredBall.Direction) + ' ' + String(gyro.heading));
}
