int MOTOR[4][2] = {{2, 5}, {4, 3}, {6, 9}, {8, 7}};
int yaw, yawold, up, kp, kd, t, a = 0, sign, b, deg, power, dd, ot, timee, yawNew;
boolean white;
int grey_sens[16] = {513, 489, 559, 511, 517, 519, 446, 465, 519, 437, 466, 526, 581, 552, 548, 502 };
// робот без pixy: 450, 553, 521, 552, 494, 592, 440, 493, 511, 571, 597, 526, 493, 526, 559, 515
// робот c pixy: 566, 478, 518, 520, 508, 419, 519, 509, 488, 474, 408, 481, 424, 594, 514, 589
int num_sens[16] = {9, 11, 5, 8, 10, 12, 6, 7, 3, 2, 13, 0, 1, 4, 14, 15};
int sens[16];
int zones[4];
byte motorL, motorR;
long duration; // variable for the duration of sound wave travel
int distance;

#define echoPin 29
#define trigPin 28


#include <Wire.h>
#include "GyroControl.h"
#include <Pixy2.h>
#include<SPI.h>

int x = 0, y = 0;

// This is the main Pixy object
Pixy2 pixy;

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
GyroControl gyro;
elapsedMillis timer;


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

void ult() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
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

void Move_deg(int angle, int power) {
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

void White_check() {
  for (int i = 0; i < 4; i++) {
    zones[i] = 0;
  }
  if (sens[2] < grey_sens[2] or sens[1] < grey_sens[1] or sens[3] < grey_sens[3]) {
    white = 1;
    zones[0] = 1;
  }
  if (sens[7] < grey_sens[7] or sens[8] < grey_sens[8] or
      sens[4] < grey_sens[4] or sens[5] < grey_sens[5] or sens[6] < grey_sens[6]) {
    white = 1;
    zones[1] = 1;
  }
  if (sens[10] < grey_sens[10] or sens[11] < grey_sens[11] or sens[9] < grey_sens[9]) {
    white = 1;
    zones[2] = 1;
  }
  if (sens[15] < grey_sens[15] or sens[0] < grey_sens[0] or sens[12] < grey_sens[12] or sens[13] < grey_sens[13] or sens[14] < grey_sens[14]) {
    white = 1;
    zones[3] = 1;
  }
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
  kp = 1 ;
  kd = 5;

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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(20, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(9600);
  pinMode(39, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(37, OUTPUT);
  gyro.start();
  SPI.setSCK(14);
  Serial.print("Starting...\n");
  pixy.init();
  delay(2000);
}



void loop()
{
    for (int i = 0; i < 8; i++) {
      digitalWrite(PinD1, i % 2);
      digitalWrite(PinD2, (i % 4) / 2);
      digitalWrite(PinD3, i / 4);
      sens[num_sens[i]] = analogRead(PinA1);
      sens[num_sens[i + 8]] = analogRead(PinA2);
    }
    White_check();
    int i;
    pixy.ccc.getBlocks();
  
    if (pixy.ccc.numBlocks)
    {
      Serial.print("Detected ");
      Serial.println(pixy.ccc.numBlocks);
      for (i=0; i<pixy.ccc.numBlocks; i++)
      {
        Serial.print("  block ");
        Serial.print(i);
        Serial.print(": ");
        x=pixy.ccc.blocks[i].m_x;
        y=pixy.ccc.blocks[i].m_y;
        pixy.ccc.blocks[i].print();
      }
    }
    InfraredResult InfraredBall = InfraredSeeker::ReadAC();
  gyro.read();
  yaw = gyro.heading+ yawNew;
  if(yaw>179)yaw=179;
  else if(yaw<-179)yaw=-179;
  //  timee = millis();
  //  while (timee + 1000 > millis()) {
  //    gyro.read();
  //    yaw = gyro.heading;
  //    Move_deg(0, 100);
  //  }
  //  timee = millis();
  //  while (timee + 1000 > millis()) {
  //    gyro.read();
  //    yaw = gyro.heading;
  //    Move_deg(90, 100);
  //  }
  //  timee = millis();
  //  while (timee + 1000 > millis()) {
  //    gyro.read();
  //    yaw = gyro.heading;
  //    Move_deg(180, 100);
  //  }
  //  timee = millis();
  //  while (timee + 1000 > millis()) {
  //    gyro.read();
  //    yaw = gyro.heading;
  //    Move_deg(-90, 100);
  //  }
  //  Serial.println(yaw);
  if (zones[0] == 1) {
    Move_deg(0+((InfraredBall.Direction - 5)/abs(InfraredBall.Direction-5))*45, 200);
    dd += 1;
  }
  else if (zones[2] == 1) {
    Move_deg(180-((InfraredBall.Direction - 5)/abs(InfraredBall.Direction-5))*45, 200);
    dd += 1;
  }
  else {
    Move_deg(0, 0);
    dd = 0;
  }
  yawNew=2*(x-158);
  if (dd == 0) {
    if (InfraredBall.Direction != 1 && InfraredBall.Direction != 9 && InfraredBall.Direction != 0 && InfraredBall.Direction != 8 && InfraredBall.Direction != 2) {
      Move_deg(90, ((InfraredBall.Direction - 5)/abs(InfraredBall.Direction-5)) * 200);
    }
  } else if (zones[0] == 1 && zones[2] == 1) {
    if (zones[1] == 1) {
      if (InfraredBall.Direction > 5 || x>158) {
        Move_deg(120, (InfraredBall.Direction - 5) * 100);
      }
    } else if (zones[3] == 1) {
      if (InfraredBall.Direction < 5 || x<158) {
        Move_deg(120, (InfraredBall.Direction - 5) * 100);
      }
    } else {
      Move_deg(90, (InfraredBall.Direction - 5) * 60 * (-1));
    }
  }
  Serial.print(String(zones[0]) + ' ' + String(zones[1]) + ' ' + String(zones[2]) + ' ' + String(zones[3]) + ' ' + InfraredBall.Strength);
  Serial.println(" ");
}
