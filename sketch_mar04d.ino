int MOTOR[4][2] = {{2, 5}, {4, 3}, {6, 9}, {8, 7}};
byte motorL, motorR;

#include <Wire.h>

byte DirectA  = 9;
byte RDirectA = 6;
byte powerA   = 21;
byte DirectB  = 8;
byte RDirectB = 7;
byte powerB =20;
byte DirectC  = 35;
byte RDirectC = 36;
byte powerC=22;
byte DirectD =  30;
byte RDirectD  =31;
byte powerD =23;
byte Mpower = 150;


/*
  IRSeeker.ino - A library/class for the HiTechnic IRSeeker V2 infrared sensor.
  Created by B. Blechschmidt, August 1, 2013.
  Released into the public domain.
*/


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

//void Move_deg(int angle,int power){
//    up = Yaw * kp + (Yaw - Yawold) * kd;
//    if(up > 50){
//      up = 50;
//    }else if(up < -50){
//      up = -50;
//    }else if(up < 20 and up > -20){
//      up = 0;
//    }
//    MotorA(cos(radians(-45 + angle)) * power + up);
//    MotorB(cos(radians(-135 + angle)) * -power + up);
//    MotorC(cos(radians(45 + angle)) * power - up);
//    MotorD(cos(radians(135 + angle)) * -power - up);
//    Serial.println(up);
//    Yawold = Yaw;
//    }

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

void motor(int Speed1, int Speed2, int Speed3, int Speed4)
{
  int sp[4] = {Speed1, Speed2, Speed3, Speed4};
  int SP;
  for (int i = 0; i < 4; i++)
  {
    SP = (abs(sp[i]) > 255) ? 255 : abs(sp[i]);
    if (sp[i] <= 0)
    {
      analogWrite(MOTOR[i][0], SP);
      analogWrite(MOTOR[i][1], 0);
    } else {
      analogWrite(MOTOR[i][1], SP);
      analogWrite(MOTOR[i][0], 0);
    }
  }
}

//void serialEvent() {
//  Serial4.write(0XA5);
//  Serial4.write(0X52);//send it for each read
//  while (Serial4.available()) {
//    Re_buf[counter] = (unsigned char)Serial4.read();
//    if (counter == 0 && Re_buf[0] != 0xAA) return;
//    counter++;
//    if (counter == 8)
//    {
//      counter = 0;
//      if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) // data package is correct
//      {
//        Yaw = (int16_t)(Re_buf[1] << 8 | Re_buf[2]) / 100.00;
//        Pitch = (int16_t)(Re_buf[3] << 8 | Re_buf[4]) / 100.00;
//        Roll = (int16_t)(Re_buf[5] << 8 | Re_buf[6]) / 100.00;
//      }
//    }
//  }
//}


void setup()
{
//  Serial.begin(9600);
  InfraredSeeker::Initialize();
//  pinMode(2, OUTPUT);
//  pinMode(3, OUTPUT);
//  pinMode(4, OUTPUT);
//  pinMode(5, OUTPUT);
//  pinMode(6, OUTPUT);
//  pinMode(7, OUTPUT);
//  pinMode(8, OUTPUT);
//  pinMode(9, OUTPUT);
//  //  kp = 1.7;
//  //  kd = 10;
//  pinMode(26, OUTPUT);
//  digitalWrite(26, HIGH);
//  Serial.begin(115200);
//  Serial4.begin(115200);
//  delay(2000);
//  Serial4.write(0XA5);
//  Serial4.write(0X54);
//  delay(2000);
//  Serial4.write(0XA5);
//  Serial4.write(0X52);
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
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //  t = millis();
}



void loop()
{
  InfraredResult InfraredBall = InfraredSeeker::ReadAC();
  motorL = (5 - InfraredBall.Direction) * 20 + Mpower;
  motorR = Mpower -(5 - InfraredBall.Direction ) * 17;
  MControl(motorR, motorR, motorL, motorL);
  Serial.println(String(InfraredBall.Direction) + "   -   "+String(motorL) +"      -       "+String(motorR));
  delay(10);
// digitalWrite(DirectA, HIGH);
// digitalWrite(RDirectA, LOW);
// analogWrite(powerA, 100);
}
