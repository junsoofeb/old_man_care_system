#include<Wire.h>
#include <I2Cdev.h>
#include <SoftwareSerial.h> 
#include "TinyGPS++.h"


int sensorPin = 0; //Analog 0 번에 센서핀 연결

const int MPU = 0x68;  // I2C address of the MPU-6050

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;


static const int RXPin = 10, TXPin = 11; 
static const uint32_t SIOBaud = 9600; 
static const uint32_t GPSBaud = 9600; 
 
// The TinyGPS++ object 
TinyGPSPlus gps; 
 
// The serial connection to the GPS device 
SoftwareSerial ss(RXPin, TXPin); 
 
  
// 프로그램 시작 - 초기화 작업
void setup()
{
  Serial.begin(9600);     // 시리얼 통신 초기화

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
 
  // gyro scale
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  //
  Wire.write(0xF8);     //
  Wire.endTransmission(true);
 
  // acc scale
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  //
  Wire.write(0xF8);     //
  Wire.endTransmission(true);
 
 
  Serial.begin(SIOBaud); 
  ss.begin(GPSBaud); 
 
}
 
void loop()
{
  Serial.println("start");
  //전압의 변화값을 센서핀으로 부터 읽음
 int reading = analogRead(sensorPin);  
 
 //전압값을 읽음, 센서에 공급되는 전압이 5V면 읽은값x5.0 이고 3.3V 이면 3.3을 곱해준다
 float voltage = reading * 5.0;
 voltage /= 1024.0; 

 
 //전압값(mV)으로 온도 구함 (섭씨) , 시리얼 모니터 출력
 float temperatureC = (voltage - 0.5) * 100 ;
 Serial.println(temperatureC);
  
  
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);  // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
  /*
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
    delay(333);
  */
 
 // xyz
  int xAxis = (AcX);
  int yAxis = (AcY);
  int zAxis = (AcZ);
  
   
  //int xAxis = (AcX - 1090);
  //int yAxis = (AcY - 930);
  //int zAxis = (AcZ - 1000);
  
  //Serial.print(xAxis);
  //Serial.print(" ");
  //Serial.print(yAxis);
  //Serial.print(" ");
  //Serial.print(zAxis);
  //Serial.println(" ");
 
  Serial.println(xAxis);
  Serial.println(yAxis);
  Serial.println(zAxis);
 
 
 
 
 
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002; 
 
  //printInt(gps.satellites.value(), gps.satellites.isValid(), 5); 
  //printInt(gps.hdop.value(), gps.hdop.isValid(), 5); 
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  Serial.println(); 
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  Serial.println(); 
  //printInt(gps.location.age(), gps.location.isValid(), 5); 
  printDateTime(gps.date, gps.time); 
  //printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2); 
  //printFloat(gps.course.deg(), gps.course.isValid(), 7, 2); 
  //printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2); 
  //printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6); 
 
  unsigned long distanceKmToLondon = 
    (unsigned long)TinyGPSPlus::distanceBetween( 
      gps.location.lat(), 
      gps.location.lng(), 
      LONDON_LAT,  
      LONDON_LON) / 1000; 
  //printInt(distanceKmToLondon, gps.location.isValid(), 9); 
 
  double courseToLondon = 
    TinyGPSPlus::courseTo( 
      gps.location.lat(), 
      gps.location.lng(), 
      LONDON_LAT,  
      LONDON_LON); 
 
  //printFloat(courseToLondon, gps.location.isValid(), 7, 2); 
 
  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon); 
 
 // printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6); 
 
  //printInt(gps.charsProcessed(), true, 6); 
  //printInt(gps.sentencesWithFix(), true, 10); 
  //printInt(gps.failedChecksum(), true, 9); 
  Serial.println(); 
   
  smartDelay(1000); 
 
  if (millis() > 5000 && gps.charsProcessed() < 10) 
    Serial.println(F("No GPS data received: check wiring")); 
 
 
 
 
 
  //delay(1000);
}

 
// This custom version of delay() ensures that the gps object 
// is being "fed". 
static void smartDelay(unsigned long ms) 
{ 
  unsigned long start = millis(); 
  do  
  { 
    while (ss.available()) 
      gps.encode(ss.read()); 
  } while (millis() - start < ms); 
} 
 
static void printFloat(float val, bool valid, int len, int prec) 
{ 
  if (!valid) 
  { 
    while (len-- > 1) 
      Serial.print('*'); 
    Serial.print(' '); 
  } 
  else 
  { 
    Serial.print(val, prec); 
    int vi = abs((int)val); 
    int flen = prec + (val < 0.0 ? 2 : 1); // . and - 
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1; 
    for (int i=flen; i<len; ++i) 
      Serial.print(' '); 
  } 
  smartDelay(0); 
} 
 
static void printInt(unsigned long val, bool valid, int len) 
{ 
  char sz[32] = "*****************"; 
  if (valid) 
    sprintf(sz, "%ld", val); 
  sz[len] = 0; 
  for (int i=strlen(sz); i<len; ++i) 
    sz[i] = ' '; 
  if (len > 0)  
    sz[len-1] = ' '; 
  Serial.print(sz); 
  smartDelay(0); 
} 
 
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) 
{ 
  if (!d.isValid()) 
  { 
    Serial.print(F("********** ")); 
  } 
  else 
  { 
    char sz[32]; 
    sprintf(sz, "%02d/%02d/%02d\n", d.month(), d.day(), d.year()); 
    Serial.print(sz); 
  } 
   
  if (!t.isValid()) 
  { 
    Serial.print(F("******** ")); 
  } 
  else 
  { 
    char sz[32]; 
    sprintf(sz, "%02d:%02d:%02d", t.hour() + 9, t.minute(), t.second()); 
    Serial.print(sz); 
  } 
 
  //printInt(d.age(), d.isValid(), 5); 
  smartDelay(0); 
} 
 
static void printStr(const char *str, int len) 
{ 
  int slen = strlen(str); 
  for (int i=0; i<len; ++i) 
    Serial.print(i<slen ? str[i] : ' '); 
  smartDelay(0); 
} 
