#include <DS3231.h>
#include <Wire.h>

#define SDA A0
#define SCL A1

DS3231  rtc(SDA, SCL);
const int MPU_addr=0x68;  // I2C address of t he MPU-6050
int16_t AcX,AcY,AcZ,Tmp;

void setup()
{
  Serial.begin(115200);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  rtc.begin();
  rtc.setDOW(WEDNESDAY);     // Set Day-of-Week to SUNDAY
  rtc.setTime(14, 0, 0);     // Set the time to 12:00:00 (24hr format)
  rtc.setDate(22, 2, 2017);   // Set the date to January 1st, 2014
}

void loop()
{
  // Set I2C and send accel
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.println(AcZ);

  // Which phase is on the ground
  if(AcX >= -10000){
    if(AcX < 10000){
       if(AcZ >= 10000){
          Serial.println("   C   ");
          // Send Day-of-Week
          Serial.print(rtc.getDOWStr());
          Serial.print(" ");
  
          // Send date
          Serial.print(rtc.getDateStr());
          Serial.print(" -- ");

          // Send time
          Serial.print(rtc.getTimeStr());

          // Send temperature
          Serial.print(" Tmp = "); Serial.println(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
       }
       else{
          Serial.println("   A   ");
          Serial.println("This is alarm function.");
       }
    }
    else{
       Serial.println("   B   ");
       Serial.println("This is weather notice function.");
    }
  }
  else{
    Serial.println("   D   ");
    Serial.println("This is fairing function.");
  }

  // Wait one second before repeating :)
  delay (1000);
}
