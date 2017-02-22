//MPU6050
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Timer.h"

Timer t;
MPU6050 accelgyro;

#define uchar unsigned char
#define uint unsigned int

#define PI 3.141592653

#define ledPin 13

//raw data of acceleration sensor
int16_t ax, ay, az;
//raw data of MPU6050
int16_t gx, gy, gz;

float ACCX, ACCY, ACCZ;

float GYROX, GYROY, GYROZ;

float accxPitch, accyPitch, acczPitch;

float gyroxAngle, gyroyAngle, gyrozAngle;

uint32_t time;

float timeChange = 20;
float dt = timeChange * 0.001;
//first-order filter
float K1 = 0.05;
float angle1;
//second order filter
float K2 =0.2; 
float x1,x2,y1;
float angle2;
//kalmanFilter
float angle, angle_dot;
float angle_0, angle_dot_0;

long axOffset = 0, ayOffset = 0, azOffset = 0;
long gxOffset = 0, gyOffset = 0, gzOffset = 0;


float P[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; 
float R_angle=0.5 ,C_0 = 1; 
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;


void setup() 
{ 
  pinMode(ledPin, OUTPUT);
  Wire.begin();
  Serial.begin(9600); 
  accelgyro.initialize();
  digitalWrite(ledPin, HIGH);
  simpleCalibration();
  digitalWrite(ledPin, LOW);

  int tickEvent1=t.every(timeChange, getAngle);//本语句执行以后timeChange毫秒执行回调函数getangle
  int tickEvent2=t.every(500, serialOutput) ;//本语句执行以后50毫秒执行回调函数printout，串口输出
}


void loop() 
{ 
  t.update();//time system
}


void mpuGetRaw(void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void dataQuantize(void)
{
  ACCX =  ( ax - axOffset ) / 16384.00;
  ACCY =  ( ay - ayOffset ) / 16384.00;
  ACCZ =  ( az - azOffset ) / 16384.00;
  GYROX = ( gx - gxOffset ) / 131.00;
  GYROY = ( gy - gyOffset ) / 131.00;
  GYROZ = ( gz - gzOffset ) / 131.00;
}

void serialOutput(void)
{
  Serial.print("a/g:\t");
  Serial.print(ACCX); Serial.print("\t");
  Serial.print(ACCY); Serial.print("\t");
  Serial.print(ACCZ); Serial.print("\t");
  Serial.print(GYROX); Serial.print("\t");
  Serial.print(GYROY); Serial.print("\t");
  Serial.println(GYROZ);
  Serial.print(accxPitch);Serial.print("\t");
  Serial.print(accyPitch);Serial.print("\t");
  Serial.println(acczPitch);
  Serial.print(gyroxAngle);Serial.print("\t");
  Serial.print(gyroyAngle);Serial.print("\t");
  Serial.println(gyrozAngle);
  Serial.print(angle1);Serial.print("\t");
  Serial.print(angle2);Serial.print("\t");
  Serial.println(angle);
  Serial.println();
}

void accGetPitch(void)
{
  
  acczPitch = atan(sqrt((ACCX * ACCX + ACCY * ACCY)) / ACCZ) * 180 / PI;
  
  accxPitch = atan(ACCX / sqrt((ACCY * ACCY + ACCZ * ACCZ))) * 180 / PI;
  
  accyPitch = atan(ACCY / sqrt((ACCX * ACCX + ACCZ * ACCZ)))* 180 / PI;
}

void gyroGetAngle(void)
{ 
  gyroxAngle = gyroxAngle + GYROX * ((double)(micros() - time) / 1000000);
  gyroyAngle = gyroyAngle + GYROY * ((double)(micros() - time) / 1000000);
  gyrozAngle = gyrozAngle + GYROZ * ((double)(micros() - time) / 1000000);
  
  time = micros();
}

void complementaryfilter_gyroGetAngle(float angle_m, float gyro_m)
{
  angle1 = K1 * angle_m+ (1-K1) * (angle1 + gyro_m * dt);
}

void complementaryfilter2_gyroGetAngle(float angle_m, float gyro_m)
{
  x1=(angle_m-angle2)*(1-K2)*(1-K2);
  y1=y1+x1*dt;
  x2=y1+2*(1-K2)*(angle_m-angle2)+gyro_m;
  angle2=angle2+ x2*dt;
}
//kalmanFilter
void kalmanFilter_gyroGetAngle(double angle_m, double gyro_m)
{
  angle += (gyro_m-q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] =- P[1][1];
  Pdot[2] =- P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; 
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m-q_bias;
}
void getAngle(void)
{
  mpuGetRaw();
  dataQuantize();
  accGetPitch();
  gyroGetAngle();
  complementaryfilter_gyroGetAngle(accxPitch, GYROY);
  complementaryfilter2_gyroGetAngle(accxPitch, GYROY);
  kalmanFilter_gyroGetAngle(accxPitch, GYROY);
}

void simpleCalibration(void)
{
  for(int16_t i = 0; i < 1024; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axOffset = axOffset + ax;
    ayOffset = ayOffset + ay;
    azOffset = azOffset + az;
    gxOffset = gxOffset + gx;
    gyOffset = gyOffset + gy;
    gzOffset = gzOffset + gz;
  }
  axOffset = axOffset / 1024;
  ayOffset = ayOffset / 1024;
  azOffset = azOffset / 1024 - 16384;
  gxOffset = gxOffset / 1024;
  gyOffset = gyOffset / 1024;
  gzOffset = gzOffset / 1024;
}

