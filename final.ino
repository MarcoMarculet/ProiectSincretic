#include <Wire.h>
#include <math.h>
#define pin1 9
#define pin2 10

int inputAngle = 20;
double angle;
double error, last_error = 0, total_error = 0;
double kP = 6;
double kI = 0.007;
double kD = 0;
int T = 10;
int last_time = 0;
double PID_signal;

double pM2=0, pM1=0;
double procent;

const uint8_t MPU6050SlaveAddress = 0x68;
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ;
double Ax, Ay, Az;

void PID()
{
  unsigned long _time = millis();
  int d_time = _time - last_time;

  if (d_time >= T)
  {
    angle = Angle();
    error = inputAngle - angle;
    total_error += error;

    double deriv_error = error - last_error;

    PID_signal = kP * error + kI * T * total_error + (kD * deriv_error) / T;

    if (PID_signal >= 255)
      PID_signal = 255;
    else if (PID_signal <= -255)
      PID_signal = -255;

    last_error = error;
    last_time = _time;
  }
}

void Motor()
{
  int aux;
  procent = 1 / (1 + exp(-error * 1.2));

  if ( PID_signal >= 0)
  {
    pM2 = procent * PID_signal;
    //pM2 = 150;
    //pM1 = 0;
    pM1 = (1 - procent) * PID_signal;
  }
  else
  {
    //pM2 = 0;
    //pM1 = 150;
    pM2 = procent * abs(PID_signal);
    pM1 = (1 - procent) * abs(PID_signal);
  }
  if ( pM1 < 80) 
    pM1 = 80;
  if ( pM2 < 80) 
    pM2 = 80;
  analogWrite(pin1, (int)pM1);
  analogWrite(pin2, (int)pM2);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  MPU6050_Init();
  delay(500);
}
int sw=1;
void loop() {
    Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
    Ax = (double)AccelX/AccelScaleFactor;
    Ay = (double)AccelY/AccelScaleFactor;
    Az = (double)AccelZ/AccelScaleFactor;
    //angle = Angle();
    PID();
    Motor();
    /*
    if(angle < 0){
      analogWrite(pin2,150);
      analogWrite(pin1,0);
    }
    else{
      analogWrite(pin2,0);
      analogWrite(pin1,150);
    }
    */
    Serial.println(angle);
    //Serial.print(" ");
    //Serial.println(PID_signal);
    delay(100);
}

double Angle()
{
  double _angle;

  _angle = (atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180.0) / PI;
  //Serial.println(angle);
  return _angle;
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
