#define pi 3.14159265
#include <ESP32Encoder.h>
#include <Wire.h>
#include <math.h>

#define TCAADDR 0x70
// упрощеный I2C адрес нашего гироскопа/акселерометра MPU-6050.
const int MPU_addr = 0x68;
// переменные для хранения данных возвращаемых прибором.
int16_t AcX, AcY, AcZ;

unsigned long enc_t = 0;
unsigned long timer = 0;

void tcaselect(uint8_t i);

//void Calc_CompensatorZ(uint8_t adr, unsigned long mill_sec);
void Data_mpu6050(uint8_t adr);

ESP32Encoder encoder;
unsigned long long t0 = 0;
long enc = 0, encPrev = 0;
const int ledPin = 15;
const int ledPin2 = 14;
const int red = 17;
const int blue = 16;
const int freq = 20000;

const int ledChannel = 1;
const int redChannel = 3;
const int blueChannel = 4;
const int resolution = 8;
float w = 2*pi*1;
double duty, vel;
float set_speed = 0, pv_speed = 0, e_speed, pulse = 0, e_speed_pre = 0;;
int kp = 200, kd = 200;
float xn = 0.0, xn1 = 0.0,  yn1 = 0.0;
float y;
String rx_str = "";
void PD(){
  //if (micros() - enc_t >=5000){
  //Serial.println(set_speed);
  
  if (set_speed >= 0){
    enc = encoder.getCount();
    vel = (enc - encPrev)/0.17;
    
    y = 0.7285*yn1 + 0.1358*vel + 0.1358*xn1;
    xn1 = vel;
    yn1 = y;
    pv_speed = y;
    e_speed = (set_speed - pv_speed) / set_speed;
    pulse = e_speed * kp + (e_speed - e_speed_pre)*kd;
    e_speed_pre = e_speed;
    //Serial.println(pulse);
    if (pulse <= 255 && pulse > 0){
      ledcWrite(ledChannel, (int)pulse);
      ledcWrite(ledChannel + 1, 0);
    }
    else {
      if (pulse > 255){
        ledcWrite(ledChannel, 255);
        ledcWrite(ledChannel + 1, 0);
      } else {
        ledcWrite(ledChannel, 0);
        ledcWrite(ledChannel + 1, 0);
      }
    }
  } else {
    enc = encoder.getCount();
    vel = (enc - encPrev) / (-0.17);
    
    y = 0.7285*yn1 + 0.1358*vel + 0.1358*xn1;
    xn1 = vel;
    yn1 = y;
    pv_speed = y;
    e_speed = (-1*set_speed - pv_speed) / (-1*set_speed);
    pulse = e_speed * kp + (e_speed - e_speed_pre)*kd;
    e_speed_pre = e_speed;
    if (pulse <= 255 && pulse > 0){ 
      ledcWrite(ledChannel + 1, (int)pulse);
      ledcWrite(ledChannel, 0);
    }
    else{
      if (pulse>255){
        ledcWrite(ledChannel + 1, 255);
        ledcWrite(ledChannel, 0);
      } else {
        ledcWrite(ledChannel + 1, 0);
        ledcWrite(ledChannel, 0);
      }
    }
  }
  //Serial.println(y);
  encPrev = enc;
  }
 // Serial.print(y*1000); Serial.print(" "); Serial.print(set_speed*1000);
//}

void setup(){
  Serial.begin(1000000);
  Serial.setTimeout(5);
  Wire.begin(22, 23, (uint32_t)819200);//(22,23); //SDA //SCL
  //ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachSingleEdge(4, 5);
  encoder.clearCount();
  encoder.setFilter(1023);
 
  ledcSetup(ledChannel, freq, resolution);
  ledcSetup(redChannel, freq, resolution);
  ledcSetup(blueChannel, freq, resolution);
  ledcSetup(ledChannel+1, freq, resolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcAttachPin(ledPin2, ledChannel+1);
  ledcAttachPin(red, redChannel);
  ledcAttachPin(blue, blueChannel);
}
unsigned long long counter = 0;
void loop(){
  // increase the LED brightness
  //for (int i = 0; i < 1001; i++){
  t0 = micros();
  if (Serial.available()){
    String rx_str = Serial.readStringUntil('\r\n');
    set_speed = rx_str.toInt();
  }
  int pwm = 255*sin(2*pi*0.1*millis()/1000.0);
  if (set_speed <= 0){
    ledcWrite(redChannel, abs(pwm));
    ledcWrite(blueChannel, 255);
  } else {
    ledcWrite(redChannel, 255);
    ledcWrite(blueChannel, abs(pwm));
  }

  //Serial.println(set_speed);
  
  PD();
  Data_mpu6050(0);
  
 
    Serial.print("  "); Serial.print(t0/1000.0);  Serial.print("  ");
    Serial.print("  "); Serial.print(AcX);Serial.print("  ");
    Serial.print("  "); Serial.print(AcY);Serial.print("  ");
    Serial.print("  "); Serial.print(AcZ);Serial.print("  ");
//    Serial.print("  "); Serial.print(fil_yn); Serial.print("  ");
//
//
  Data_mpu6050(1);
    Serial.print("  "); Serial.print(AcX);Serial.print("  ");
    Serial.print("  "); Serial.print(AcY);Serial.print("  ");
    Serial.print("  "); Serial.print(AcZ);Serial.print("  ");
//    Serial.print("  "); Serial.print(fil_yn2); Serial.print("  ");

  Data_mpu6050(2);

   Serial.print("  "); Serial.print(AcX);Serial.print("  ");
    Serial.print("  "); Serial.print(AcY);Serial.print("  ");
    Serial.print("  "); Serial.print(AcZ);Serial.print("  ");
//    Serial.print("  "); Serial.print(fil_yn3); Serial.print("  ");

  Data_mpu6050(3);

    Serial.print("  "); Serial.print(AcX);Serial.print("  ");
    Serial.print("  "); Serial.print(AcY);Serial.print("  ");
    Serial.print("  "); Serial.println(AcZ);
  
  //}
  delayMicroseconds(5000 - micros() + t0);
  //Serial.println(counter / 1000.0);
  //counter = 0;
}


void Data_mpu6050(uint8_t adr)
{
  tcaselect(adr); //Выбор переключаемого адреса
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  //Готовим для чтения регистры  с адреса 0x3B.
  Wire.endTransmission(false);
  // Запрос 14 регистров.
  Wire.requestFrom(MPU_addr, 14, true);
  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcX = Wire.read() << 8 | Wire.read();
  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcY = Wire.read() << 8 | Wire.read();
  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();
  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  //  Tmp = Wire.read() << 8 | Wire.read();
  //  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  //  GyX = Wire.read() << 8 | Wire.read();
  //  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  //  GyY = Wire.read() << 8 | Wire.read();
  //  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  //  GyZ = Wire.read() << 8 | Wire.read();
}

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
