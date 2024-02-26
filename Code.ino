#include <Wire.h> 
#include <Servo.h> 
#include <SimpleKalmanFilter.h>

Servo myservoX;  
Servo myservoY; 

long accelX, accelY, accelZ; 
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float gyroX_cal, gyroY_cal, gyroZ_cal;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
int acc_calibration_value = 1000;                
float angle_acc;

long loop_timer;
int servoXpos=0;
int servoYpos=80;
int count = 0;
int t=0;
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup(){
  Serial.begin(9600);
  
  //myservoX.attach(9);   // pitch  
  //myservoY.attach(10);  // roll
  //myservoX.write(40);  
  //myservoY.write(80);
  Wire.begin(); 
  setupMPU();       // mpu6050 اتصال
  //delay(1000);
  pinMode(13,OUTPUT);
  Serial.println("caliberating MPU6050");  

  for(int i=0; i<2000; i++){
    if(i %125 == 0){Serial.print("."); }
    recordGyroRegisters();
    gyroX_cal += gyroX;
    gyroY_cal += gyroY;
    gyroZ_cal += gyroZ;
    delayMicroseconds(3700);
  }
  gyroX_cal /= 2000;
  gyroY_cal /= 2000;
  gyroZ_cal /= 2000;
  Serial.print("gyroX_cal: ");
  Serial.print(gyroX_cal);
  Serial.print("  gyroY_cal: ");
  Serial.print(gyroY_cal);
  Serial.print("  gyroZ_cal: ");
  Serial.print(gyroZ_cal);
  //delay(2000);
  loop_timer = micros();
}

void loop(){
  recordAccelRegisters();
  recordGyroRegisters();
  
  gyroX -= gyroX_cal;
  gyroY -= gyroY_cal;
  gyroZ -= gyroZ_cal;

  float gX = simpleKalmanFilter.updateEstimate(gyroX);//Kalman estimated
  float gY = simpleKalmanFilter.updateEstimate(gyroY);
  float gZ = simpleKalmanFilter.updateEstimate(gyroZ);

  //Serial.print(" | real = "); Serial.print(gyro_X);
  //Serial.print(" | kalman = "); Serial.println(gx);

  angle_roll += gX * 0.000122;
  angle_pitch += gY * 0.000122;

  angle_roll += angle_pitch * sin(gZ * 0.000002131);             
  angle_pitch -= angle_roll * sin(gZ * 0.000002131);   
//
  servoXpos = map(angle_pitch, 90.00,-90.00,0,180);
  servoYpos = map(-angle_roll, -90.00,90.00,0,180);

 // t = micros()-t;
  //Serial.println(t);
   //count++;
   
   while(micros() - loop_timer < 8000);{
    if(count==1){
    if(servoYpos >=0 && servoYpos <=180){
         myservoY.write(servoYpos-10);

        }
     }
    if(count==2){
      count=0;
      if(servoXpos >=0 && servoXpos <=180){
         myservoX.write(servoXpos-50 );
      }
     }
   }
   //
  loop_timer += 8000;
  }
  
void setupMPU(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
 /* Wire.beginTransmission(0b1101000);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(); */
}

void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000);  
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
}

void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}
