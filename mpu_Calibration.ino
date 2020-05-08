#include<Wire.h>
// assuming X as pitch, Y as yaw, z as roll axis 
float AccelX,AccelY,AccelZ ,GyroX,GyroY,GyroZ,Accel_X,Accel_Y,Accel_Z;
float rawGyroX,rawGyroY,rawGyroZ,rawAccelX,rawAccelY,rawAccelZ;
float accelSetPoint_X,accelSetPoint_Y,accelSetPoint_Z,gyroSetPoint_X,gyroSetPoint_Y,gyroSetPoint_Z;
float accelSetPointX,accelSetPointY,accelSetPointZ,gyroSetPointX,gyroSetPointY,gyroSetPointZ;
float Pitch,Yaw,Roll;
float elapsed_Time,prev_Time,time;
float Gyro_X ,Gyro_Y ,Gyro_Z;
float Pitch_Set_Point, Roll_Set_Point,Yaw_Set_Point;
 
void setup(){
 Serial.begin(9600);
 Serial.println("Calibrating Mpu....");
 Wire.begin();
 mpuSetup();
 
 // taking accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  while(Wire.available()<6);
  rawAccelX = Wire.read()<<8|Wire.read();
  rawAccelY = Wire.read()<<8|Wire.read();
  rawAccelZ = Wire.read()<<8|Wire.read();
  
  // Accelerometer sensitivity is 16384 for + - 2g Full scale
  AccelX = rawAccelX/16384.0;
  AccelY = rawAccelY/16384.0;
  AccelZ = rawAccelZ/16384.0;
 Accel_X = (asin(AccelX/1))*57.324;
 Accel_Y = (asin(AccelY/1))*57.324;
 Accel_Z = (asin(AccelZ/1))*57.324;

Gyro_X = Accel_Y;
Gyro_Y = Accel_X;
Gyro_Z = Accel_Z;


calibratingMpu();
 Serial.print("Gyro Set Point X : ");
 Serial.print(gyroSetPointX);
 Serial.print("| Gyro Set Point Y : ");
 Serial.print(gyroSetPointY);
 Serial.print("|Gyro Set Point Z  : ");
 Serial.println(gyroSetPointZ);
 Serial.print("|Accel Set Point X : ");
 Serial.print(accelSetPointX);
 Serial.print("|Accel Set Point Y : ");
 Serial.print(accelSetPointY);
 Serial.print("|Accel Set Point Z  : ");
 Serial.print(accelSetPointZ);

  

}

void loop() {
// mpuData();
// Serial.println(Yaw);
}
void mpuSetup(){
 // waking the mpu 6050
 Wire.beginTransmission(0x68);
 Wire.write(0x6B);
 Wire.write(0b00000000);
 Wire.endTransmission();
  // setting up accelerometer to + - 2g of full scale range 
 Wire.beginTransmission(0x68);
 Wire.write(0x1C);
 Wire.write(0b00000000);
 Wire.endTransmission();
 //setting up gyro to + - 250 degrees/s
 Wire.beginTransmission(0x68);
 Wire.write(0x1B);
 Wire.write(0b00000000);
 Wire.endTransmission();  
 
  }
  
  void mpuData(){
    
 prev_Time = time;
 time = millis();
 elapsed_Time = (time - prev_Time)/1000;
 
  // taking accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  while(Wire.available()<6);
  rawAccelX = Wire.read()<<8|Wire.read();
  rawAccelY = Wire.read()<<8|Wire.read();
  rawAccelZ = Wire.read()<<8|Wire.read();
  
  // Accelerometer sensitivity is 16384 for + - 2g Full scale
  AccelX = rawAccelX/16384.0;
  AccelY = rawAccelY/16384.0;
  AccelZ = rawAccelZ/16384.0;
 // checking the bounds
//  if(AccelX >1)
//  AccelX = 1;
//  if(AccelX <-1)
//  AccelX = -1;
//  if(AccelY >1)
//  AccelY = 1;
//  if(AccelY <-1)
//  AccelY = -1;
//  
  // taking gyro data
 Wire.beginTransmission(0x68);
 Wire.write(0x43);
 Wire.endTransmission();
 Wire.requestFrom(0x68,6);
 while(Wire.available()<6);
  rawGyroX = Wire.read()<<8|Wire.read();
  rawGyroY = Wire.read()<<8|Wire.read();
  rawGyroZ = Wire.read()<<8|Wire.read();
  
  
  // gyro sensitivity is 131 for + - 250  degrees/s full scale speeds
  
 GyroX = rawGyroX/131.0;
 GyroY = rawGyroY/131.0;
 GyroZ = rawGyroZ/131.0;
  // assuming the initial rest position of the sensor is horizontal
  // converting radians to degrees
  
 Gyro_X += GyroX*elapsed_Time;
 Gyro_Y += GyroY*elapsed_Time;
 Gyro_Z += GyroZ*elapsed_Time;

// Accel_X = (atan(AccelY / sqrt(pow(AccelX, 2) + pow(AccelZ, 2))) * 180 / PI); //- 0.58 AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
// Accel_Y = (atan(-1 * AccelX / sqrt(pow(AccelY, 2) + pow(AccelZ, 2))) * 180 / PI); //+ 1.58 AccErrorY ~(-1.58)

 // convert radian to degrees  
 Accel_X = (asin(AccelX/1))*57.324;
 Accel_Y = (asin(AccelY/1))*57.324;
 Accel_Z = (asin(AccelZ/1))*57.324;
// 
   // assuming X as pitch, Y as yaw, z as roll axis 
   //after applying te complimentay filters
 Pitch = 0.02* Accel_X + 0.98*Gyro_Y;
 Yaw = 0.02*Accel_Y  + 0.98*Gyro_X;
 Roll = 0.02*Accel_Z + 0.98*Gyro_Z;
  
  }
void calibratingMpu(){
   
    for(int i =1;i<=500;i++){
       mpuData();
      accelSetPointX += rawAccelX;
      accelSetPointY += rawAccelY;
      accelSetPointZ += rawAccelZ;
      gyroSetPointX += rawGyroY;
      gyroSetPointY += rawGyroX;
      gyroSetPointZ += rawGyroZ; 
           
     }
     accelSetPointX /=500;
     accelSetPointY /=500;
     accelSetPointZ /=500;
     gyroSetPointX /= 500;
     gyroSetPointY /= 500;
     gyroSetPointZ /= 500;
      // filtering the calibrating data
//        Pitch_Set_Point = 0.02*accelSetPointX/500 + 0.98*gyroSetPointX/500;
//        Yaw_Set_Point = 0.02*accelSetPointY/500 +  0.98*gyroSetPointY/500;
//        Roll_Set_Point = 0.02*accelSetPointZ/500 + 0.98*gyroSetPointZ/500;
       
           
    }
   
