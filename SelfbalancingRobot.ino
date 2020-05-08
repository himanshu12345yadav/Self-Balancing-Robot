// I have made this program from scratch without any external library so that it is easy to understand and implement. It does its job quite well but sometimes if the PID contants or if the mechanical structure of the robot is not close to perfect then the robot may get some difficulty in balancing. For better balancing try to keep the center of graviy of robot close to ground so that it will generate a lesser torque and able to balance itsef. Although PID algorithm is also working to help the robot achieve its stable position but the mechanical structure is more responsible for balancing the robot.To set the PID constants you can take help of serial monitor and find out the best values of PID constants(Kp,ki,kd) for better stabilisation.

// Made by Himanshu Yadav

#include<Wire.h>
// assuming X as pitch, Y as yaw, z as roll axis 
float AccelX,AccelY,AccelZ ,GyroX,GyroY,GyroZ,Accel_X,Accel_Y,Accel_Z;
int16_t rawGyroX,rawGyroY,rawGyroZ,rawAccelX,rawAccelY,rawAccelZ;
float Pitch,Yaw,Roll;
float elapsed_Time,prev_Time,time;
float error,Pid_P,Pid_I,Pid_D,Previous_error,PID;
const float Kp = 35;
const float Ki= 0.8;
const float Kd= 0.8;
float Gyro_X ,Gyro_Y ,Gyro_Z ;
float Yaw_Set_Point=-3.25;
uint8_t EN1=9,EN2=3,IN1=7,IN2=6,IN3=5,IN4=4;
 // mpu slave address b1101000 (0x68) check section 9.2  datasheet
 //int mpuAddress =  B1101000;

void setup() {
 Serial.begin(9600);
 Wire.begin();
 mpuSetup();
  // taking accelerometer data for  coupling with the gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  while(Wire.available()<6);
  rawAccelX = Wire.read()<<8|Wire.read();
  rawAccelY = Wire.read()<<8|Wire.read();
  rawAccelZ = Wire.read()<<8|Wire.read();
  // Accelerometer sensitivity is 16384 for + - 2g Full scale
  
 Accel_X = (asin(rawAccelX/16384.0))*57.324;
 Accel_Y = (asin(rawAccelY/16384.0))*57.324;
 Accel_Z = (asin(rawAccelZ/16384.0))*57.324;
 
Gyro_X = Accel_Y;
Gyro_Y = Accel_X;
Gyro_Z = Accel_Z;


}

void loop() {
mpuData();
Calculate_Pid();
Control_Motors();
Serial.print("Pid ");
Serial.println(PID);
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
  AccelX = rawAccelX/16384.0 ;
  AccelY = rawAccelY/16384.0 ;
  AccelZ = rawAccelZ/16384.0 ;
  // checking the bounds 
  if(AccelX >1)
  AccelX = 1;
  if(AccelX <-1)
  AccelX = -1;
  if(AccelY >1)
  AccelY = 1;
  if(AccelY <-1)
  AccelY = -1;
  
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
 
//if(GyroX>-0.80&&GyroX)
//  GyroX =0;
//  

 // assuming the initial rest position of the sensor is horizontal
  // converting radians to degrees
  
 prev_Time = time;
 time = millis();
 elapsed_Time = (time - prev_Time)/1000;
 Gyro_X += GyroX*elapsed_Time ;
 Gyro_Y += GyroY*elapsed_Time ;
 Gyro_Z += GyroZ*elapsed_Time ;
 
 // convert radian to degrees  
 //  Calculating Roll and Pitch from the accelerometer data
// Accel_X = (atan(AccelY / sqrt(pow(AccelX, 2) + pow(AccelZ, 2))) * 180 / PI); // AccErrorX ~(0.58) 
// Accel_Y = (atan(-1 * AccelX / sqrt(pow(AccelY, 2) + pow(AccelZ, 2))) * 180 / PI);  //+ 1.58 AccErrorY ~(-1.58)

 Accel_X = (asin(AccelX/1))*57.324 ;
 Accel_Y = (asin(AccelY/1))*57.324 ;
 Accel_Z = (asin(AccelZ/1))*57.324 ;

// assuming X as pitch Y as yaw z as roll axis 
// after applying te complimentay filters
 Pitch = 0.02*Accel_X + 0.98*Gyro_Y;
 Yaw = 0.02*Accel_Y + 0.98*Gyro_X;
 Roll = 0.02*Accel_Z+ 0.98*Gyro_Z;

  }
  void Calculate_Pid(){
    //  pid for the yaw axis
    error = Yaw - Yaw_Set_Point;
    Pid_P = Kp*error;
  
    Pid_I += Ki*error;
    
    Pid_D = Kd*((error - Previous_error)/elapsed_Time);
    Previous_error = error;
    PID = Pid_P + Pid_I + Pid_D;   
    }
  void Control_Motors(){
    // using  AnalogWrite takes value from 0 to 255.  
    // pid positive move backward 
    // pid negative move forwards
    if(PID > 255)
    PID =255;
    if(PID < -255)
    PID =-255;
       // forwards is opposite to the wires side
      if(PID > 0)
      moveForwards(PID);
      else
      moveBackwards((-1)*PID);    
      }
  void moveForwards(int a){
    analogWrite(EN1,a);
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(EN2,a);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);   
    }
  void moveBackwards(int b){
    analogWrite(EN1,b);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(EN2,b);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH); 
    }
 
