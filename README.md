# About
This is a self balancing robot made using Arduino Uno microcontroller.

# Hardware Required
|Hardware|Quantity|
|----|----|
|Arduino Uno|1x|
|Mpu-6050|1x|
| L298 H-bridge Motor Driver| 1x|
| 9v Batteries|2x|
| generic Motors|2x|

# Software Required
You  have  to install `Arduino Software` to burn this program into the arduino board.

# Getting Started
First clone the repository in your pc.
```bash 
git clone https://github.com/himanshu12345yadav/Self-Balancing-Robot.git
cd Self-Balancing-Robot
```

After cloning the project you can find two files **`selfBalancinRobot`** and **`mpu_calibration`** out of which the first file is responsible to control the robot. Flash this file in the arduino board using the Arduino Software. You can use the second file to find the balance point of the robot.

### How to get PID constants
Pid constants may vary for structure to structre. To find the best values of the PID constants run the **`selfBalancingRobot`** file and check the serial monitor and the find the PID contants by hit and trial method. The best PID value is one which make the robot to balancing easily without any gittering.

> For the hardware connections use the image provided in the project folder. In this i  have specified a bluetooth module but you can completely ignore it and do other wiring.

To understand the program, i recommend you to take a look at the register map of the mpu-6050 by [`clicking here`](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

`Happy hacking!`