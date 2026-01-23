# Westview-Capstone 2026
**Self balancing robot**
![Alt Text](https://github.com/Chase-520/Westview-Capstone/blob/main/images/display.gif)

# Summary
As a software-oriented engineer, the biggest challenge in this project was the mechanical design. Over the past six weeks, I spent a significant amount of time learning OnShape and designing the robot. Logistics and the Bill of Materials (BOM) also taught me about engineering aspects beyond just technical skills.

# Concept explained
This capstone project is inspired by the RoboMaster robotics competition in China. Robots in that competition are required to complete missions via remote control, and the ability to jump provides a significant competitive advantage. My capstone project aims to build a robot that can balance itself on only two wheels and avoid obstacles by jumping.

The sensor used in this project is an IMU (Inertial Measurement Unit), which provides the estimated orientation of the robot in Euler angles (yaw, pitch, roll) or quaternions. The software running on the ESP32 microcontroller uses a simple PID (Proportional, Integral, Derivative) control loop that constantly calculates the error between the desired pitch angle of the robot and the actual pitch angle (0 degrees compared to -3 degrees).

# Hardware
- ESP32
- Adafruit 9 axis imu
- Adafruit GPIO extension board
- Pololu dual motor driver * 2
- N20 motor with encoder * 4
- Rev 12v Battery
- 2 axis joystick * 2
**Bill of material can be found [here](https://github.com/Chase-520/Westview-Capstone/blob/main/BOM.xlsx)**

![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/spurgearver.png)

# Software
- Adafruit BN085 library

# First Prototype
![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/prototype.png)

# Electrical Schematics
![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/electrical_schematics.png)

**ESP32 pinout**
![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/pinout.png)
# Software Schematics
![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/Software_schematics.png)

# Future plan
- As for now, the robot's mechanical design is the weakest portion, I'm planning to use spur gear to distribute the load from the motor shaft to the motor mount and the 3D printed box.
- Specifically for mechanical design, the robot doesn't have the freedom to dynamically align the contact point of the wheel to the center of mass, consider a different leg structure to give more freedom.
- On software controll loop, controller is unit tested but not integrated, the control loop is simple but hard to tune. Consider using a cascade pid loop to have a more responsive control
- Consider addng a feedforward control method (MPC, LQR) to optimize control response.

# Last updated
01/12/2026
