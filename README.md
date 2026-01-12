# Westview-Capstone 2026
Self balancing robot with two wheels
![Alt Text](https://github.com/Chase-520/Westview-Capstone/blob/main/images/milestone.gif)

# Summary
As a software orienated engineer, the biggest challeng on this challeng would be the mechanical design. In the past 6 weeks, I spent significant amount of my time learning OnShape and design the robot.Logistic, BOM (Bill of material) also taught me engineering beyond technical.

# Concept explained
- This capstone project is inspired by a robotics competition in China named RoboMaster. Robots in that competition are required to complete missions through remote control, and being able to jump give the robot significant advantage in the competitoin. My capstone project aims to build a robot that can balance itself with only two wheels and being able to avoid obstacles through jumping.
- The sensor used in this project is an IMU (inertia measurement unit), which can give the estimated orientation of the robot in yaw, pitch, roll for Euler angles or in quaterions. The software load in the micro controller ESP32 used a simple PID (Proportion, Integration, Derivation) controll loop that constantly calculate the error between desire pitch angle of the robot and the actual pitch angle (0 degrees copare to -3 degrees).

# Hardward
- ESP32
- Adafruit 9 axis imu
- Adafruit GPIO extension board
- Pololu dual motor driver * 2
- N20 motor with encoder * 4
- Rev 12v Battery
- 2 axis joystick * 2

# Software
- Adafruit BN085 library


# First Prototype
![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/prototype.png)

# Electrical Schematics
![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/Electrical_schematics.png)
# Software Schematics
![image](https://github.com/Chase-520/Westview-Capstone/blob/main/images/Software_schematics.png)

# Future plan
As for now, the robot's mechanical design is the weakest portion, I'm planning to use spur gear to distribute the load from the motor shaft to the motor mount and the 3D printed box.

# Last updated
01/12/2026
