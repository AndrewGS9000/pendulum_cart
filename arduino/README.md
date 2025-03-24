Arduino code

This folder contains all code used on the Arduino for our project.

There are 3 folders: LQR, PID, and PolePlacement, each corresponding to a different type of controller.

To run our code on an Arduino:
Upload the code of the desired controller to an Arduino Uno R3 board ("LQR/LQR.ino" for LQR for example). This is assuming the arduino is correctly wired to the electronics of the robot.

Requirements:
- Arduino IDE 2.3.3 (or another IDE that supports arduino), selecting the board "Arduino Uno"
- Arduino Uno R3
- Rest of the hardware correctly connected to the arduino (encoders, motors, motor shield, etc)
- "Motoron.h" library (by Pololu) version 1.4.0
