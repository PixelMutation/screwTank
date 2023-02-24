# TBRo-Microcontroller
Code which runs on the microcontroller to perform low level tasks:
- Closed loop motor control, with acceleration, position control and feedback (of applied power vs motor output)
- Traction control? Might be best to keep on Pi?
- Inverse kinematics of robotic arm and scoop
- Read and process IMU and Optical Flow sensor data.
- Monitor battery and current draw
- Provide safety features such as disabling motors if a keepalive signal is lost.
Also takes direct commands from the Radio Control backup system, to bypass the Pi.  
Uses a Serial interface to communicate with the Pi.  
Currently configured for the ScrewTank.  
