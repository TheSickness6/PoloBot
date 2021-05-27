# PoloBot
PoloBot is an Automated Guided Vehicle project which began during the COVID pandemic shutdown. I began this project to apply the knowledge I learned in school, specifically on differential drive robots and control systems, to the real world. My hope is to use the code and ideas gained for projects in the future.

PoloBot currently uses a Raspberry Pi as it's main communication hub and 2 Arduino Micros for controlling the two wheels. The Raspberry Pi calculates the speed needed for the individual speeds and sends it to the controllers over i2c. The controllers take this speed as a set point and use a control loop to reach the speed. The control loop in the controllers uses a PID controller tuned using Matlab and adjusted by hand.
