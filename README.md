# SCARA-Drawbot
A remix of this drawbot: https://www.thingiverse.com/thing:3096135. This project includes full Arduino sketch and some helper Python scripts along with some sample Gcode.

<img width="5712" height="4284" alt="IMG_4592" src="https://github.com/user-attachments/assets/011c02cf-fefb-423a-98ac-50e8f99c9096" />

This design is NOT a pure SCARA in the traditional sense.
It is a base-driven, 2-DOF SCARA-like arm with a mechanically coupled elbow via a 4-bar linkage.

Unlike the original this remix uses a ESP32-S3-Zero and TMC2209 (UART + STEP/DIR) Drivers

[![In Action](https://youtu.be/CUzrO_Chii8/0.jpg)](https://youtu.be/CUzrO_Chii8 "Drawbot")

Example output:
<img width="4032" height="3024" alt="IMG_4591" src="https://github.com/user-attachments/assets/cb575240-8f04-4c77-8dfc-0a71db16fa6d" />

Send Gcode to robot over serial:
Example:  python3 gcode_sender.py spirograph2.gcode /dev/cu.usbmodem201101
