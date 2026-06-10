# SCARA-Drawbot
A remix of this drawbot: https://www.thingiverse.com/thing:3096135. This project includes full Arduino sketch and some helper Python scripts along with some sample Gcode.

<img width="5712" height="4284" alt="IMG_4592" src="https://github.com/user-attachments/assets/011c02cf-fefb-423a-98ac-50e8f99c9096" />

This design is NOT a pure SCARA in the traditional sense.
It is a base-driven, 2-DOF SCARA-like arm with a mechanically coupled elbow via a 4-bar linkage.

Unlike the original this remix uses a ESP32-S3-Zero and TMC2209 (UART + STEP/DIR) Drivers

[![In Action](https://youtu.be/CUzrO_Chii8/0.jpg)](https://youtu.be/CUzrO_Chii8 "Drawbot")

# Example output:
<img width="4032" height="3024" alt="IMG_4591" src="https://github.com/user-attachments/assets/cb575240-8f04-4c77-8dfc-0a71db16fa6d" />

<img width="2935" height="2539" alt="IMG_4594" src="https://github.com/user-attachments/assets/4ae784b8-08cb-4b90-b9d1-bbfbf10a10d4" />

# Send Gcode to robot over serial:
Example:  
     ```
     python3 gcode_sender.py spirograph2.gcode /dev/cu.usbmodem201101
	 ```

# Lineart to Gcode via Inkscape
Create .svg file
1. Open Inkscape
2. New Document
3. Set document size to 160 x 160mm
4. Import Line Art (png, bmp etc)
5. Trace bitmap
	  Centerline tracing (autotrace) / apply
	  Drag converted linear off of bitmap and delete bitmap leaving just line art
	  Save as .svg

Convert SVG to Gcode
Open:  https://sameer.github.io/svg2gcode/#close
	Configure Settings for pen actions and homing
  <img width="634" height="651" alt="Dots per Inch" src="https://github.com/user-attachments/assets/c516572f-0517-4664-a622-a0622392f2df" />

Generate Gcode
Choose your .svg created in Inkscape and select Generate G-Code
<img width="1182" height="890" alt="svg2gcode" src="https://github.com/user-attachments/assets/fc0c6758-2027-46f8-b212-5b543cae5434" />

Cleanup Gcode for robot:  
	```
	python3 inkscape_to_robot.py elephant.gcode --offset-x -57.6 --offset-y -46.6 
    ```

Send to robot:  
    ```
	python3 gcode_sender.py elephant_robot.gcode /dev/cu.usbmodem201101
	```
