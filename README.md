# Building the robot
Building a robot with Computer Vision capabilities. Robot is controlled using a raspberry Pi 4B processor. 
The peripherals include a tf-Luna distance detector, and a 32 mp Hi resoultion camera to capture images or record videos.
The drivetrain is built using 2 DC motors operating on 12-15v and controlled using a L298 motor driver. 
TF Luna creates a distance graph and overrides all movement commands in order to stop movement before approaching any obstacle. 
I am powering the motor using 2 batteries of 9v connected in series (outputs 18v but L298 leads to some losses resulting in 14v)
The L298 driver itself is powered using a seperate 9v battery which is attached to a 330 ohm resistor to reduce voltage to a 4v.  

# Computer Vision
The goal is to build a robot that can detect plants in its surroundings and perform assigned actions.
To add Computer Vision capabalities, I am currently currently running an ultralytics YOLO model on the Pi itself. 
I have also finetuned the YOLO model on agricultral dataset for it to be able to distinguish between plants and weeds.
Drawbacks: Running Computer Vision CNN models on pi is very slow.
Fix: Host a containerized model using flask api and use ngrok for hosting. Pi can ping this service to get image predictions.

# Voice Control Features
To interface with the robot while its running I am currently using voice control from my bluetooth buds.
Bluetooth connection is provided using pulseaudio module (bluetooth buds card is set to hfp for both input and output).
I am using speech_recognition library with google cloud speech recognition service for speech to text conversion.
Currently the robot supports single word commands - forward, reverse, right, left, stop.
pyttsx3 library provides text to speech output to prompt user for input and relay any error messages.

# Robot images:
Top View:
![PXL_20241207_170700225](https://github.com/user-attachments/assets/65aa1004-ad71-4d3f-be6d-fdf788f3cd46)

Side View:
![PXL_20241207_170650358](https://github.com/user-attachments/assets/bf1c318c-c084-4d42-8a80-22c9ce910a82)

Front View:
![PXL_20241207_170641184](https://github.com/user-attachments/assets/504cd7a0-b8d2-4fd7-ae23-dbf517e464cc)

# Resources:
Speech Recognition: https://atsss.medium.com/real-time-speech-to-text-on-raspberry-pi-and-python-4be8c347a8fc

Text to Speech (For Feedback): https://pypi.org/project/pyttsx3/
