# Hardware
Building a robot with Computer Vision capabilities. Robot is controlled using a raspberry Pi 4B processor. 
The peripherals include a tf-Luna distance detector, and a 32 mp Hi resoultion camera to capture images or record videos.
The drivetrain is built using 2 DC motors operating on 12-15v and controlled using a L298 motor driver. 
TF Luna creates a distance graph and overrides all movement commands in order to stop movement before approaching any obstacle. 
I am powering the motor using 2 batteries of 9v connected in series (outputs 18v but L298 leads to some losses resulting in 14v output)
The L298 driver itself is powered using a seperate 9v battery which is attached to a 330 ohm resistor to reduce voltage to 4v.  

# Computer Vision
Initial experiments of running a yolo (cnn) model on raspberry pi had crucial drawbacks including high latency and occasional OOM errors.
To overcome this I am hosting a Docker image on my PC that which hosts a computer vision model. Raspberry pi will do a network call to this image in order to get predictions. 
This will allow me to experiment with computer vision architecture and I can host a vision transformer or an ensemble of finetuned CNN models to perform computer vision tasks. For now, the task is very simple - detect plants in the jpeg images sent by Raspberry-Pi.

# Voice Control Features
To interface with the Robot while its running I am currently using voice control using bluetooth buds connected to RAspberry Pi.
Bluetooth connection is provided using pulseaudio module (bluetooth buds card is set to hfp for both input and output).
I am using speech_recognition library with google cloud speech recognition service for speech to text conversion.
Currently, the robot supports single word commands - forward, reverse, right, left, stop. (TODO: Use LLM to process complete sentences)
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
