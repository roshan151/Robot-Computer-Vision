import os
import json
import time
import datetime
import base64
import openai
import requests
from picamera2 import Picamera2
import multiprocessing
from prompts_and_glossary import movement_prompt, commands
import sounddevice as sd
import speech_recognition as sr
from drivetrain import movement
from flask import Flask, render_template, Response
from dotenv import load_dotenv
import sys

sys.path.append('/home/roshan151/nix-tts/nix-tts')
from nix.models.TTS import NixTTSInference

class voice:
    # Movement Commands dictionary

    history = []

    def __init__(self):
        # Speech recognition
        self.mic = sr.Microphone()
        self.rec = sr.Recognizer()

        # Voice feedback
        #self.speak = pyttsx3.init()
        #elf.speak.setProperty('rate', 120)

        # Initiate Nix-TTS
        self.samplerate = 22050
        self.nix = NixTTSInference(model_dir = "/home/roshan151/nix-tts/nix-deterministic/nix-deterministic")

        load_dotenv()
        self.openai_api_key = os.getenv("OPENAI_API_KEY_ROBIN")
        
    def speak(self, text : str):
        c, c_length, phoneme = self.nix.tokenize(text)
        # Convert text to raw speech
        xw = self.nix.vocalize(c, c_length)
        sd.play(xw[0,0], self.samplerate)
        sd.wait() 

    def query_gpt(self, messages, model="gpt-4o-mini", temperature=0.9, max_tokens=1500):
        """
        param prompt: The input text prompt.
        param api_key: Your OpenAI API key.
        param model: The OpenAI model to use (default: gpt-3.5-turbo).
        param temperature: Sampling temperature (higher values make output more random).
        param max_tokens: Maximum number of tokens to generate.
        """
        openai.api_key = self.openai_api_key
        client = openai.OpenAI(api_key=self.openai_api_key)
        
        try:
            response = client.chat.completions.create(
                model=model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens
            )
            
            return response.choices[0].message.content.strip()
        
        except Exception as e:
            return f"Error: {str(e)}"
    
    def origin(self):
        '''
        Loop through the history stack and execute complementary movements
        '''
        while len(self.history) > 0:
            command = self.history.pop(0)
            complement = self.commands['movement'][command]['complement']
            if complement is None:
                continue
            
            eval(f'self.move.{complement}()')

    def scan(self, objects, interval = 15, span = 180):
        '''
        Scan the area by turning small angles (interval) and clicking a picture
        These pictures will be sent in a batch for object detection
        '''

        left, right = 0, 0
        pictures = []
        angles = []

        # Click pictures on left
        while left<span//2:
            self.move.left(interval)
            image_name = self.vision.action('capture')
            pictures.append(image_name)
            angles.append(-1*left)
            left += interval

        # Back to center
        self.move.right(left)

        # Click pictures on right
        while right<span//2:
            self.move.right(interval)
            image_name = self.vision.action('capture')
            pictures.append(image_name)
            angles.append(-1*right)
            right += interval

        # Back to center
        self.move.left(right)
        angle = 0
        for idx, i in enumerate(pictures, objects):
            detect = self.vision.detect_image(i)
            if detect == True:
                angle = angles[idx]
                break

        if angle<0:
            self.move.left(abs(angle))
        elif angle >0:
            self.move.right(angle)

        return pictures, angles
        

    def initiate(self, move : movement, vision = None):

        self.move = move
        self.vision, self.vision_started = vision, False

        # This cell runs the voice recognition loop
        self.listen = True
        messages = [{'role' : 'system', 'content' : movement_prompt}]
        while self.listen:
            with self.mic as source:
                self.rec.adjust_for_ambient_noise(source)

                self.speak("Speak out commands.")
                try:
                    audio = self.rec.listen(source, timeout=8, phrase_time_limit=8)
                    speech = str(self.rec.recognize_google(audio))  # Using Google's API for recognition
                except:
                    self.speak('No audio detected, try again in two seconds.')
                    continue

            #print(f"Recognized speech: {speech}")  # Debug output
            query = speech.lower()
            messages.append({'role': 'user', 'content': query})

            query_commands = self.query_gpt3(messages)

            messages.append([{'role' : 'assistant', 'content' : query_commands}])

            for item in query_commands:
                word = item.keys()[0]
                value = item[word]

                if word in self.commands['movement'].keys():
                    self.history.append({word : value})

                    # Execute movement
                    eval(f'self.move.{word}({value})')

                elif word in self.commands['vision']:

                    if self.vision:
                        self.vision_started = True
                        self.vision.initiate()

                        response = self.vision.action(value)

                        if value == 'detect':
                            if response:
                                self.speak(f'Required objects found')
                            else:
                                self.speak(f'Required objects not found')
                            
                    else:
                        self.speak(f'Command {word} not found. Breaking loop.')

                elif word in self.commands['terminate']:

                    self.listen = False
                    break

                elif word in self.commands['origin']:
                    
                    self.origin(value)
                    self.history = []

                else:
                    self.speak(f'Command {word} not found.')
                    continue

            time.sleep(1)  # Adding a brief pause before the next iteration  

        if self.vision_started:
            self.vision.terminate()

class vision:

    def __init__(self, act : str = 'detect'):

        self.picam = Picamera2()

        # Flag to start or stop vision
        self.stop = False
        self.act= act

    def terminate(self):
        if self.act == 'record':
            self.picam.stop_recording()

        self.picam.stop()

    def action(self):
        if self.act == 'record':
            self.record()
        elif self.act == 'detect':
            self.detect()
        elif self.act == 'capture':
            self.capture()

    def record(self):
        # Configure for video recording
        self.picam.configure(self.picam.create_video_configuration())
        self.picam.start()

        # using now() to get current time
        current_time = datetime.datetime.now()

        file_name = f"video-{current_time}.mp4"
        
        # Start recording video
        self.picam.start_recording(file_name)
        return file_name

    def capture(self):

        # Configure for still image capture
        self.picam.configure(self.picam.create_still_configuration())

        # using now() to get current time
        current_time = datetime.datetime.now()

        image_name = f"image-{current_time}.jpg"
        # Capture a picture
        self.picam.capture_file(image_name)

        return image_name

    def detect(self, objects):
        
        image_name = self.capture()
        with open(image_name, "rb") as f:
            im_bytes = f.read()        
        im_b64 = base64.b64encode(im_bytes).decode("utf8")
        headers = {'content-type' : 'application/json'}

        ip = f'**:**:**:**' # Remote systems ip address
        response = requests.post(f'http://{ip}:8080/detect_plants:frame', headers = headers, json =  { "image": im_b64, "objects" : objects } )
        result = json.loads(response.content)
        return result['response']
    
    def detect_image(self, image_name, objects = None):
        
        with open(image_name, "rb") as f:
            im_bytes = f.read()        
        im_b64 = base64.b64encode(im_bytes).decode("utf8")
        headers = {'content-type' : 'application/json'}
        ip = f'**:**:**:**' # Remote systems ip address
        response = requests.post(f'http://{ip}:8080/detect_plants:frame', headers = headers, json = { "image": im_b64, "objects" : objects } )
        result = json.loads(response.content)

        return result['response']
        
        

