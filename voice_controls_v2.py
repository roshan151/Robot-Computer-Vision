import cv2
import time
import datetime
from picamera2 import Picamera2
import multiprocessing
from ultralytics import YOLO

import sounddevice as sd
import speech_recognition as sr
from drivetrain import movement
from flask import Flask, render_template, Response

import sys
sys.path.append('/home/roshan151/nix-tts/nix-tts')
from nix.models.TTS import NixTTSInference

class voice:
    # Movement Commands dictionary
    commands = {
        'movement' : {
            'straight': {'command' : 'straight', 'complement' : 'reverse'}, 
            'forward' : {'command' : 'straight', 'complement' : 'reverse'},
            'reverse' : {'command' : 'reverse', 'complement' : 'straight'},
            'back' : {'command': 'reverse', 'complement' : 'straight'},
            'backwards' : {'command' : 'reverse', 'complement' : 'straight' } ,
            'right' : {'command' : 'right', 'complement' : 'left'},
            'write' : {'command' : 'right', 'complement' : 'left'},
            'left' : {'command' : 'left', 'complement' : 'right'},
            'stop' : {'command' : 'stop', 'complement' : None}
        },
        'vision' : ['vision', 'see', 'look'],
        'terminate' : ['shut', 'terminate'],
        'origin' : ['origin', 'return']
    }

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
        
    def speak(self, text : str):
        c, c_length, phoneme = self.nix.tokenize(text)
        # Convert text to raw speech
        xw = self.nix.vocalize(c, c_length)
        sd.play(xw[0,0], self.samplerate)
        sd.wait() 
    
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

    def initiate(self, move : movement, vision = None):

        self.move = move
        self.vision, self.vision_started = vision, False

        # This cell runs the voice recognition loop
        self.listen = True
        
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
            words = speech.lower().split()
            
            for word in words:
                if word in self.commands['movement'].keys():
                    val = self.commands['vision'][word]['command']

                    self.history.append(word)

                    # Execute movement
                    eval(f'self.move.{val}()')

                elif word in self.commands['vision']:

                    if self.vision:
                        self.vision_started = True
                        self.vision.initiate()
                        multiprocessing.Process(target=self.vision.action).start()
                    else:
                        self.speak(f'Command {word} not found. Breaking loop.')

                elif word in self.commands['terminate']:

                    self.listen = False
                    break

                elif word in self.commands['origin']:
                    
                    self.origin()
                    self.history = []

                else:
                    self.speak(f'Command {word} not found.')
                    continue

            time.sleep(1)  # Adding a brief pause before the next iteration  

        if self.vision_started:
            self.vision.terminate()

class vision:

    def __init__(self, act : str = 'record'):

        # Yolo v8 model from ultralytics 
        self.model = YOLO("yolov8m.pt")

        self.picam = Picamera2()
        self.picam.configure(self.picam.create_preview_configuration(main = {"format":'XRGB8888', "size" : (2592, 1944)}))

        # Flag to start or stop vision
        self.stop = False
        self.act= act

    def initiate(self):
        cv2.startWindowThread()
        self.picam.start()

        #self.vid = cv2.VideoCapture(0, cv2.CAP_V4L2)
        #fps = self.vid.get(cv2.CAP_PROP_FPS)

    def terminate(self):
        if self.act == 'record':
            self.picam.stop_recording()
        elif self.act in ['detect_objects', 'stream']:
            self.stop = True

    def action(self):
        if self.act == 'record':
            self.record()
        elif self.act == 'detect_objects':
            self.detect_objects()
        elif self.act == 'capture':
            self.capture()
        elif self.act == 'stream':
            self.stream()

    def record(self):
        # Configure for video recording
        self.picam.configure(self.picam.create_video_configuration())
        # Start recording video
        self.picam.start_recording("video.mp4")

    def stream(self, predict : bool= False):
        app = Flask(__name__)

        @app.route('/')
        def index():
            return render_template('index.html')

        def gen(self):     
            #get camera frame
            while self.stop == False:
                frame = self.picam.capture_array()

                if predict == True:

                    frame = self.preprocess_frame(frame)
                    results = self.model(frame)

                    # Plot results
                    frame = results[0].plot()

                    #ret, frame = cv2.imencode('.jpg', frame)
                yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame.tobytes() + b'\r\n\r\n')
                
        @app.route('/video_feed')
        def video_feed():
            return Response(gen(self.picam),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        app.run(host='0.0.0.0', debug=False)

    def capture(self):
        # Configure for still image capture
        self.picam.configure(self.picam.create_still_configuration())

        # using now() to get current time
        current_time = datetime.datetime.now()

        # Capture a picture
        self.picam.capture_file(f"image-{current_time}.jpg")

    def process_frame(self, result, detect : list):
        
        boxes = result.boxes
        all_classes = []
        # Loop over all boxes detected by the model
        for box in boxes:

            class_name = self.model.names[int(box.cls)]

            # Return class name if this is what we are looking for
            all_classes.append(class_name)
            if class_name in detect:
                return True, all_classes
                
        return False, all_classes
    
    def preprocess_frame(self):
        if frame.shape[2] == 4:  # Check if there are 4 channels
            frame = frame[:, :, :3]  # Keep only the first 3 channels

        return frame

    def detect_objects(self, objects : list = ['plant', 'plants', 'leaf', 'leaves']):

        ct = 0
        while self.stop == False:
            frame = self.picam.capture_array()
            if ct % 3 == 0: # Process every 3rd frame

                frame = self.preprocess_frame(frame)
                results = self.model(frame)
                check, class_names = self.process_frame(results[0], objects)

                if check:
                    self.speak.say('Specified object found')
                
                if ct%10 == 0 and len(class_names) > 0:
                    names = ' '.join(class_names)
                    self.speak.say(f'All objects found include: {names}')
            ct += 1

