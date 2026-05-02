import io

import base64 
from PIL import Image
from ultralytics import YOLO

import numpy as np
from typing import Dict
from fastapi import FastAPI, APIRouter, Response

router = APIRouter()

class computer_vision:

    def __init__(self, model_name :str = "yolov8m.pt"):
        # Instantiate a CNN model 
        self.model = YOLO(model_name)


    #Loop through the identified box coordinates and check any of them is a plant/leaf 
    def detect_plant(self, result, objects : list):

        for r in result:
            boxes = r.boxes
            for box in boxes:
                class_name = self.model.names[int(box.cls)]
                if class_name in objects:
                    return True
        return False
    
    def preprocess_frame(self, frame):

        if frame.shape[2] == 4:  # Check if there are 4 channels
            frame = frame[:, :, :3]  # Keep only the first 3 channels

        return frame

    # Process the frame through model
    def predict_frame(self, frame):

        frame = self.preprocess_frame(frame)
        result = self.model(frame)
        plants = self.detect_plant(result)

        return plants

cv = computer_vision()

@router.post('/detect_objects:frame')
def predict(frame_json : Dict, cv : computer_vision = cv):

    im_b64 = frame_json['image']
    objects = frame_json['objects']

    if objects == None:
        objects = ['plant', 'plants', 'leaf', 'leaves']

    # convert it into bytes  
    img_bytes = base64.b64decode(im_b64.encode('utf-8'))
    # convert bytes data to PIL Image object
    img = Image.open(io.BytesIO(img_bytes))
    
    # PIL image object to numpy array
    img_arr = np.asarray(img)      
    response = cv.predict_frame(img_arr)

    frame_json['response'] = response

    return frame_json

    






