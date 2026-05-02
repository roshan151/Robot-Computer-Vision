FROM python:3.11.9-bullseye
WORKDIR /app

RUN apt-get update && apt-get install -y pkg-config ffmpeg libsm6 libxext6  -y 
#libhdf5-dev

COPY requirements.txt /app/
RUN pip install --upgrade pip
RUN pip install -r requirements.txt

COPY yolov8m.pt /app/
COPY service.py /app/

CMD ["uvicorn", "service:router", "--host", "0.0.0.0", "--port", "8080"]