import os
import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

class YOLOModel:
    def __init__(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(base_dir, '..', 'yolov3.weights')
        cfg_path = os.path.join(base_dir, '..', 'yolov3.cfg')
        names_path = os.path.join(base_dir, '..', 'coco.names')
        
        weights_path = os.path.abspath(weights_path)
        cfg_path = os.path.abspath(cfg_path)
        names_path = os.path.abspath(names_path)
        
        print(f"Weights path: {weights_path}")
        print(f"Config path: {cfg_path}")
        print(f"Names path: {names_path}")

        if not os.path.exists(weights_path):
            raise FileNotFoundError(f"Weights file not found at {weights_path}")
        if not os.path.exists(cfg_path):
            raise FileNotFoundError(f"Config file not found at {cfg_path}")
        if not os.path.exists(names_path):
            raise FileNotFoundError(f"Names file not found at {names_path}")

        self.net = cv2.dnn.readNet(weights_path, cfg_path)
        self.classes = []
        with open(names_path, 'r') as f:
            self.classes = f.read().splitlines()
        self.output_layers = self.net.getUnconnectedOutLayersNames()

    def detect(self, frame):
        blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)
        detections = self.process_outputs(outputs, frame.shape[:2])
        return detections

    def process_outputs(self, outputs, dimensions):
        detections = []
        height, width = dimensions
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    detections.append({
                        'bbox': [x, y, w, h],
                        'label': self.classes[class_id],
                        'confidence': float(confidence)
                    })
        return detections

class YOLODetectionThread(QThread):
    detections_ready = pyqtSignal(list)

    def __init__(self, yolo_model):
        super().__init__()
        self.yolo_model = yolo_model
        self.frame = None
        self.running = True

    def run(self):
        print("YOLODetectionThread started.")
        while self.running:
            if self.frame is not None:
                detections = self.yolo_model.detect(self.frame)
                self.detections_ready.emit(detections)
                self.frame = None
        print("YOLODetectionThread stopped.")

    def stop(self):
        self.running = False
        self.wait()

    def update_frame(self, frame):
        self.frame = frame.copy()