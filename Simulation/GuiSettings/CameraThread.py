from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage
import cv2

class CameraThread(QThread):
    frame_updated = pyqtSignal(QImage)

    def __init__(self, camera_index=0, width=640, height=480):
        super().__init__()
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.running = False

    def run(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.running = True
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.frame_updated.emit(qt_image)
            else:
                break
        self.cap.release()

    def stop(self):
        self.running = False
        self.wait()