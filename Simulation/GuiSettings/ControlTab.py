from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QGroupBox, QSizePolicy, QStyle
)
from PyQt5.QtCore import Qt
from Simulation.SimulationSettings.models import Hexapod

class ControlTab(QWidget):
    def __init__(self, robot: Hexapod, update_plot_callback):
        super().__init__()
        self.robot = robot
        self.update_plot_callback = update_plot_callback
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.camera_label = QLabel("Camera Output")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setStyleSheet("border: 1px solid black;")
        self.camera_label.setFixedSize(640, 480)  
        layout.addWidget(self.camera_label, alignment=Qt.AlignCenter)

        control_group = QGroupBox("Hexapod Controls")
        control_layout = QHBoxLayout()
        control_group.setLayout(control_layout)
        control_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        control_group.setFixedHeight(100) 

        self.forward_button = QPushButton("Forward")
        self.backward_button = QPushButton("Backward")
        self.left_button = QPushButton("Left")
        self.right_button = QPushButton("Right")
        self.stop_button = QPushButton("Stop")

        self.forward_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowUp))
        self.backward_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowDown))
        self.left_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowLeft))
        self.right_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowRight))
        self.stop_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))

        buttons = [
            self.forward_button,
            self.backward_button,
            self.left_button,
            self.right_button,
            self.stop_button
        ]
        for btn in buttons:
            btn.setFixedSize(100, 40) 
            btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        control_layout.addStretch()
        control_layout.addWidget(self.forward_button)
        control_layout.addWidget(self.backward_button)
        control_layout.addWidget(self.left_button)
        control_layout.addWidget(self.right_button)
        control_layout.addWidget(self.stop_button)
        control_layout.addStretch()

        layout.addWidget(control_group)

        camera_control_layout = QHBoxLayout()
        self.start_camera_button = QPushButton("Start Camera")
        self.start_camera_yolo_button = QPushButton("Start Camera with YOLO")
        self.stop_camera_button = QPushButton("Stop Camera")
        self.stop_camera_button.setEnabled(False)

        self.start_camera_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.start_camera_yolo_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.stop_camera_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))

        self.start_camera_button.setFixedSize(160, 40)
        self.start_camera_yolo_button.setFixedSize(200, 40)
        self.stop_camera_button.setFixedSize(120, 40)

        camera_control_layout.addStretch()
        camera_control_layout.addWidget(self.start_camera_button)
        camera_control_layout.addWidget(self.start_camera_yolo_button)
        camera_control_layout.addWidget(self.stop_camera_button)
        camera_control_layout.addStretch()

        layout.addLayout(camera_control_layout)

        media_control_layout = QHBoxLayout()
        self.capture_photo_button = QPushButton("Capture Photo")
        self.record_video_button = QPushButton("Start Recording")

        self.capture_photo_button.setIcon(self.style().standardIcon(QStyle.SP_DialogOpenButton))
        self.record_video_button.setIcon(self.style().standardIcon(QStyle.SP_DialogYesButton))

        self.capture_photo_button.setFixedSize(150, 40)
        self.record_video_button.setFixedSize(150, 40)

        media_control_layout.addStretch()
        media_control_layout.addWidget(self.capture_photo_button)
        media_control_layout.addWidget(self.record_video_button)
        media_control_layout.addStretch()

        layout.addLayout(media_control_layout)

