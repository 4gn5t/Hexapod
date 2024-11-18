from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QTabWidget, QCheckBox, QGridLayout, QGroupBox,
    QSizePolicy, QStyle, QStyleFactory
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QIcon, QFont, QPixmap, QImage
from Simulation.SimulationSettings.models import Hexapod
from Simulation.SimulationSettings.constant import *

class SimulationTab(QWidget):
    def __init__(self, robot: Hexapod, update_plot_callback):
        super().__init__()
        self.robot = robot
        self.update_plot_callback = update_plot_callback
        self.init_ui()

    def init_ui(self):
        layout = QHBoxLayout()
        self.setLayout(layout)

        controls_layout = QVBoxLayout()
        controls_layout.setAlignment(Qt.AlignTop)

        self.dim_group = QGroupBox("Dimension Settings")
        self.dim_layout = QVBoxLayout()
        self.dim_group.setLayout(self.dim_layout)

        dim_labels = ['Front', 'Middle', 'Side', 'coxa', 'Femur', 'Tibia']
        default_values = DEFAULT_DIMSIONS + DEFAULT_LEG_LENGTH
        self.dim_sliders = {}

        for label, value in zip(dim_labels, default_values):
            lbl = QLabel(label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(1)
            slider.setMaximum(20)
            slider.setValue(value)
            slider.setTickInterval(1)
            slider.setTickPosition(QSlider.TicksBelow)
            lbl.setFont(QFont("Arial", 10))
            self.dim_layout.addWidget(lbl)
            self.dim_layout.addWidget(slider)
            self.dim_sliders[label] = slider

        button_layout = QHBoxLayout()
        self.reset_dim_button = QPushButton("Reset Dimensions")
        self.reset_view_button = QPushButton("Reset 3D View")
        self.reset_dim_button.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        self.reset_view_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowBack))
        button_layout.addWidget(self.reset_dim_button)
        button_layout.addWidget(self.reset_view_button)
        self.dim_layout.addLayout(button_layout)

        controls_layout.addWidget(self.dim_group)

        self.simulation_tabs = QTabWidget()
        controls_layout.addWidget(self.simulation_tabs)

        self.init_leg_pattern_tab()
        self.init_ik_tab()
        self.init_gait_tab()

        layout.addLayout(controls_layout)

    def init_leg_pattern_tab(self):
        self.leg_tab = QWidget()
        self.leg_tab_layout = QVBoxLayout()
        self.leg_tab.setLayout(self.leg_tab_layout)

        self.leg_tab_layout.addWidget(QLabel("<i>Legs share the same pose</i>"))

        leg_labels = [r'α (coxa-zaxis)', r'β (femur-xaxis)', r'γ (tibia-xaxis)']
        self.leg_sliders = {}

        for label in leg_labels:
            lbl = QLabel(label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.setTickInterval(10)
            slider.setTickPosition(QSlider.TicksBelow)
            lbl.setFont(QFont("Arial", 10))
            self.leg_tab_layout.addWidget(lbl)
            self.leg_tab_layout.addWidget(slider)
            self.leg_sliders[label.split()[0]] = slider 

        self.simulation_tabs.addTab(self.leg_tab, "Leg Pattern")

    def init_ik_tab(self):
        self.ik_tab = QWidget()
        self.ik_tab_layout = QVBoxLayout()
        self.ik_tab.setLayout(self.ik_tab_layout)

        self.ik_sliders = {}
        axes = ['X', 'Y', 'Z']
        translations = ['TX', 'TY', 'TZ']
        rotations = ['RX', 'RY', 'RZ']

        # Translation sliders
        trans_group = QGroupBox("Translations")
        trans_layout = QGridLayout()
        trans_group.setLayout(trans_layout)
        for i, (axis, label) in enumerate(zip(axes, translations)):
            lbl = QLabel(label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-100)  
            slider.setMaximum(100)
            slider.setValue(0)
            slider.setTickInterval(10)
            slider.setTickPosition(QSlider.TicksBelow)
            lbl.setFont(QFont("Arial", 10))
            trans_layout.addWidget(lbl, i, 0)
            trans_layout.addWidget(slider, i, 1)
            self.ik_sliders[label] = slider

        self.ik_tab_layout.addWidget(trans_group)

        rot_group = QGroupBox("Rotations")
        rot_layout = QGridLayout()
        rot_group.setLayout(rot_layout)
        for i, (axis, label) in enumerate(zip(axes, rotations)):
            lbl = QLabel(label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-30)
            slider.setMaximum(30)
            slider.setValue(0)
            slider.setTickInterval(1)
            slider.setTickPosition(QSlider.TicksBelow)
            lbl.setFont(QFont("Arial", 10))
            rot_layout.addWidget(lbl, i, 0)
            rot_layout.addWidget(slider, i, 1)
            self.ik_sliders[label] = slider

        self.ik_tab_layout.addWidget(rot_group)

        self.simulation_tabs.addTab(self.ik_tab, "Inverse Kinematics")

    def init_gait_tab(self):
        self.gait_tab = QWidget()
        self.gait_tab_layout = QVBoxLayout()
        self.gait_tab.setLayout(self.gait_tab_layout)

        self.gait_sliders = {}
        gait_params = [
            ('LiftSwing', 10, 40, 20),
            ('HipSwing', 10, 40, 12),
            ('GaitStep', 5, 20, 10),
            ('GaitSpeed', 5, 20, 10)
        ]

        gait_group = QGroupBox("Gait Parameters")
        gait_layout = QGridLayout()
        gait_group.setLayout(gait_layout)

        for i, (label, min_val, max_val, default_val) in enumerate(gait_params):
            lbl = QLabel(label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setValue(default_val)
            slider.setTickInterval(1)
            slider.setTickPosition(QSlider.TicksBelow)
            lbl.setFont(QFont("Arial", 10))
            gait_layout.addWidget(lbl, i, 0)
            gait_layout.addWidget(slider, i, 1)
            self.gait_sliders[label] = slider

        self.gait_tab_layout.addWidget(gait_group)

        self.is_tripod_cb = QCheckBox("Tripod Gait")
        self.is_forward_cb = QCheckBox("Forward Direction")
        self.is_rotate_cb = QCheckBox("Rotate Body")
        self.is_tripod_cb.setChecked(True)
        self.is_forward_cb.setChecked(True)

        self.gait_tab_layout.addWidget(self.is_tripod_cb)
        self.gait_tab_layout.addWidget(self.is_forward_cb)
        self.gait_tab_layout.addWidget(self.is_rotate_cb)

        button_layout = QHBoxLayout()
        self.gait_play_button = QPushButton("Play")
        self.gait_pause_button = QPushButton("Pause")
        self.gait_step_button = QPushButton("Step")

        self.gait_play_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.gait_pause_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        self.gait_step_button.setIcon(self.style().standardIcon(QStyle.SP_MediaSkipForward))

        button_layout.addWidget(self.gait_play_button)
        button_layout.addWidget(self.gait_pause_button)
        button_layout.addWidget(self.gait_step_button)

        self.gait_tab_layout.addLayout(button_layout)

        self.simulation_tabs.addTab(self.gait_tab, "Walking Gaits")