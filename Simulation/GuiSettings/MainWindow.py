from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QStyle, QStyleFactory
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap, QImage
from Simulation.SimulationSettings.models import Hexapod
import matplotlib.pyplot as plt
from Simulation.SimulationSettings.constant import *
from Simulation.GuiSettings.SimulationTab import SimulationTab
from Simulation.GuiSettings.ControlTab import ControlTab
from Simulation.GuiSettings.CameraThread import CameraThread
try:
    import qdarkstyle
except ImportError:
    qdarkstyle = None 
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import cv2
import configparser
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PyQt5.QtCore import QDateTime
import numpy as np
from Simulation.GuiSettings.YOLOmodel import YOLOModel, YOLODetectionThread

default_elev = 20
default_azim = 30

conf = configparser.ConfigParser()
conf.read('style.ini', encoding='utf-8')

body_color = conf.get("robot plotter", 'body_color', fallback='blue')
leg_color = conf.get("robot plotter", 'leg_color', fallback='red')
joint_size = int(conf.get("robot plotter", 'joint_size', fallback='5'))
head_color = conf.get("robot plotter", 'head_color', fallback='green')
head_size = int(conf.get("robot plotter", 'head_size', fallback='10'))
axis_colors = {
    'x': conf.get("axis", 'color_x', fallback='red'),
    'y': conf.get("axis", 'color_y', fallback='green'),
    'z': conf.get("axis", 'color_z', fallback='blue')
}
origin_size = int(conf.get("axis", 'origin_size', fallback='10'))
ground_color = conf.get('ground', 'color', fallback='gray')
ground_opacity = float(conf.get('ground', 'opacity', fallback='0.2'))


def draw_robot(robot: Hexapod, ax):
    ax.clear()

    body_verts = [(p.x, p.y, p.z) for p in robot.body.vertices]
    body_poly = Poly3DCollection([body_verts], color=body_color, alpha=0.7)
    ax.add_collection3d(body_poly)

    outline_x = [p.x for p in robot.body.vertices] + [robot.body.vertices[0].x]
    outline_y = [p.y for p in robot.body.vertices] + [robot.body.vertices[0].y]
    outline_z = [p.z for p in robot.body.vertices] + [robot.body.vertices[0].z]
    ax.plot3D(outline_x, outline_y, outline_z, color=leg_color, linewidth=2)

    ax.scatter3D([robot.body.head.x], [robot.body.head.y], [robot.body.head.z],
                color=head_color, s=head_size)

    for leg in robot.legs.values():
        leg_x = [p.x for p in leg.points_global]
        leg_y = [p.y for p in leg.points_global]
        leg_z = [p.z for p in leg.points_global]
        ax.plot3D(leg_x, leg_y, leg_z, color=leg_color, linewidth=2)
        ax.scatter3D(leg_x, leg_y, leg_z, color=leg_color, s=joint_size)

    support_x = [p.x for p in robot.ground_contact_points.values()]
    support_y = [p.y for p in robot.ground_contact_points.values()]
    support_z = [p.z - 0.01 for p in robot.ground_contact_points.values()]

    s = float(conf.get('ground', 'size', fallback='20'))
    ground_x = [s / 2, -s / 2, -s / 2, s / 2, s / 2]
    ground_y = [s / 2, s / 2, -s / 2, -s / 2, s / 2]
    ground_z = [0, 0, 0, 0, 0]
    ax.plot3D(ground_x, ground_y, ground_z, color=ground_color, alpha=ground_opacity)

    ax.set_xlim([-8, 8])
    ax.set_ylim([-8, 8])
    ax.set_zlim([-8, 8])
    ax.axis('off')

    ax.view_init(elev=default_elev, azim=default_azim)

def play_robot_walking(robot: Hexapod, t):
    robot.set_pose_from_walking_sequence(t)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hexapod Simulator")
        self.resize(1200, 800)

        self.robot = Hexapod()
        self.gait_step = 0

        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')

        self.init_ui()
        self.setup_connections()
        self.update_plot()

        self.is_recording = False
        self.video_writer = None

        self.is_yolo_enabled = False

    def init_ui(self):
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout()
        self.main_widget.setLayout(self.main_layout)

        self.main_tabs = QTabWidget()
        self.main_layout.addWidget(self.main_tabs)

        self.simulation_tab = SimulationTab(self.robot, self.update_plot)
        simulation_layout = QHBoxLayout()
        simulation_layout.addWidget(self.simulation_tab)
        simulation_layout.addWidget(self.canvas, stretch=1)
        simulation_container = QWidget()
        simulation_container.setLayout(simulation_layout)
        self.main_tabs.addTab(simulation_container, "Simulation")

        self.control_tab = ControlTab(self.robot, self.update_plot)
        self.main_tabs.addTab(self.control_tab, "Control")

        self.apply_styles()

    def apply_styles(self):
        font = QFont("Arial", 10)
        self.setFont(font)

        if qdarkstyle:
            self.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
        else:
            QApplication.setStyle(QStyleFactory.create('Fusion'))
            palette = QApplication.palette()
            palette.setColor(palette.Window, Qt.gray)
            palette.setColor(palette.WindowText, Qt.white)
            QApplication.setPalette(palette)

    def setup_connections(self):
        for slider in self.simulation_tab.dim_sliders.values():
            slider.valueChanged.connect(self.on_dimension_changed)

        self.simulation_tab.reset_dim_button.clicked.connect(self.reset_dimensions)
        self.simulation_tab.reset_view_button.clicked.connect(self.reset_view)

        for slider in self.simulation_tab.leg_sliders.values():
            slider.valueChanged.connect(self.on_leg_pattern_changed)

        for slider in self.simulation_tab.ik_sliders.values():
            slider.valueChanged.connect(self.on_ik_changed)

        for slider in self.simulation_tab.gait_sliders.values():
            slider.valueChanged.connect(self.on_gait_parameters_changed)
        self.simulation_tab.is_tripod_cb.stateChanged.connect(self.on_gait_parameters_changed)
        self.simulation_tab.is_forward_cb.stateChanged.connect(self.on_gait_parameters_changed)
        self.simulation_tab.is_rotate_cb.stateChanged.connect(self.on_gait_parameters_changed)

        self.simulation_tab.gait_play_button.clicked.connect(self.on_gait_play)
        self.simulation_tab.gait_pause_button.clicked.connect(self.on_gait_pause)
        self.simulation_tab.gait_step_button.clicked.connect(self.on_gait_step)

        self.gait_timer = QTimer()
        self.gait_timer.timeout.connect(self.update_gait)

        self.control_tab.start_camera_button.clicked.connect(self.start_camera)
#        self.control_tab.start_camera_yolo_button.clicked.connect(self.start_camera_with_yolo)
        self.control_tab.stop_camera_button.clicked.connect(self.stop_camera)

        self.control_tab.forward_button.clicked.connect(self.move_forward)
        self.control_tab.backward_button.clicked.connect(self.move_backward)
        self.control_tab.left_button.clicked.connect(self.turn_left)
        self.control_tab.right_button.clicked.connect(self.turn_right)
        self.control_tab.stop_button.clicked.connect(self.stop_movement)

        self.control_tab.capture_photo_button.clicked.connect(self.capture_photo)
        self.control_tab.record_video_button.clicked.connect(self.toggle_video_recording)


    def on_dimension_changed(self):
        values = [self.simulation_tab.dim_sliders[label].value() for label in ['Front', 'Middle', 'Side', 'coxa', 'Femur', 'Tibia']]
        if self.robot.update_dimensions(values):
            self.update_plot()

    def on_leg_pattern_changed(self):
        values = [self.simulation_tab.leg_sliders[label].value() for label in ['α', 'β', 'γ']]
        if self.robot.update_leg_pattern(values):
            self.update_plot()

    def reset_dimensions(self):
        default_values = DEFAULT_DIMSIONS + DEFAULT_LEG_LENGTH
        for label, value in zip(['Front', 'Middle', 'Side', 'coxa', 'Femur', 'Tibia'], default_values):
            self.simulation_tab.dim_sliders[label].setValue(value)

    def reset_view(self):
        self.ax.view_init(elev=default_elev, azim=default_azim)
        self.update_plot()

    def on_ik_changed(self):
        rx = self.simulation_tab.ik_sliders['RX'].value()
        ry = self.simulation_tab.ik_sliders['RY'].value()
        rz = self.simulation_tab.ik_sliders['RZ'].value()
        tx = self.simulation_tab.ik_sliders['TX'].value() * 0.01 * self.robot.body.f
        ty = self.simulation_tab.ik_sliders['TY'].value() * 0.01 * self.robot.body.s
        tz = self.simulation_tab.ik_sliders['TZ'].value() * 0.01 * self.robot.legs[0].lengths[-1]
        self.robot.solve_ik([rx, ry, rz], [tx, ty, tz])
        self.update_plot()

    def on_gait_parameters_changed(self):
        ls = self.simulation_tab.gait_sliders['LiftSwing'].value()
        hs = self.simulation_tab.gait_sliders['HipSwing'].value()
        st = self.simulation_tab.gait_sliders['GaitStep'].value()
        sp = self.simulation_tab.gait_sliders['GaitSpeed'].value()
        is_tripod = self.simulation_tab.is_tripod_cb.isChecked()
        is_forward = self.simulation_tab.is_forward_cb.isChecked()
        is_rotate = self.simulation_tab.is_rotate_cb.isChecked()

        para = {}
        para['HipSwing'] = hs
        para['LiftSwing'] = ls
        para['StepNum'] = st
        para['Speed'] = sp
        para['Gait'] = 'Tripod' if is_tripod else 'Ripple'
        para['Direction'] = 1 if is_forward else -1
        para['Rotation'] = 1 if is_rotate else 0
        self.robot.generate_walking_sequence(para)
        self.gait_timer.stop()
        self.gait_step = 0

    def on_gait_play(self):
        sp = self.simulation_tab.gait_sliders['GaitSpeed'].value()
        if sp == 0:
            return  #
        interval = max(10, int(1000 / sp))  # Minimum interval of 10ms
        self.gait_timer.start(interval)

    def on_gait_pause(self):
        self.gait_timer.stop()

    def on_gait_step(self):
        self.gait_step += 1
        self.update_gait()

    def update_gait(self):
        if not self.robot.walking_sequence:
            return
        num_steps = len(next(iter(self.robot.walking_sequence[0].values())))
        t = self.gait_step % num_steps
        play_robot_walking(self.robot, t)
        self.update_plot()
        self.gait_step += 1

    def start_camera(self):
        self.is_yolo_enabled = False
        self._start_camera_common()

    def _start_camera_common(self):
        self.camera_thread = CameraThread(width=640, height=480)
        self.camera_thread.frame_updated.connect(self.update_camera_feed)
        self.camera_thread.start()
        self.control_tab.start_camera_button.setEnabled(False)
        self.control_tab.start_camera_yolo_button.setEnabled(False)
        self.control_tab.stop_camera_button.setEnabled(True)

    def stop_camera(self):
        if hasattr(self, 'camera_thread') and self.camera_thread.isRunning():
            self.camera_thread.stop()
            self.camera_thread = None
        self.control_tab.camera_label.clear()
        self.control_tab.start_camera_button.setEnabled(True)
        self.control_tab.start_camera_yolo_button.setEnabled(True)
        self.control_tab.stop_camera_button.setEnabled(False)

    def update_camera_feed(self, image):
        image = image.convertToFormat(QImage.Format_RGB888)
        width = image.width()
        height = image.height()
        ptr = image.bits()
        ptr.setsize(image.byteCount())
        frame = np.array(ptr).reshape(height, width, 3)
    
        if self.is_yolo_enabled and hasattr(self, 'yolo_detection_thread'):
            self.yolo_detection_thread.update_frame(frame)
    
        frame_display = frame.copy()
        image = QImage(frame_display.data, frame_display.shape[1], frame_display.shape[0], QImage.Format_RGB888)
        self.control_tab.camera_label.setPixmap(QPixmap.fromImage(image).scaled(
            self.control_tab.camera_label.width(),
            self.control_tab.camera_label.height(),
            Qt.KeepAspectRatio
        ))
        
        if self.is_recording and self.video_writer is not None:
            self.video_writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))


    def capture_photo(self):
        if hasattr(self, 'camera_thread') and self.camera_thread.isRunning():
            pixmap = self.control_tab.camera_label.pixmap()
            if pixmap:
                # Save the pixmap as an image file
                timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_HHmmss")
                filename = f"photo_{timestamp}.png"
                pixmap.save(filename)
                print(f"Photo saved as {filename}")
        else:
            print("Camera is not running.")

    def toggle_video_recording(self):
        if not self.is_recording:
            timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_HHmmss")
            filename = f"video_{timestamp}.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))
            if not self.video_writer.isOpened():
                print("Failed to open video writer.")
                self.video_writer = None
                return
            self.is_recording = True
            self.control_tab.record_video_button.setText("Stop Recording")
            self.control_tab.record_video_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))
            print(f"Recording started: {filename}")
        else:
            self.is_recording = False
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
                print("Recording stopped and saved.")
            self.control_tab.record_video_button.setText("Start Recording")
            self.control_tab.record_video_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def move_forward(self):
        print("Hexapod moving forward")
        self.robot.move_forward()
        self.update_plot()

    def move_backward(self):
        print("Hexapod moving backward")
        self.robot.move_backward()
        self.update_plot()

    def turn_left(self):
        print("Hexapod turning left")
        self.robot.turn_left()
        self.update_plot()

    def turn_right(self):
        print("Hexapod turning right")
        self.robot.turn_right()
        self.update_plot()

    def stop_movement(self):
        print("Hexapod stopped")
        self.robot.stop()
        self.update_plot()

    def update_plot(self):
        draw_robot(self.robot, self.ax)
        self.ax.grid(True)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Adjust margins and padding
        self.figure.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

        self.canvas.draw()


