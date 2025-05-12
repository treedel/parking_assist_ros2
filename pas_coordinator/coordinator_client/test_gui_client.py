import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QFrame
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from geometry_msgs.msg import Twist
from CoordinatorClientInterface import CoordinatorClientInterface
import datetime

from PyQt5.QtGui import QFont, QPixmap

interface = CoordinatorClientInterface("127.0.0.1", 5555)

class ROS2Interface(Node):
    def __init__(self):
        super().__init__('dashboard_listener')
        self.speed = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        self.speed = (msg.linear.x ** 2 + msg.linear.y ** 2 + msg.linear.z ** 2) ** 0.5

class Dashboard(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.number = "CAR001"
        self.group_name = None
        self.init_ui()
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_dashboard)
        self.update_timer.start(100)

    def init_ui(self):
        self.setWindowTitle("Parking Assistant Dashboard")
        self.setStyleSheet("background-color: #1e1e1e; color: #ffffff;")
        self.setFixedSize(300, 600)

        font_large = QFont("Arial", 18, QFont.Bold)
        font_medium = QFont("Arial", 14)

        layout = QVBoxLayout()

        self.time_label = QLabel("--:--:--")
        self.time_label.setFont(font_large)
        self.time_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.time_label)

        self.car_icon = QLabel()
        pixmap = QPixmap("car.png")
        scaled_pixmap = pixmap.scaled(256, 256, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.car_icon.setPixmap(scaled_pixmap)
        self.car_icon.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.car_icon)

        self.speed_label = QLabel("0.0 m/s")
        self.speed_label.setFont(font_medium)
        self.speed_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.speed_label)

        layout.addWidget(self._make_separator())

        self.reserve_button = QPushButton("Reserve Slot")
        self.reserve_button.setFont(font_medium)
        self.reserve_button.clicked.connect(self.reserve_slot)
        layout.addWidget(self.reserve_button)

        self.relinquish_button = QPushButton("Relinquish Slot")
        self.relinquish_button.setFont(font_medium)
        self.relinquish_button.clicked.connect(self.relinquish_slot)
        layout.addWidget(self.relinquish_button)

        self.guidance_button = QPushButton("Start Guidance System")
        self.guidance_button.setFont(font_medium)
        self.guidance_button.clicked.connect(self.start_guidance_system)
        layout.addWidget(self.guidance_button)

        layout.addWidget(self._make_separator())

        self.status_label = QLabel("Status: Idle")
        self.status_label.setFont(font_medium)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    def _make_separator(self):
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setFrameShadow(QFrame.Sunken)
        sep.setStyleSheet("color: #888888;")
        return sep

    def reserve_slot(self):
        if self.group_name:
            self.status_label.setText("Status: Already Reserved")
            return
        res = interface.reserve_parking(self.number)
        if res != "False":
            self.group_name = res
            self.status_label.setText(f"Status: Reserved Group {res}")
        else:
            self.status_label.setText("Status: Reservation Failed")

    def relinquish_slot(self):
        res = interface.relinquish_parking(self.number)
        if res:
            self.group_name = None
            self.status_label.setText("Status: Slot Relinquished")
        else:
            self.status_label.setText("Status: Relinquish Failed")

    def start_guidance_system(self):
        if self.group_name:
            pos = interface.get_group_position(self.group_name)
        else:
            pos = interface.get_group_position("EXIT")
        self.status_label.setText("Status: Arrived at Destination")

    def update_dashboard(self):
        now = datetime.datetime.now().strftime("%H:%M:%S")
        self.time_label.setText(now)
        self.speed_label.setText(f"{self.ros_node.speed:.3f} m/s")

def main():
    rclpy.init()
    ros_node = ROS2Interface()
    app = QApplication(sys.argv)
    dashboard = Dashboard(ros_node)
    dashboard.show()

    ros_spin_timer = QTimer()
    ros_spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    ros_spin_timer.start(10)

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
