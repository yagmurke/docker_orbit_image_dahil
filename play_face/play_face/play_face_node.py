import os
import sys
import rclpy
import cv2
import numpy as np
import librosa
import time
import pygame
import socket,subprocess

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QFrame, QStackedWidget
from PyQt5.QtCore import QThread, pyqtSignal, QUrl, QTimer, Qt, QPropertyAnimation, QPoint
from PyQt5.QtGui import QImage, QPixmap


from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float32, String, Bool
from orbit_command_msgs.srv import Face,Tasks
from arduino_msgs.msg import WebSocket
from PyQt5.QtWidgets import QShortcut
from PyQt5.QtGui import QKeySequence
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

RESOURCE = os.path.join(get_package_share_directory('play_face'), 'resource')
VIDEOS = os.path.join(RESOURCE, 'videos')
ADS = os.path.join(VIDEOS, 'ads')
FACES = os.path.join(RESOURCE, 'images', 'faces')
LIPS = os.path.join(RESOURCE, 'images', 'lips')
AUDIO_FOLDER = os.path.join(RESOURCE,'audios')

FACE_IDS = {
    0: "blinking.mp4",
    1: "breathe.mp4",
    2: "compassion.mp4",
    3: "curious.mp4",
    4: "error.mp4",
    5: "heart_eyes.mp4",
    6: "hello.mp4",
    7: "loading.mp4",
    8: "playful.mp4",
    9: "shy.mp4",
    10: "star_eyes.mp4",
    11: "surprised.mp4",
    12: "thank_you.mp4",

    13: "merhaba"
}

class AdsView(QWidget):
    def __init__(self, face_thread:QThread):
        super().__init__()
        self.setObjectName("ad_view")
        self.setStyleSheet("QWidget#ad_view{background-color: black;}")
        self.face_thread = face_thread

        self.face_thread.audio_started.connect(self.audio_callback)
        self.face_thread.websocket_status.connect(self.websocket_callback)
        self.window_width = 1280
        self.window_height = 720
        self.scale = 3.5
        self.icon_width = int(640 / self.scale)
        self.icon_height = int(905 / self.scale)
        self.x_pos = int(self.window_width - self.icon_width - 12)
        self.y_pos = int(self.window_height - self.icon_height - 12)
        self.center_widget = QWidget(self)
        self.center_widget.setMinimumHeight(self.window_height)
        self.center_widget.setMinimumWidth(self.window_width)
        self.media_widget = QLabel(self.center_widget)
        self.media_widget.setGeometry(0,0, 1280,720)

        self.robot_icon = QLabel(self.center_widget)
        self.robot_icon.setGeometry(self.x_pos, self.y_pos, self.icon_width, self.icon_height)
        self.robot_icon.setPixmap(self.scale_pixel("orbit_v5.png", self.icon_width, self.icon_height))
        self.center_widget.setObjectName("center_object")
        self.center_widget.setStyleSheet("QWidget#center_object{background-color: black;}")
        self.robot_icon.setStyleSheet("background-color: rgba(0, 255, 255, 0);")
        self.robot_icon.hide()
        
        # self.robot_icon.setStyleSheet("background-color: blue;")

        self.face_frame = QFrame(self.center_widget)
        self.main_face = MainFace(self.face_frame, self.face_thread)
        self.main_face.setParent(self.center_widget)

        self.web_label = QLabel(self.center_widget)
        self.web_label.move(20,20)
        # self.web_label.setMinimumHeight(50)
        self.web_label.setMinimumWidth(300)
        self.web_label.setText(f"{self.get_wifi_ssid()}: ws//{self.get_local_ip()}:9090")
        self.web_label.setStyleSheet("color: white;")

        layout = QVBoxLayout()
        layout.addWidget(self.center_widget)
        # layout.addWidget(self.web_label)
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)
        # end_pos_x = int(self.x_pos + (self.icon_width - 128) /2)
        # end_pos_y = int(self.y_pos + (self.icon_height / 9))
        # self.main_face.resize_face(128, 72)
        # self.main_face.setGeometry(end_pos_x, end_pos_y, 128, 72)
        # self.start_reverse_animation()
    def get_local_ip(self):
        """Yerel IP adresini döndürür."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        except Exception:
            ip = "10.42.0.1"
        finally:
            s.close()
        return ip
    def get_wifi_ssid(self):
        """Ubuntu'da bağlı olunan Wi-Fi ağının SSID'sini döndürür."""
        try:
            result = subprocess.run(['nmcli', '-t', '-f', 'active,ssid', 'device', 'wifi'], stdout=subprocess.PIPE)
            wifi_info = result.stdout.decode('utf-8').strip().split('\n')
            
            # Aktif bağlantıyı bul ve SSID'yi döndür
            for info in wifi_info:
                if info.startswith('yes'):
                    return info.split(':')[1]
            return "unknown"
        except Exception as e:
            return "unknown"
    def audio_callback(self, msg):
        if msg:
            self.start_animation()
            self.media_widget.show()
        else:
            self.media_widget.hide()
            self.start_reverse_animation()
    def websocket_callback(self, msg):
        print(f"WebSocket status: {msg[0]}, IP: {msg[1]}, Name: {msg[2]}")
        self.web_status = msg[0]
        self.web_ip = msg[1]
        self.web_name = msg[2]

        if self.web_status:
            self.web_label.setText(f"{self.web_name}: ws://{self.web_ip}:9090")
        else:
            self.web_label.setText("")

    def start_animation(self):
        self.play("cnc.mp4")
        self.diff = 0.9
        self.timer = QTimer()
        self.timer.timeout.connect(self.animate_face)
        self.timer.start(40)
    def start_reverse_animation(self):
        self.robot_icon.hide()
        self.diff = 0.9
        self.timer_reverse = QTimer()
        self.timer_reverse.timeout.connect(self.reverse_animate_face)
        self.timer_reverse.start(30)

    def animate_face(self):
        w = self.main_face.max_width
        h = self.main_face.max_height
        x0 = int((self.window_width - w) / 2)
        y0 = int((self.window_height - h) / 2)
        end_pos_x = int(self.x_pos + (self.icon_width - 98) /2)
        end_pos_y = int(self.y_pos + (self.icon_height / 6.5))
        m = (end_pos_y - y0) / (end_pos_x - x0)
        if w > 98 and h > 48:
            self.main_face.resize_face(int(w * self.diff), int(h * self.diff))
            self.main_face.setGeometry(int(x0 *2), int(m * x0 + y0), int(w * self.diff), int(h * self.diff))
        else:
            self.main_face.resize_face(98, 48)
            self.main_face.setGeometry(end_pos_x, end_pos_y, 98, 48)
            self.timer.stop()
            self.robot_icon.show()
        self.diff -= 0.01
    
    def reverse_animate_face(self):
        w = self.main_face.max_width
        h = self.main_face.max_height
        x0 = int((self.window_width - w) / 2)
        y0 = int((self.window_height - h) / 2)
        end_pos_x = 0
        end_pos_y = 0
        if y0 >0 and x0 > 0:
            m = (end_pos_y - y0) / (end_pos_x - x0)
        if w < self.window_width and h < self.window_height:
            self.main_face.resize_face(int(w * 1.1), int(h * 1.1))
            self.main_face.setGeometry(int(x0 *2), int(m * x0 /2  + y0), int(w * 1.05), int(h * 1.05))
        else:
            self.main_face.resize_face(self.window_width, self.window_height)
            self.main_face.setGeometry(end_pos_x, end_pos_y, self.window_width, self.window_height)
            self.timer_reverse.stop()
        self.diff += 0.01
    
    def play(self, video_name):
        self.cap = cv2.VideoCapture(os.path.join(ADS, video_name))
        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self.update_frame)
        self.video_timer.start(30)
    
    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (self.window_width, self.window_height), interpolation = cv2.INTER_LINEAR)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            q_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.media_widget.setPixmap(QPixmap.fromImage(q_image))
        
        else:
            print("video stopped")
            self.start_reverse_animation()
            self.video_timer.stop()
    
    def scale_pixel(self, file_name, w, h):
        pixel = QPixmap(os.path.join(FACES, file_name))
        return pixel.scaled(w, h, Qt.KeepAspectRatio)

class MainFace(QWidget):
    def __init__(self, face_frame:QFrame, face_thread:QThread):
        super().__init__()

        self.max_width = int(1280/1)
        self.max_height = int(720/1)
        self.face_scale = 1.0
        self.face_publisher = PlayFaceThread()
        self.feelings_indx = 0
        self.motions_id = 0
        self.motions_name = ""

        #create ui
        self.face_frame = face_frame
        self.face_frame.setMinimumSize(self.max_width, self.max_height)
        self.face_frame.setMaximumSize(self.max_width, self.max_height)
        self.video_label = QLabel(self.face_frame)
        self.lip_label = QLabel(self.face_frame)
        # self.setGeometry(0,0,1280, 720)
        self.setStyleSheet('background-color: transparent;')

        layout = QVBoxLayout()
        layout.addWidget(self.face_frame)
        layout.setContentsMargins(0, 0, 0,0)
        layout.setAlignment(Qt.AlignCenter)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        self.cap = cv2.VideoCapture(os.path.join(VIDEOS, "blinking.mp4"))

        self.lip_images = {
            "1": cv2.imread(os.path.join(LIPS, "1_face_.png"), cv2.IMREAD_UNCHANGED),  # Kapalı ağız
            "1_2": cv2.imread(os.path.join(LIPS, "1_face.png"), cv2.IMREAD_UNCHANGED),
            "2": cv2.imread(os.path.join(LIPS, "2_face.png"), cv2.IMREAD_UNCHANGED),
            "4": cv2.imread(os.path.join(LIPS, "3_face.png"), cv2.IMREAD_UNCHANGED),  # Yarı açık ağız
            "3": cv2.imread(os.path.join(LIPS, "4_face.png"), cv2.IMREAD_UNCHANGED),
            "5": cv2.imread(os.path.join(LIPS, "5_face.png"), cv2.IMREAD_UNCHANGED)  # Tam açık ağız
        }

        self.play_face_thread = face_thread
        self.play_face_thread.face_id.connect(self.change_face_callbackk)
        self.play_face_thread.decibel_msg.connect(self.update_lip)
        self.play_face_thread.motions_name.connect(self.motions_callback)

        self.previous_time = time.time()
    
    def resize_face(self, w, h):
        self.face_frame.setMinimumSize(w, h)
        self.face_frame.setMaximumSize(w, h)
        self.max_width = w
        self.max_height = h
        self.update_frame()
        self.update_lip(10)
    
    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            new_width = int(self.max_width * self.face_scale)
            new_height = int(self.max_height * self.face_scale)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (new_width, new_height), interpolation = cv2.INTER_LINEAR)
            # x1 = int((new_width - self.max_width) * 1.8)
            x1 = (int(new_width / 8))
            x2 = new_width - x1
            crop = frame[0:new_height, x1:x2]
            h, w, ch = crop.shape
            bytes_per_line = ch * w
            q_image = QImage(crop.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setGeometry(int(x1), 0, w, h)
            self.video_label.setPixmap(QPixmap.fromImage(q_image))
        
        else:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

            self.cap = cv2.VideoCapture(os.path.join(VIDEOS, "blinking.mp4"))
            self.lip_label.show()
            self.face_publisher.node.get_logger().info(f'{self.feelings_indx}')

            if self.motions_id == 1:
                print("merhaba")
                self.face_publisher.mp3_player("merhaba")
            self.motions_id = 0

    def motions_callback(self, msg):
        self.motions_name = msg
        if self.motions_name == "greeting":
            print("greeting")
            self.motions_id = 1
            request = Face.Request()
            request.face = 6
            self.face_publisher.node.create_client(Face, 'change_face').call_async(request)

    def update_lip(self, decibel):
        if decibel <= 15:
            lip_time = time.time() - self.previous_time
            if lip_time < 0.25:
                image = self.lip_images["2"]
            else:
                image = self.lip_images["1"]
        elif decibel <= 60:
            image = self.lip_images["2"]
        elif decibel <= 80:
            image = self.lip_images["3"]
        elif decibel <= 100:
            image = self.lip_images["4"]
        elif decibel <= 120:
            image = self.lip_images["5"]
        else:
            image = self.lip_images["1_2"]
        # self.previous_time = time.time()80

        # frame = cv2.imread(os.path.join(LIPS, image), cv2.IMREAD_UNCHANGED)
        frame = image
        new_width = int(self.max_width * self.face_scale)
        new_height = int(self.max_height * self.face_scale)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (new_width, new_height), interpolation = cv2.INTER_LINEAR)
        # x1 = int((new_width - self.max_width) * 1.8)
        x1 = (int(new_width / 8))
        x2 = new_width - x1
        crop = frame[int(new_height/1.5):new_height, x1:x2]
        h, w, ch = crop.shape
        bytes_per_line = ch * w
        q_image = QImage(crop.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888)
        self.lip_label.setGeometry(int(x1), int(new_height/1.3), w, h)
        self.lip_label.setPixmap(QPixmap.fromImage(q_image))
    
    def change_face_callbackk(self, face_id):
        print("face_id")
        self.cap = cv2.VideoCapture(os.path.join(VIDEOS, FACE_IDS[face_id]))
        self.feelings_indx= face_id
        self.lip_label.hide()

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.face_thread = PlayFaceThread()

        self.setObjectName("main_window")
        self.setStyleSheet(""" QWidget#main_window{background-color: black;}""")

        self.stacked_widget = QStackedWidget(self)

        self.face_center = QFrame()

        # self.main_face = MainFace(self.face_center, self.face_thread)
        self.ads_view = AdsView(self.face_thread)

        # self.stacked_widget.addWidget(self.main_face)
        self.stacked_widget.addWidget(self.ads_view)

        layout = QVBoxLayout()
        layout.addWidget(self.stacked_widget)
        layout.setContentsMargins(0, 0, 0,0)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.singleShot(5000, self.change_page)
        self.stacked_widget.setCurrentWidget(self.ads_view)

        
        self.face_thread.start()
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.shortcut = QShortcut(QKeySequence('Q'), self)
        self.shortcut.activated.connect(self.close)
    def change_page(self):
        # self.stacked_widget.setCurrentWidget(self.ads_view)
        # self.ads_view.start_animation()
        ...

class PlayFaceThread(QThread):
    face_id = pyqtSignal(int)
    decibel_msg = pyqtSignal(int)
    audio_started = pyqtSignal(bool)
    motions_name = pyqtSignal(str)
    websocket_status = pyqtSignal(list)  # Add new signal for WebSocket status
    def __init__(self):
        super().__init__()

        self.node = Node("play_face_node")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Depth of 1 to simulate latched topic
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.node.get_logger().info("Starting play_face_node")

        self.mp3_publisher = self.node.create_publisher(String, 'mp3_name', 10)

        self.node.create_service(Face, 'change_face', self.change_face_callback)
        self.node.create_service(Face, 'face_task', self.task_face_callback)
        self.node.create_subscription(Float32, 'decibel', self.decibel_sub, 1)
        self.node.create_subscription(String, 'motions', self.motions_sub, 1)
        
        self.subscription = self.node.create_subscription(
            Bool,
            '/audio_mode',
            self.listener_callback,
            10)
        self.subscription = self.node.create_subscription(
            WebSocket,
            '/websocket_status',
            self.websocket_callback,
            10)
        
    
    def mp3_player(self, mp3_name):
        msg = String()
        msg.data = mp3_name
        self.mp3_publisher.publish(msg)

    def task_face_callback(self, request, response):
        # self.node.get_logger().info(f"received {request.face}")
        self.face_index = (request.face)

        self.node.get_logger().info(f"Playing video: {FACE_IDS[self.face_index]}")
        video_path = os.path.join(VIDEOS, FACE_IDS[self.face_index])
        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        duration = int(frame_count / fps)
        self.node.get_logger().info(f"Video duration: {duration} seconds")
        cap.release()
        self.face_id.emit(self.face_index)
        time.sleep(duration)

        response.response = True
        return response
    
    def change_face_callback(self, request, response):
        self.node.get_logger().info(f"received {request.face}")
        self.face_id.emit(request.face)
        response.response = True
        return response
    
    def decibel_sub(self, msg):
        self.decibel_msg.emit(int(msg.data))
    
    def motions_sub(self, msg):
        self.motions_name.emit(msg.data)
    
    def listener_callback(self, msg):
        print(msg)
        self.audio_started.emit(msg.data)

    def websocket_callback(self, msg):
        self.node.get_logger().info(f"WebSocket status: {msg.status}, IP: {msg.ip}, Name: {msg.name}")
        
        status_list = [msg.status, msg.ip, msg.name]
        self.websocket_status.emit(status_list)  # Emit the WebSocket status
    
    def run(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init()
    app = QApplication(sys.argv)
    main_face = MainWindow()
    main_face.showFullScreen()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()