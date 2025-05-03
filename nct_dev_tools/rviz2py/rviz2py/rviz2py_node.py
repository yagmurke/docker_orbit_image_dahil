import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QPushButton, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt
from rviz2py.rviz2_visualizer import MapView, Modes

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("RViz2View")

        buttons_frame = QFrame(self)
        layout = QVBoxLayout(self)
        h_layout = QHBoxLayout(buttons_frame)

        # Create an instance of MapView and add it to the layout
        self.map_view = MapView()
        camera_btn = QPushButton("Camera", self)
        pose_btn = QPushButton("Pose", self)
        send_btn = QPushButton("Send", self)
        keepout = QPushButton("KeepOut", self)
        save_pgm = QPushButton("SavePGM", self)
        camera_btn.clicked.connect(lambda: self.send_callback(Modes.DRAG))
        pose_btn.clicked.connect(lambda: self.send_callback(Modes.POS))
        send_btn.clicked.connect(lambda: self.send_callback(Modes.GOAL))
        keepout.clicked.connect(lambda: self.send_callback(Modes.KEEPOUT))
        save_pgm.clicked.connect(self.save_keep)
        layout.addWidget(self.map_view)
        layout.addWidget(buttons_frame)
        h_layout.addWidget(camera_btn)
        h_layout.addWidget(pose_btn)
        h_layout.addWidget(send_btn)
        h_layout.addWidget(keepout)
        h_layout.addWidget(save_pgm)
        h_layout.setAlignment(Qt.AlignLeft)
    
    def send_callback(self, mode):
        self.map_view.set_cursor_mode(mode)
    
    def save_keep(self):
        self.map_view.save_keepout('outputmap.pgm')

if __name__ == "__main__":
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())