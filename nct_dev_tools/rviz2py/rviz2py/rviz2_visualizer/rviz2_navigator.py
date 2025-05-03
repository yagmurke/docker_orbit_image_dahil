import os
import sys

from ament_index_python import get_package_share_directory

from PyQt5.QtWidgets import (
    QApplication, 
    QWidget, 
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QLabel,
    QFrame
)

from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap, QIcon

from rviz2py.utils import Modes
from rviz2py.rviz2_visualizer import MapViewer
from rviz2py.rviz2_styles import NavigatorStyles

IMAGES = os.path.join(get_package_share_directory('rviz2py'), 'resource', 'images')

class Rviz2Navigator(QWidget):
    def __init__(self, parent=None):
        super(Rviz2Navigator, self).__init__(parent)
        self.setWindowTitle("Rviz2 Navigator")
        self.setObjectName("rviz2_navigator")

        self.navigator_styles = NavigatorStyles()
        self.setStyleSheet(self.navigator_styles.container('rviz2_navigator'))

        self.navigator_layout = QVBoxLayout(self)
        self.navigator_layout.setContentsMargins(0, 0, 0, 0)
        self.navigator_layout.setSpacing(0)

        self.map_viewer = MapViewer()
        self.map_viewer.setObjectName("map_viewer")
        self.map_viewer.arrow_released.connect(self.release_callback)
        self.navigator_layout.addWidget(self.top_frame_ui())
        self.navigator_layout.addWidget(self.map_viewer)

        self.set_mode(Modes.DRAG)


    
    def top_frame_ui(self):
        self.top_frame = QFrame(self)

        ### Top frames creation
        self.top_frame.setFrameStyle(QFrame.NoFrame | QFrame.Raised)
        self.top_frame.setObjectName("top_frame")
        self.move_camera_frame = QFrame(self.top_frame)
        self.move_camera_frame.setObjectName("move_camera_frame")
        self.pose_frame = QFrame(self.top_frame)
        self.pose_frame.setObjectName("pose_frame")
        self.goal_frame = QFrame(self.top_frame)
        self.goal_frame.setObjectName("goal_frame")
        self.save_map_frame = QFrame(self.top_frame)
        self.save_map_frame.setObjectName("save_map")
        self.cancel_frame = QFrame(self.top_frame)
        self.cancel_frame.setObjectName("cancel_frame")

        hlayout = QHBoxLayout(self.top_frame)
        hlayout.setContentsMargins(0, 0, 0, 0)
        top_frames = self.top_frame.findChildren(QFrame)

        for frame in top_frames:
            if top_frames.index(frame) != len(top_frames) - 1:
                frame.setStyleSheet(self.navigator_styles.menu_frame(frame.objectName()))
                frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
            else:
                frame.setFrameStyle(QFrame.NoFrame | QFrame.Raised)
            hlayout.addWidget(frame)
        
        hlayout.setAlignment(Qt.AlignLeft)
        self.top_frame.setLayout(hlayout)

        #### Buttons 
        self.move_camera_btn = QPushButton("", self.move_camera_frame, icon=QIcon(self.set_pixel("selection.png", 50)))
        self.pose_btn = QPushButton("", self.pose_frame, icon=QIcon(self.set_pixel("arrow.png", 60)))
        self.goal_btn = QPushButton("", self.goal_frame, icon=QIcon(self.set_pixel("arrow.png", 60)))
        self.save_map_btn = QPushButton("", self.save_map_frame, icon=QIcon(self.set_pixel('save.png', 50)))
        self.cancel_btn = QPushButton("", self.cancel_frame, icon=QIcon(self.set_pixel("close.png", 50)))

        self.move_camera_btn.clicked.connect(lambda: self.set_mode(Modes.DRAG))
        self.pose_btn.clicked.connect(lambda: self.set_mode(Modes.POS))
        self.goal_btn.clicked.connect(lambda: self.set_mode(Modes.GOAL))
        self.cancel_btn.clicked.connect(lambda: self.map_viewer.rviz2_node_thread.cancel_goal())

        # Labels 
        self.move_camera_label = QLabel(self.move_camera_frame)
        self.pose_label = QLabel(self.pose_frame)
        self.goal_label = QLabel(self.goal_frame)
        self.save_map_label = QLabel(self.save_map_frame)
        self.cancel_label = QLabel(self.cancel_frame)

        self.move_camera_label.setText("Move Camera")
        self.pose_label.setText("2D Pose Estimate")
        self.goal_label.setText("Nav2 Goal")
        self.save_map_label.setText("Save map")
        self.cancel_label.setText("Cancel Goal")

        for button in self.top_frame.findChildren(QPushButton):
            button.setIconSize(QSize(24, 24))
            button.setMaximumHeight(30)
            button.setMaximumWidth(30)
            button.setMinimumHeight(30)
            button.setMinimumWidth(30)
        
        for label in self.top_frame.findChildren(QLabel):
            label.setAlignment(Qt.AlignCenter)

        ### Button frames layout 
        move_camera_lyt = QVBoxLayout(self.move_camera_frame)
        move_camera_lyt.addWidget(self.move_camera_btn)
        move_camera_lyt.addWidget(self.move_camera_label)
        self.move_camera_frame.setLayout(move_camera_lyt)

        pose_lyt = QVBoxLayout(self.pose_frame)
        pose_lyt.addWidget(self.pose_btn)
        pose_lyt.addWidget(self.pose_label)
        self.pose_frame.setLayout(pose_lyt)

        goal_lyt = QVBoxLayout(self.goal_frame)
        goal_lyt.addWidget(self.goal_btn)
        goal_lyt.addWidget(self.goal_label)
        self.goal_frame.setLayout(goal_lyt)

        save_map_lyt = QVBoxLayout(self.save_map_frame)
        save_map_lyt.addWidget(self.save_map_btn)
        save_map_lyt.addWidget(self.save_map_label)
        self.save_map_frame.setLayout(save_map_lyt)

        cancel_lyt = QVBoxLayout(self.cancel_frame)
        cancel_lyt.addWidget(self.cancel_btn)
        cancel_lyt.addWidget(self.cancel_label)
        self.cancel_frame.setLayout(cancel_lyt)

        self.top_frame.setStyleSheet(self.navigator_styles.top_frame())

        return self.top_frame
    
    def set_pixel(self, file_name, scale):
        pixel = QPixmap(os.path.join(IMAGES, file_name))
        return pixel.scaled(scale, scale)
    
    def set_mode(self, mode):
        self.map_viewer.change_mode(mode)
        for button in self.top_frame.findChildren(QPushButton):
            button.setStyleSheet(self.navigator_styles.inactive_btn())
        if mode == Modes.DRAG:
            self.move_camera_btn.setStyleSheet(self.navigator_styles.active_btn())
        else:
            self.sender().setStyleSheet(self.navigator_styles.active_btn())
    
    def release_callback(self, msg):
        if msg:
            self.set_mode(Modes.DRAG)

        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    rviz2_navigator = Rviz2Navigator()
    rviz2_navigator.show()
    sys.exit(app.exec_())