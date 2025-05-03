import os
import sys

from ament_index_python import get_package_share_directory

from PyQt5.QtWidgets import (
    QWidget, 
    QApplication, 
    QStackedWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QPushButton,
    QDialog,
    QLineEdit
)

from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap, QIcon

from rviz2py.rviz2_visualizer import Rviz2Navigator, MapEditor
from rviz2py.rviz2_styles import NavigatorStyles

IMAGES = os.path.join(get_package_share_directory('rviz2py'), 'resource', 'images')

class Rviz2Viewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rviz2py")
        self.setObjectName("rviz2_viewer")

        self.pages = QStackedWidget(self)
        self.rviz_navigator = Rviz2Navigator()
        self.map_editor = MapEditor() 
        self.navigator_styles = NavigatorStyles()
        self.left_frame_ui()

        self.setStyleSheet(self.navigator_styles.container('rviz2_viewer'))

        self.rviz_layout = QHBoxLayout(self)
        self.rviz_layout.setContentsMargins(0, 0, 0, 0)
        self.rviz_layout.setSpacing(0)
        self.rviz_layout.addWidget(self.left_frame)
        self.rviz_layout.addWidget(self.pages)
        self.pages.addWidget(self.rviz_navigator)
        self.pages.addWidget(self.map_editor)
    
    def left_frame_ui(self):
        self.left_frame = QFrame(self)
        self.left_frame.setFrameStyle(QFrame.NoFrame | QFrame.Raised)
        self.left_frame.setObjectName("left_frame")
        self.left_frame.setStyleSheet(self.navigator_styles.left_frame())
        self.left_frame.setMinimumWidth(200)

        self.rviz_btn = QPushButton("Navigation", self.left_frame, icon=QIcon(self.set_pixel("navigation.png")))
        self.map_editor_btn = QPushButton("Edit Map", self.left_frame, icon=QIcon(self.set_pixel("wall.png")))

        for button in self.left_frame.findChildren(QPushButton):
            button.setMinimumHeight(30)
            button.setIconSize(QSize(24, 24))

        self.rviz_btn.clicked.connect(lambda: self.pages.setCurrentWidget(self.rviz_navigator))
        self.map_editor_btn.clicked.connect(lambda: self.pages.setCurrentWidget(self.map_editor))

        buttons_layout = QVBoxLayout(self.left_frame)
        buttons_layout.setAlignment(Qt.AlignTop)
        buttons_layout.addWidget(self.rviz_btn)
        buttons_layout.addWidget(self.map_editor_btn)
        buttons_layout.setContentsMargins(0, 10, 0, 10)

        self.left_frame.setLayout(buttons_layout)
    
    def set_pixel(self, file_name):
        pixel = QPixmap(os.path.join(IMAGES, file_name))
        return pixel

class SaveMapDialog(QDialog):
    def __init__(self):
        pass
        # super().__ini


if __name__ == "__main__":
    import rclpy
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    rviz2_viewer = Rviz2Viewer()
    rviz2_viewer.show()
    sys.exit(app.exec_())