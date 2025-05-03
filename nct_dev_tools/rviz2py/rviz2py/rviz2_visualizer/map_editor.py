import sys
import os
import subprocess
import math
import yaml

from enum import Enum

from ament_index_python import get_package_share_directory

from PyQt5.QtWidgets import (
    QApplication, 
    QVBoxLayout, 
    QGridLayout,
    QWidget, 
    QPushButton, 
    QHBoxLayout, 
    QFrame, 
    QLabel, 
    QComboBox,
    QGraphicsScene,
    QGraphicsView,
    QMessageBox,
)
from PyQt5.QtCore import Qt, QSize

from PyQt5.QtGui import (
    QCursor,
    QPen,
    QPolygonF,
    QBrush,
    QColor,
    QPixmap,
    QKeySequence, 
    QIcon,
    QPainter
)


from rviz2py.utils import SceneSaver
from rviz2py.rviz2_scene_items import HovablePolygon
from rviz2py.rviz2_styles import NavigatorStyles


IMAGES = os.path.join(get_package_share_directory('rviz2py'), 'resource', 'images')
MAP_PATH = os.path.join(get_package_share_directory('rviz2py'), 'maps')
CONFIG = os.path.join(get_package_share_directory('rviz2py'), 'config')

class Modes(Enum):
    DRAG = 1
    POS = 2
    GOAL = 3
    KEEPOUT = 4
    WALL = 5
    WHITE_SPACE = 6

class EditorViewer(QGraphicsView):
    def __init__(self, parent=None):
        super(EditorViewer, self).__init__(parent)
        self.setScene(QGraphicsScene(self))

        self.setRenderHint(QPainter.Antialiasing)

        self.scene_saver = SceneSaver()
        #Initial Variables
        self.right_button_pressed = False
        self.last_mouse_pos = None
        self.mode = Modes.DRAG
        self.line_item = None
        self.line_count = 1
        self.line_items = []
        self.points = []

        self.line_color = Qt.darkYellow
        self.painting_color = QColor('#FFA5007F')

        self.icon_path = ''

        self.modes = {
            Modes.POS: ['location', 'arrowm', Qt.black],
            Modes.GOAL: ['icon', 'send_posem', Qt.black],
            Modes.KEEPOUT: ['draw', 'draw', QColor(255, 165, 0, 80)],
            Modes.WHITE_SPACE: ['draw', 'draw', Qt.white]
        }

    def load_progress(self, mapname):
        self.scene().clear()
        map_path = os.path.join(MAP_PATH, f"{mapname}.pgm")
        progress_path = os.path.join(MAP_PATH, f"{mapname}.nct")
        map_item = self.scene_saver.load_map(map_path)
        self.scene().addItem(map_item)
        if os.path.isfile(progress_path):
            progress_items = self.scene_saver.load_progress(progress_path)
            if len(progress_items) > 0:
                for item in progress_items:
                    self.scene().addItem(item)        
        self.setMouseTracking(True)
        self.setMouseTracking(False)
    
    def save_map(self, mapname):
        map_path = os.path.join(MAP_PATH, f"{mapname}.pgm")
        keepout_path = os.path.join(MAP_PATH, 'keepout', f"{mapname}.pgm")
        progress_path = os.path.join(MAP_PATH, f"{mapname}.nct")
        self.scene_saver.save_map(self.scene(), map_path, keepout_path, progress_path)
        map_yaml = os.path.join(MAP_PATH, f"{mapname}.yaml")
        copy_process = subprocess.Popen(
            f"cp {map_yaml} {os.path.join(MAP_PATH, 'keepout')}",
            stdout=subprocess.PIPE,
            shell=True,
            preexec_fn=os.setsid
        )
    
    def setpolygonhover(self, enabled):
        for item in self.scene().items():
            if isinstance(item, HovablePolygon):
                item.setAcceptHoverEvents(enabled)
    
    def mousePressEvent(self, event):
        mouse_position = self.mapToScene(event.pos())
        # if self.map_item.contains(mouse_position):
        if event.button() == Qt.RightButton or event.button() == Qt.MiddleButton:
            if self.mode == Modes.DRAG:
                self.right_button_pressed = True
                self.setpolygonhover(False)
                self.last_mouse_pos = event.pos()
                self.setDragMode(QGraphicsView.NoDrag)
            
        elif event.button() == Qt.LeftButton:
            if self.mode == Modes.DRAG:
                self.setpolygonhover(False)
                self.setDragMode(QGraphicsView.ScrollHandDrag)
                self.setCursor(QCursor(Qt.ArrowCursor))
            else:
                self.setDragMode(QGraphicsView.NoDrag)
                if self.mode == Modes.KEEPOUT or self.mode == Modes.WHITE_SPACE or self.mode == Modes.WALL:
                    self.points.append(mouse_position)
                    self.update_line()
                    self.setMouseTracking(True)
        super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event):
        mouse_position = self.mapToScene(event.pos())

        if self.right_button_pressed and self.last_mouse_pos is not None:
            delta = event.pos() - self.last_mouse_pos
            angle = delta.x()  # Change the rotation angle based on horizontal mouse movement
            self.rotate(angle / 10)
            self.last_mouse_pos = event.pos()
        else:
            # if self.map_item.contains(mouse_position):
            if self.mode == Modes.KEEPOUT or self.mode == Modes.WHITE_SPACE or self.mode == Modes.WALL:
                if len(self.points) > 0:
                    self.draw_line(mouse_position)
        
        super().mouseMoveEvent(event)
    
    def mouseReleaseEvent(self, event):
        self.right_button_pressed = False
    
    # def keyPressEvent(self, event):
        
    #     if QKeySequence(event.modifiers() | event.key()) == QKeySequence.Cancel:
    #         self.cancel_keepout()
        
    #     elif event.key() == Qt.Key_Return:
    #         self.leave_drawing()
    
    def draw_line(self, current_point):
        if self.line_item:
            self.scene().removeItem(self.line_item)
        width = current_point.x() 
        height = current_point.y() 
        x = self.points[self.line_count - 1].x()
        y = self.points[self.line_count - 1].y()
        if len(self.points) > 1:
            x0 = self.points[0].x()
            y0 = self.points[0].y()
            s = math.sqrt((width - x0)** 2 + (height - y0) ** 2)
            if s < 10:
                width = x0
                height = y0
        self.line_item = self.scene().addLine(x, y, width, height, pen=QPen(self.line_color))
    
    def update_line(self):
        if len(self.points) > 1:
            x0 = self.points[0].x()
            y0 = self.points[0].y()
            x = self.points[self.line_count - 1].x()
            y = self.points[self.line_count - 1].y()
            x2 = self.points[self.line_count].x()
            y2 = self.points[self.line_count].y()
            s = math.sqrt((x2 - x0)** 2 + (y2 - y0) ** 2)
            if self.mode == Modes.KEEPOUT or self.mode == Modes.WHITE_SPACE:
                if s < 10:
                    self.points[self.line_count] = self.points[0]
                    self.draw_polygon()
                    self.scene().removeItem(self.line_item)
                    for i in self.line_items:
                        self.scene().removeItem(i)
                    self.line_count = 1
                    self.points = []
                    self.line_items = []
                else:
                    line_item = self.scene().addLine(x, y, x2, y2, pen=QPen(self.line_color))
                    self.line_items.append(line_item)
                    self.line_count += 1
            elif self.mode == Modes.WALL:
                self.line_count += 1
                pen = QPen(Qt.black)
                pen.setWidth(3)
                line_item = self.scene().addLine(x, y, x2, y2, pen)
                self.line_items.append(line_item)
    
            self.redo_stack.clear()
    
    def draw_polygon(self):
        if len(self.points) > 1:
            self.polygon = QPolygonF(self.points)
            pol = HovablePolygon(self.polygon)
            pol.setBrush(QBrush(self.painting_color))
            self.scene().addItem(pol)

    
    def cancel_keepout(self):
        self.setMouseTracking(False)
        if self.line_item:
            self.scene().removeItem(self.line_item)
        if len(self.line_items) > 0:
            for i in self.line_items:
                self.scene().removeItem(i)
        self.line_items = []
        self.points = []
        self.line_count = 1
    
    def leave_drawing(self):
        if self.mode == Modes.WALL:
            self.line_items = []
        self.cancel_keepout()
        self.set_cursor_mode(Modes.DRAG)
    
    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.zoom(1.1)
        else:
            self.zoom(0.9)

    def zoom(self, factor):
        self.scale(factor, factor)
    
    def set_custom_cursor(self, cursor_path, width, height):
        pixmap = QPixmap(cursor_path)
        scaled_pixmap = pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        custom_cursor = QCursor(scaled_pixmap)
        self.setCursor(custom_cursor)
    
    def set_cursor_mode(self, mode):
        self.mode = mode
        if self.mode != Modes.DRAG:
            self.setMouseTracking(True)
            self.setpolygonhover(True)
        else:
            self.setMouseTracking(False)
        self.setDragMode(QGraphicsView.NoDrag)
        if mode in self.modes:
            self.set_custom_cursor(f"{IMAGES}/{self.modes[mode][0]}.png", 20, 20)
            self.icon_path = f"{IMAGES}/{self.modes[mode][1]}.png"
            self.painting_color = self.modes[mode][2]

class MapEditor(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Map Editor")
        self.setObjectName("map_editor")
        self.styles = NavigatorStyles()
        self.setStyleSheet(self.styles.container('map_editor'))
        self.editor_layout = QVBoxLayout(self)
        self.editor_layout.setContentsMargins(0, 0, 0, 0)
        self.editor_layout.setSpacing(0)

        self.map_viewer = EditorViewer()
        self.map_viewer.setObjectName("map_viewer")
        self.editor_layout.addWidget(self.top_frame())
        self.editor_layout.addWidget(self.map_viewer)
        self.map_viewer.styleSheet()

    def top_frame(self):
        self.top_frame = QFrame(self)
        self.top_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.top_frame.setObjectName("top_frame")
        self.load_map_frame = QFrame(self.top_frame)
        self.load_map_frame.setObjectName("load_map_frame")
        self.selection_frame = QFrame(self.top_frame)
        self.selection_frame.setObjectName("selection_frame")
        self.map_frame = QFrame(self.top_frame)
        self.map_frame.setObjectName("map_frame")
        self.keepout_frame = QFrame(self.top_frame)
        self.keepout_frame.setObjectName("keepout_frame")
        self.save_frame = QFrame(self.top_frame)
        self.save_frame.setObjectName("save_frame")
        self.set_default_frame = QFrame(self.top_frame)
        self.set_default_frame.setObjectName("set_default_frame")
        self.apply_frame = QFrame(self.top_frame)
        self.apply_frame.setObjectName("apply_frame")
        self.close_frame = QFrame(self.top_frame)
        self.close_frame.setObjectName("close_frame")
        
        hlayout = QHBoxLayout(self.top_frame)
        hlayout.setContentsMargins(0, 0, 0, 0)
        top_frames = self.top_frame.findChildren(QFrame)
        for frame in top_frames:
            if top_frames.index(frame) != len(top_frames) - 1:
                frame.setStyleSheet(self.styles.menu_frame(frame.objectName()))
                frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
            else:
                frame.setFrameStyle(QFrame.NoFrame | QFrame.Raised)
            hlayout.addWidget(frame)

        hlayout.setAlignment(Qt.AlignLeft)
        self.top_frame.setLayout(hlayout)

        #create buttons to frames
        self.loadmap_btn = QPushButton("", self.load_map_frame, icon=QIcon(self.set_pixel('load.png', 50)))
        self.select_btn = QPushButton("", self.selection_frame, icon=QIcon(self.set_pixel('selection.png', 50)))
        self.wall_btn = QPushButton("", self.map_frame, icon=QIcon(self.set_pixel('wall.png', 48)))
        self.freespace = QPushButton("", self.map_frame, icon=QIcon(self.set_pixel('free.png', 64)))
        self.keepout = QPushButton("", self.keepout_frame, icon=QIcon(self.set_pixel('keepout.png', 50)))
        self.save_btn = QPushButton("", self.save_frame, icon=QIcon(self.set_pixel('save.png', 50)))
        self.set_default_btn = QPushButton("", self.set_default_frame, icon=(QIcon(self.set_pixel('default.png', 64))))
        self.apply_btn = QPushButton("", self.apply_frame, icon=QIcon(self.set_pixel('apply.png', 60)))
        self.close_btn = QPushButton("", self.close_frame, icon=QIcon(self.set_pixel('close.png', 50)))

        self.loadmap_btn.setToolTip("Load new map to workspace")
        self.select_btn.setToolTip("Move map")
        self.wall_btn.setToolTip("Create wall")
        self.freespace.setToolTip("Remove obstacles")
        self.keepout.setToolTip("Draw restricted area")
        self.save_btn.setToolTip("Save map")
        self.set_default_btn.setToolTip("Set this map as default")
        self.apply_btn.setToolTip("Appy")
        self.close_btn.setToolTip("Close")

        self.select_btn.clicked.connect(lambda: self.set_mode(Modes.DRAG))
        self.keepout.clicked.connect(lambda: self.set_mode(Modes.KEEPOUT))
        self.freespace.clicked.connect(lambda: self.set_mode(Modes.WHITE_SPACE))
        self.wall_btn.clicked.connect(lambda: self.set_mode(Modes.WALL))
        self.save_btn.clicked.connect(self.save_map)
        self.set_default_btn.clicked.connect(self.save_default_map)
        self.loadmap_btn.clicked.connect(self.load_map)
        self.apply_btn.clicked.connect(self.apply_drawing)
        self.close_btn.clicked.connect(lambda: self.set_mode(Modes.DRAG))

        for button in self.top_frame.findChildren(QPushButton):
            button.setIconSize(QSize(24, 24))
            button.setMaximumHeight(30)
            button.setMaximumWidth(30)
            button.setMinimumHeight(30)
            button.setMinimumWidth(30)
        #create frames' labels
        self.loadmap_label = QLabel(self.load_map_frame)
        self.select_label = QLabel(self.selection_frame)
        self.map_label = QLabel(self.map_frame)
        self.keepout_label = QLabel(self.keepout_frame)
        self.save_label = QLabel(self.save_frame)
        self.set_default_label = QLabel(self.set_default_frame)
        self.apply_label = QLabel(self.apply_frame)
        self.close_label = QLabel(self.close_frame)
        self.loadmap_label.setText("Load Map")
        self.select_label.setText("Drag")
        self.map_label.setText("Edit Map")
        self.keepout_label.setText("Keepout")
        self.save_label.setText("Save")
        self.set_default_label.setText("Set Default")
        self.apply_label.setText("Apply")
        self.close_label.setText("Close")
        self.loadmap_label.setAlignment(Qt.AlignCenter)
        self.select_label.setAlignment(Qt.AlignCenter)
        self.map_label.setAlignment(Qt.AlignCenter)
        self.keepout_label.setAlignment(Qt.AlignCenter)
        self.save_label.setAlignment(Qt.AlignCenter)
        self.set_default_label.setAlignment(Qt.AlignCenter)
        self.apply_label.setAlignment(Qt.AlignCenter)
        self.close_label.setAlignment(Qt.AlignCenter)

        #Combobox
        self.maps_combobox = QComboBox(self.load_map_frame)
        self.maps_combobox.addItems(self.load_files())

        #Frames' layout
        loadmap_layout = QGridLayout(self.load_map_frame)
        loadmap_layout.addWidget(self.maps_combobox, 0, 0)
        loadmap_layout.addWidget(self.loadmap_btn, 0, 1)
        loadmap_layout.addWidget(self.loadmap_label, 1, 0)
        self.load_map_frame.setLayout(loadmap_layout)

        selection_layout = QVBoxLayout(self.selection_frame)
        selection_layout.addWidget(self.select_btn)
        selection_layout.addWidget(self.select_label)
        self.selection_frame.setLayout(selection_layout)

        map_layout = QGridLayout(self.map_frame)
        map_layout.addWidget(self.freespace, 0, 0)
        map_layout.addWidget(self.wall_btn, 0, 1)
        map_layout.addWidget(self.map_label, 1, 0)
        self.map_frame.setLayout(map_layout)

        keepout_layout = QVBoxLayout(self.keepout_frame)
        keepout_layout.addWidget(self.keepout)
        keepout_layout.addWidget(self.keepout_label)
        self.keepout_frame.setLayout(keepout_layout)

        save_layout = QVBoxLayout(self.save_frame)
        save_layout.addWidget(self.save_btn)
        save_layout.addWidget(self.save_label)
        self.save_frame.setLayout(save_layout)

        default_layout = QVBoxLayout(self.set_default_frame)
        default_layout.addWidget(self.set_default_btn)
        default_layout.addWidget(self.set_default_label)
        self.set_default_frame.setLayout(default_layout)

        apply_layout = QVBoxLayout(self.apply_frame)
        apply_layout.addWidget(self.apply_btn)
        apply_layout.addWidget(self.apply_label)
        self.apply_frame.setLayout(apply_layout)

        close_layout = QVBoxLayout(self.close_frame)
        close_layout.addWidget(self.close_btn)
        close_layout.addWidget(self.close_label)
        self.close_frame.setLayout(close_layout)

        self.apply_frame.hide()
        self.close_frame.hide()

        self.map_frame.setDisabled(True)
        self.keepout_frame.setDisabled(True)
        self.save_frame.setDisabled(True)
        self.set_default_frame.setDisabled(True)
        self.top_frame.setStyleSheet(self.styles.top_frame())

        return self.top_frame
    
    def load_map(self):
        empty_scene = bool(len(self.map_viewer.scene().items()) > 0)
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle("Exit without saving")
        msg.setText("Unsaved changes will be lost. Are you sure you want to continue?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        if empty_scene:
            retval = msg.exec_()
        if msg.standardButton(msg.clickedButton()) == QMessageBox.Yes or not empty_scene:
            self.map_viewer.load_progress(self.maps_combobox.currentText().split('.')[0])
            self.map_frame.setDisabled(False)
            self.keepout_frame.setDisabled(False)
            self.save_frame.setDisabled(False)
            self.set_default_frame.setDisabled(False)
    
    def save_map(self):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle("Save map")
        msg.setText("Changes will be applied to map. Are you sure you want to save?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        retval = msg.exec_()

        if msg.standardButton(msg.clickedButton()) == QMessageBox.Yes:
            map_name = self.maps_combobox.currentText().split('.')[0]
            self.map_viewer.save_map(map_name)
            saved = QMessageBox()
            saved.setIcon(QMessageBox.Information)
            saved.setWindowTitle("Map Saved")
            saved.setText(f"{map_name} Successfully saved")
            msg.setStandardButtons(QMessageBox.Ok)
            saved_retval = saved.exec_()

    
    def apply_drawing(self):
        self.map_viewer.leave_drawing()
        self.set_mode(Modes.DRAG)
    
    def set_pixel(self, file_name, scale):
        pixel = QPixmap(os.path.join(IMAGES, file_name))
        return pixel.scaled(scale, scale)
    
    def set_mode(self, mode):
        self.map_viewer.set_cursor_mode(mode)
        for button in self.top_frame.findChildren(QPushButton):
            button.setStyleSheet(self.styles.inactive_btn())
        self.sender().setStyleSheet(self.styles.active_btn())
        if mode != Modes.DRAG:
            for frame in self.top_frame.findChildren(QFrame):
                if frame.objectName() != 'apply_frame' and frame.objectName() != 'close_frame':
                    frame.setDisabled(True)
            self.apply_frame.show()
            self.close_frame.show()
        else:
            self.map_viewer.cancel_keepout()
            
            for frame in self.top_frame.findChildren(QFrame):
                frame.setDisabled(False)
            self.apply_frame.hide()
            self.close_frame.hide()
            self.select_btn.setStyleSheet(self.styles.active_btn())
    
    def load_files(self):
        if MAP_PATH:
            files = [f for f in os.listdir(MAP_PATH) if f.endswith(".pgm")]
            return files
        return []
    
    def save_default_map(self):
        map_name = self.maps_combobox.currentText().split('.')[0] + '.yaml'
        keepout_name = self.maps_combobox.currentText().split('.')[0] + '.keepout.yaml'
        default_msg = QMessageBox()
        default_msg.setIcon(QMessageBox.Information)
        default_msg.setWindowTitle("Set as default map")
        default_msg.setText(f"Current map: {map_name} will be set as navigation default map on robot software reboot. Do you wish to proceed?")
        default_msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        retval = default_msg.exec_()

        if default_msg.standardButton(default_msg.clickedButton()) == QMessageBox.Yes:
            file_path = os.path.join(CONFIG, 'default_map.yaml')
            default_map_data = {
                'default_map': map_name,
                'default_keepout': keepout_name
            }
            with open(file_path, 'w') as file:
                yaml.dump(default_map_data, file)

            saved_msg = QMessageBox()
            saved_msg.setIcon(QMessageBox.Information)
            saved_msg.setWindowTitle("Default map saved")
            saved_msg.setText(f"{map_name} successfully set as default map. To see the changes please reboot the robot's software.")
            saved_msg.setStandardButtons(QMessageBox.Ok)
            saved_retval = saved_msg.exec_()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    map_editor = MapEditor()
    map_editor.show()
    sys.exit(app.exec_())