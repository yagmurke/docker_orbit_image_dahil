import sys
import os
import numpy as np
import math
import rclpy
import cv2

from enum import Enum

from scipy.spatial.transform import Rotation as R

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem
from PyQt5.QtGui import QPixmap, QImage, QTransform, QCursor, QBrush, QColor, QPen, QPolygonF, QKeySequence, QPainter
from PyQt5.QtCore import Qt, QPointF, QRectF

from rviz2py.connectors import MapListener, Rviz2Publisher
from rviz2py.connectors import LaserToImage


IMAGES = os.path.join(get_package_share_directory('rviz2py'), 'resource', 'images')

class Modes(Enum):
    DRAG = 1
    POS = 2
    GOAL = 3
    KEEPOUT = 4

class MapView(QGraphicsView):
    def __init__(self):
        super().__init__()
        # Set up the scene
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setStyleSheet("background-color: grey")

        self.init_map()
        self.rviz2_publisher = Rviz2Publisher()

        self.laser_to_image = LaserToImage()

        # Enable drag mode
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        # Set initial zoom factor
        self.zoom_factor = 1.0

        robot_icon = QImage(f"{IMAGES}/icon.png")
        self.robot_element = MapElement(robot_icon, self.scene)
        self.pose_element = MapElement(robot_icon, self.scene)
        self.laser_element = MapElement(None, self.scene)

        self.keepout = KeepOut(self.scene)

        self.previous_map = None
        self.previous_costmap = None
        self.current_x = 0
        self.current_y = 0
        self.current_angle = 0
        self.previous_point_items = []

        self.right_button_pressed = False
        self.last_mouse_pos = None

        self.icon_path = ''

        # dict of array where 1st index is mouse cursor png name 2nd index is arrow png name
        self.modes = {
            Modes.POS: ['location', 'arrowm'],
            Modes.GOAL: ['icon', 'send_posem']
        }

        self.mode = Modes.DRAG

    def init_map(self):
        self.map_listener = MapListener()
        self.map_listener.map_data.connect(self.map_callback)
        self.map_listener.tf_message.connect(self.tf_callback)
        self.map_listener.laser_scan.connect(self.laser_callback)
        self.map_listener.path_msg.connect(self.path_callback)
        self.map_listener.costmap_data.connect(self.costmap_callback)
        self.map_listener.start()
    
    def laser_callback(self, data):
        laser = self.laser_to_image.scan_to_image(data)
        width, height, c = laser.shape
        laser_image = QImage(laser.data, width, height, c*width, QImage.Format_RGBA8888)
        self.laser_element.element = laser_image
        self.laser_element.draw_element(self.current_x, self.current_y, self.current_angle, False)
    
    
    def path_callback(self, path_msg:Path):
        self.draw_path(path_msg.poses)
    
    def draw_path(self, path):
        if len(self.previous_point_items) > 0:
            for i in self.previous_point_items:
                self.scene.removeItem(i)
            self.previous_point_items = []
        for i in path:
            x = (i.pose.position.x - self.map_orgin[0]) / self.map_resolution
            y = ((self.map_orgin[1] - i.pose.position.y) / self.map_resolution) + self.map_width
            point = QPointF(x, y)
            point_item = QGraphicsEllipseItem(point.x() -1, point.y() - 1, 2, 2)
            point_item.setBrush(QBrush(QColor(0,255,0)))
            point_item.setPen(QPen(Qt.NoPen))
            self.scene.addItem(point_item)
            self.previous_point_items.append(point_item)
    
    def costmap_callback(self, data:OccupancyGrid):
        costmap_item = QGraphicsPixmapItem(self.get_costmap(data))
        self.scene.addItem(costmap_item)
        if self.previous_costmap is not None:
            self.scene.removeItem(self.previous_costmap)
        self.previous_costmap = costmap_item

    def map_callback(self, map_data:OccupancyGrid):
        self.map_data = map_data
        map_pixmap = self.get_map()
        self.map_item = QGraphicsPixmapItem(map_pixmap)
        self.map_item.setZValue(-1)
        self.scene.addItem(self.map_item)
        if self.previous_map is not None:
            self.scene.removeItem(self.previous_map)
        self.previous_map = self.map_item
    
    def tf_callback(self, tf_message:PoseStamped):
        x_m = tf_message.pose.position.x
        y_m = tf_message.pose.position.y
        qx = tf_message.pose.orientation.x
        qy = tf_message.pose.orientation.y
        qw = tf_message.pose.orientation.w
        qz = tf_message.pose.orientation.z

        r = R.from_quat([qx, qy, qw, qz])
        angles = r.as_euler('zyx', degrees=True)
        self.current_angle = angles[0] + 180

        self.current_x = (x_m - self.map_orgin[0]) / self.map_resolution
        self.current_y = ((self.map_orgin[1] - y_m) / self.map_resolution) + self.map_width
        self.robot_element.draw_element(self.current_x, self.current_y, self.current_angle, True)

    def get_map(self):
        map_datas = self.map_data.data
        self.map_height = self.map_data.info.width
        self.map_width = self.map_data.info.height
        self.map_resolution = self.map_data.info.resolution
        self.map_orgin = [self.map_data.info.origin.position.x, self.map_data.info.origin.position.y]
        map_array = np.array(map_datas)
        map_array = np.where(map_array == 0, 255, 0)
        map_array = np.array(map_array, dtype=np.int8).reshape((self.map_width, self.map_height))
        if map_array is not None:
            height, width = map_array.shape
            qimage = QImage(map_array.data, width, height, width, QImage.Format_Grayscale8)
            qimage = qimage.mirrored(horizontal=False, vertical=True)
            self.map_image = qimage
            self.original_image = qimage
            pixmap = QPixmap.fromImage(qimage)
            return pixmap
    
    def get_costmap(self, map_data:OccupancyGrid):
        cost_data = map_data.data
        height = map_data.info.width
        width = map_data.info.height
        resoulution = map_data.info.resolution
        orgin = [map_data.info.origin.position.x, map_data.info.origin.position.y]
        map_array = np.array(cost_data)
        map_array = np.array(map_array, dtype=np.int8).reshape((width, height))
        if map_array is not None:
            pallete = self.costmap_pallete()
            new_image = pallete[map_array]
            height, width, c = new_image.shape
            rgb_image = QImage(new_image.data, width, height, c*width, QImage.Format_RGBA8888)
            cost_image = rgb_image.mirrored(horizontal=False, vertical=True)
            pixmap = QPixmap.fromImage(cost_image)
            return pixmap
    
    def costmap_pallete(self):
        color_mapping = []
        for i in range(99):
            v = (i * 255) / 100
            color_mapping.append([i, v, 0, 255 - v, 85])
        color_mapping.append([0, 0, 0, 0, 0])
        color_mapping.append([99, 0, 255, 255, 85])
        color_mapping.append([100, 255, 0, 255, 85])
        color_table = np.zeros((101, 4), dtype=np.uint8)
        for entry in color_mapping:
            gray_value = entry[0]
            rgba = entry[1:]
            color_table[gray_value] = rgba
        return color_table
    
    def apply_colormap(self, image, colormap:cv2.COLORMAP_JET):
        if image.shape[2] != 4:
            raise ValueError("Input image must have 4 channels")
        
        bgr = image[:, :, :3]
        alpha = image[:, :, 3]

        colored_image = cv2.applyColorMap(bgr, colormap)

        mask = alpha > 0
        colored_image[~mask] = 0

        colored_image_with_alpha = cv2.cvtColor(colored_image, cv2.COLOR_RGB2BGRA)
        colored_image_with_alpha[:, :, 3] = alpha
        return colored_image_with_alpha
    
    def mousePressEvent(self, event):
        mouse_position = self.mapToScene(event.pos())
        if self.map_item.contains(mouse_position):
            if event.button() == Qt.RightButton or event.button() == Qt.MiddleButton:
                self.right_button_pressed = True
                self.last_mouse_pos = event.pos()
                self.setDragMode(QGraphicsView.NoDrag)
                
            elif event.button() == Qt.LeftButton:
                self.initial_mouse_pos_x = mouse_position.x()
                self.initial_mouse_pos_y = mouse_position.y()
                if self.mode == Modes.DRAG:
                    
                    self.setCursor(QCursor(Qt.ArrowCursor))
                elif self.mode == Modes.POS or self.mode == Modes.GOAL:
                    self.setDragMode(QGraphicsView.NoDrag)
                elif self.mode == Modes.KEEPOUT:
                    self.setDragMode(QGraphicsView.NoDrag)
                    self.keepout.update_points(mouse_position)
                    self.keepout.clear_queue()
                    self.keepout.update_line()
                    self.setMouseTracking(True)

        super().mousePressEvent(event)
    
    def keyPressEvent(self, event):
        if QKeySequence(event.modifiers() | event.key()) == QKeySequence.Undo:
            self.keepout.undo()
        
        elif QKeySequence(event.modifiers() | event.key()) == QKeySequence.Redo:
            self.keepout.redo()
        
        elif QKeySequence(event.modifiers() | event.key()) == QKeySequence.Cancel:
            self.setMouseTracking(False)
            self.keepout.cancel_keepout()
    
    def mouseMoveEvent(self, event):
        mouse_position = self.mapToScene(event.pos())

        if self.right_button_pressed and self.last_mouse_pos is not None:
            delta = event.pos() - self.last_mouse_pos
            angle = delta.x()  # Change the rotation angle based on horizontal mouse movement
            self.rotate(angle / 10)
            self.last_mouse_pos = event.pos()
        else:
            if self.map_item.contains(mouse_position):
                if self.mode == Modes.POS or self.mode == Modes.GOAL:
                    self.final_mouse_pos_x = mouse_position.x()
                    self.final_mouse_pos_y = mouse_position.y()
                    angle = self.calculate_arctan(
                        self.final_mouse_pos_y - self.initial_mouse_pos_y,
                        self.final_mouse_pos_x - self.initial_mouse_pos_x
                    )
                    img = QImage(self.icon_path)
                    self.pose_element.element = img
                    self.pose_element.draw_element(self.initial_mouse_pos_x, self.initial_mouse_pos_y, angle, True)
                elif self.mode == Modes.KEEPOUT:
                    if len(self.keepout.points) > 0:
                        self.keepout.draw_line(mouse_position)
        
        super().mouseMoveEvent(event)
    
    def mouseReleaseEvent(self, event):
        self.right_button_pressed = False
        mouse_position = self.mapToScene(event.pos())
        if self.mode != Modes.KEEPOUT:
            if self.map_item.contains(mouse_position):
                self.pose_element.delete_element()
                self.final_mouse_pos_x = mouse_position.x()
                self.final_mouse_pos_y = mouse_position.y()
                self.setDragMode(QGraphicsView.ScrollHandDrag)
                self.setCursor(QCursor(Qt.ArrowCursor))
                x = self.initial_mouse_pos_x * self.map_resolution + self.map_orgin[0]
                y = (self.map_width - self.initial_mouse_pos_y) * self.map_resolution + self.map_orgin[1]
                angle = self.calculate_arctan(
                self.final_mouse_pos_y - self.initial_mouse_pos_y,
                self.final_mouse_pos_x - self.initial_mouse_pos_x
                )
                q = R.from_euler('zyx', [angle - 180, 0, 0], degrees=True)
                orientation = q.as_quat().tolist()
                if self.mode == Modes.POS:
                    self.rviz2_publisher.publish_initialpose([x, y, 0.0], orientation)
                elif self.mode == Modes.GOAL:
                    self.rviz2_publisher.send_goal([x, y, 0.0], orientation)
                self.set_cursor_mode(Modes.DRAG)
 
    def calculate_arctan(self, y, x):
        angle_radians = math.atan2(y, x)
        angle_degrees = math.degrees(angle_radians)
        if angle_degrees < 0:
            angle_degrees += 360
        
        return angle_degrees
        
    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.zoom(1.1)
        else:
            self.zoom(0.9)

    def zoom(self, factor):
        self.zoom_factor *= factor
        self.scale(factor, factor)
    
    def set_custom_cursor(self, cursor_path, width, height):
        pixmap = QPixmap(cursor_path)
        scaled_pixmap = pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        custom_cursor = QCursor(scaled_pixmap)
        self.setCursor(custom_cursor)
    
    def set_cursor_mode(self, mode):
        self.setDragMode(QGraphicsView.NoDrag)
        self.mode = mode
        self.setMouseTracking(False)
        if mode in self.modes:
            self.set_custom_cursor(f"{IMAGES}/{self.modes[mode][0]}.png", 20, 20)
            self.icon_path = f"{IMAGES}/{self.modes[mode][1]}.png"
        else:
            self.setCursor(QCursor(Qt.ArrowCursor))
    
    def save_keepout(self, file_name):
        self.keepout.save_pgm(file_name, self.get_map())

class MapElement:
    def __init__(self, element:QImage, scene:QGraphicsScene):
        self.element = element
        self.scene = scene

        self.previous_element = None
        self.robot_item = None
    
    def draw_element(self, x, y, angle, scale:bool):
        robot_icon = self.rotate_image(self.element, angle)
        pixmap = QPixmap(robot_icon)
        if scale:
            robot_image = pixmap.scaled(50, 50, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        else:
            robot_image = pixmap
        self.robot_item = QGraphicsPixmapItem(robot_image)
        self.robot_item.setPos(x - robot_image.width() / 2, y - robot_image.height() / 2)
        self.scene.addItem(self.robot_item)
        if self.previous_element is not None:
            self.scene.removeItem(self.previous_element)
        self.previous_element = self.robot_item
    
    def rotate_image(self, image, angle):
        if image is not None:
            transform = QTransform().rotate(angle)
            rotated_image = image.transformed(transform, Qt.SmoothTransformation)
            return rotated_image
    
    def delete_element(self):
        if self.robot_item is not None:
            self.scene.removeItem(self.robot_item)

class KeepOut:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene

        self.points = []
        self.polygon_items = []
        self.polygons = []

        self.line_item = None
        self.line_count = 1
        self.line_items = []
        self.sequence_queue = []

        self.painting_color = Qt.black
    
    def update_points(self, point):
        self.points.append(point)
    
    def clear_queue(self):
        self.sequence_queue = []
    
    def draw_line(self, current_point):
        if self.line_item:
            self.scene.removeItem(self.line_item)
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
        self.line_item = self.scene.addLine(x, y, width, height, pen=QPen(self.painting_color))
    
    def update_line(self):
        if len(self.points) > 1:
            x0 = self.points[0].x()
            y0 = self.points[0].y()
            x = self.points[self.line_count - 1].x()
            y = self.points[self.line_count - 1].y()
            x2 = self.points[self.line_count].x()
            y2 = self.points[self.line_count].y()
            s = math.sqrt((x2 - x0)** 2 + (y2 - y0) ** 2)
            if s < 10:
                self.points[self.line_count] = self.points[0]
                self.draw_polygon()
                self.scene.removeItem(self.line_item)
                for i in self.line_items:
                    self.scene.removeItem(i)
                self.line_count = 1
                self.points = []
                self.line_items = []
            else:
                self.line_count += 1
                self.line_items.append(self.scene.addLine(x, y, x2, y2, pen=QPen(self.painting_color)))
    
    def draw_polygon(self):
        if len(self.points) > 1:
            self.polygon = QPolygonF(self.points)
            pol_item = self.scene.addPolygon(self.polygon, brush=QBrush(self.painting_color))
            pol_item.setPen(QPen(Qt.NoPen))
            self.polygon_items.append(pol_item)
            self.polygons.append(self.polygon)
    
    def undo(self):
        if len(self.points) > 1:
            self.line_count = self.line_count - 1
            self.scene.removeItem(self.line_items[self.line_count-1])
            self.sequence_queue.append(self.points[-1])
            self.points.pop()
            self.line_items.pop()
        
        elif len(self.polygon_items) > 0:
            self.scene.removeItem(self.polygon_items[-1])
            self.polygon_items.pop()
            self.polygons.pop()
    
    def redo(self):
        if len(self.sequence_queue) > 0:
            self.points.append(self.sequence_queue[-1])
            self.update_line()
            self.sequence_queue.pop()
    
    def cancel_keepout(self):
        if self.line_item:
            self.scene.removeItem(self.line_item)
        if len(self.line_items) > 0:
            for i in self.line_items:
                self.scene.removeItem(i)
        self.line_items = []
        self.points = []
        self.line_count = 1
    
    def save_pgm(self, file_name, map):
        scene = QGraphicsScene()
        map_copy = QGraphicsPixmapItem(map)
        scene.addItem(map_copy)
        for i in self.polygons:
            keepout = scene.addPolygon(i, brush=QBrush(self.painting_color))
        converter = GraphicsItemsToPixmap(scene, [map_copy])
        converter.save_pixmap_as_pgm(file_name)

class GraphicsItemsToPixmap:
    def __init__(self, scene, items):
        self.scene = scene
        self.items = items
        self.bounding_rect = self.calculate_bounding_rect()

    def calculate_bounding_rect(self):
        if not self.items:
            return QRectF(0, 0, 0, 0)
        # Combine bounding rects of all items
        rect = self.items[0].sceneBoundingRect()
        for item in self.items[1:]:
            rect = rect.united(item.sceneBoundingRect())
        return rect

    def render_to_pixmap(self):
        # Ensure the bounding rect has a valid size
        if self.bounding_rect.isEmpty():
            return QPixmap()

        # Create a QPixmap based on the bounding rectangle size
        pixmap = QPixmap(int(self.bounding_rect.width()), int(self.bounding_rect.height()))
        pixmap.fill(Qt.transparent)  # Optional: Fill with transparent background

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # Adjust the scene's view to the bounding rectangle
        self.scene.render(painter, QRectF(pixmap.rect()), self.bounding_rect)

        painter.end()
        return pixmap

    def save_pixmap_as_pgm(self, filename):
        pixmap = self.render_to_pixmap()
        if not pixmap.isNull():
            # Convert the QPixmap to QImage
            image = pixmap.toImage().convertToFormat(QImage.Format_Grayscale8)

            # Save the QImage as a PGM file
            image.save(filename, "PGM")
        else:
            print("Failed to render pixmap; the bounding rectangle may be empty.")


if __name__ == "__main__":
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = MapView()
    window.setWindowTitle('Map Viewer')
    window.show()
    sys.exit(app.exec_())
