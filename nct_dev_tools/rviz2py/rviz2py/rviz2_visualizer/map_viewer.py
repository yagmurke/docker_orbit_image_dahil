import os
import sys

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from PyQt5.QtWidgets import (
    QApplication, 
    QGraphicsScene, 
    QGraphicsView,
    QGraphicsPixmapItem
)

from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPainter, QCursor, QPixmap

from rviz2py.rviz2py_threads import Rviz2NodeThread
from rviz2py.rviz2_scene_items import MapItem, CostMapItem, LaserScanVeiw, ArrowItem, RobotItem, PathItem, LocationItem
from rviz2py.utils import MapElementTf, Modes

class MapViewer(QGraphicsView):
    arrow_released = pyqtSignal(bool)
    def __init__(self, parent=None):
        super(MapViewer, self).__init__(parent)

        self.setRenderHint(QPainter.Antialiasing)

        self.zoom_factor = 1.0
        self.map_element_tf = MapElementTf()
        self.mode = Modes.DRAG
        # self.setDragMode(QGraphicsView.ScrollHandDrag)

        self.initial_mouse_pose_x = 0.0
        self.initial_mouse_pose_y = 0.0
        self.final_mouse_pose_x = 0.0
        self.final_mouse_pose_y = 0.0

        self.map_graph_item = QGraphicsPixmapItem()

        self.setScene(QGraphicsScene(self))
        self.robot_item = RobotItem(self.scene())
        self.laser_item = LaserScanVeiw(self.scene())
        self.map_item = MapItem(self.scene())
        self.costmap_item = CostMapItem(self.scene())
        self.arrow_item = ArrowItem(self.scene())
        self.path_item = PathItem(self.scene())
        self.location_item = LocationItem(self.scene())
        self.rviz2_node_thread = Rviz2NodeThread()
        self.rviz2_node_thread.map_msg.connect(self.map_callback)
        self.rviz2_node_thread.costmap_msg.connect(self.costmap_callback)
        self.rviz2_node_thread.transform_msg.connect(self.tf_callback)
        self.rviz2_node_thread.laserscan_msg.connect(self.laser_callback)
        self.rviz2_node_thread.path_msg.connect(self.path_callback)
        self.rviz2_node_thread.start()

        self.tf_dict = {
            self.rviz2_node_thread.base_link_frame: TransformStamped(),
            self.rviz2_node_thread.laser_frame: TransformStamped()
        }

        self.present_map = OccupancyGrid()
    
    def map_callback(self, msg:OccupancyGrid):
        self.map_graph_item = self.map_item.add_map(msg)
        self.present_map = msg
        
    
    def costmap_callback(self, msg:OccupancyGrid):
        self.costmap_item.add_costmap(msg)

    @pyqtSlot(LaserScan)
    def laser_callback(self, scan:LaserScan):
        frame_id = scan.header.frame_id
        scan_tf = self.tf_dict[frame_id]
        scan_stamp = scan_tf.header.stamp.sec
        if scan_stamp > 0:
            tf_pose = self.map_element_tf.m_to_pixel(scan_tf, self.present_map)
            self.laser_item.add_scan(scan, tf_pose)

    @pyqtSlot(str, str, TransformStamped)
    def tf_callback(self, target_frame, fixed_frame, tf):
        self.tf_dict[target_frame] = tf
        print(self.tf_dict)
        if target_frame == self.rviz2_node_thread.base_link_frame:
            robot_tf = tf
            robot_pose = self.map_element_tf.m_to_pixel(robot_tf, self.present_map)
            self.robot_item.add_robot(robot_pose)
    
    def path_callback(self, path):
        self.path_item.draw_path(path, self.present_map)

    
    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.zoom(1.1)
        else:
            self.zoom(0.9)
    def zoom(self, factor):
        self.zoom_factor *= factor
        self.scale(factor, factor)
    
    def mousePressEvent(self, event):
        mouse_position = self.mapToScene(event.pos())
        print(mouse_position)

        if self.map_graph_item.contains(mouse_position):
            if event.button() == Qt.LeftButton:
                self.initial_mouse_pose_x = mouse_position.x()
                self.initial_mouse_pose_y = mouse_position.y()
        
        super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event):
        mouse_position = self.mapToScene(event.pos())

        if self.map_graph_item is not None:
            if self.map_graph_item.contains(mouse_position):
                if self.mode != Modes.DRAG:
                    self.final_mouse_pose_x = mouse_position.x()
                    self.final_mouse_pose_y = mouse_position.y()

                    yaw = self.map_element_tf.yaw_difference(
                        self.final_mouse_pose_y - self.initial_mouse_pose_y,
                        self.final_mouse_pose_x - self.initial_mouse_pose_x
                    )

                    self.arrow_item.add_arrow([
                        self.initial_mouse_pose_x,
                        self.initial_mouse_pose_y,
                        yaw
                    ])

        super().mouseMoveEvent(event)
    
    def mouseReleaseEvent(self, event):
        mouse_position = self.mapToScene(event.pos())

        if self.map_graph_item is not None:
            if self.map_graph_item.contains(mouse_position):
                if self.mode != Modes.DRAG:
                    self.final_mouse_pose_x = mouse_position.x()
                    self.final_mouse_pose_y = mouse_position.y()
                    
                    yaw = self.map_element_tf.yaw_difference(
                        self.final_mouse_pose_y - self.initial_mouse_pose_y,
                        self.final_mouse_pose_x - self.initial_mouse_pose_x
                    )

                    position, orientation = self.map_element_tf.pixel_to_m(
                        [self.initial_mouse_pose_x, self.initial_mouse_pose_y, yaw],
                        self.present_map
                    )

                    if self.mode == Modes.POS:
                        self.rviz2_node_thread.publish_initialpose(position, orientation)
                    else:
                        self.rviz2_node_thread.send_goal(position, orientation)
                    self.arrow_item.remove_arrow()
                    self.change_mode(Modes.DRAG)
                    self.arrow_released.emit(True)
    
    def change_mode(self, mode):
        self.mode = mode
        if self.mode == Modes.DRAG:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.setCursor(QCursor(Qt.ArrowCursor))
        else:
            self.setDragMode(QGraphicsView.NoDrag)
            self.setCursor(QCursor(Qt.CrossCursor))
    
    def set_custom_cursor(self, cursor_path, width, height):
        pixmap = QPixmap(cursor_path)
        scaled_pixmap = pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        custom_cursor = QCursor(scaled_pixmap)
        self.setCursor(custom_cursor)
    
    def add_location(self, tf:TransformStamped, name:str):
        location_pose = self.map_element_tf.m_to_pixel(tf, self.present_map)
        self.location_item.add_location(location_pose)


if __name__ == "__main__":
    import rclpy
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    mapviewer = MapViewer()
    mapviewer.show()
    sys.exit(app.exec_())