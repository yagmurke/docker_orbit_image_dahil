from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsScene
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QBrush, QColor, QPen

from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid

class PathItem:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene
        self.previous_point_items = []
    def draw_path(self, path:Path, map_msg:OccupancyGrid):
        map_origin_x = map_msg.info.origin.position.x
        map_origin_y = map_msg.info.origin.position.y
        map_height = map_msg.info.height
        map_resolution = map_msg.info.resolution

        if len(self.previous_point_items) > 0:
            for i in self.previous_point_items:
                self.scene.removeItem(i)
            self.previous_point_items = []
    
        for i in path.poses:
            x = (i.pose.position.x - map_origin_x) / map_resolution
            y = ((map_origin_y - i.pose.position.y) / map_resolution) + map_height
            point = QPointF(x, y)
            point_item = QGraphicsEllipseItem(point.x() -0.5, point.y() - 0.5, 1.0, 1.0)
            point_item.setBrush(QBrush(QColor(181,145,29)))
            point_item.setPen(QPen(Qt.NoPen))
            point_item.setZValue(250)
            self.scene.addItem(point_item)
            self.previous_point_items.append(point_item)