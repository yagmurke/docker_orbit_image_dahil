import math
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPolygonItem
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPolygonF, QBrush, QColor, QPen

class RobotItem:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene
        self.previous_arrow = None
        self.polygon_item = QGraphicsPolygonItem()
        self.polygon_item.setBrush(QBrush(QColor(53, 112, 238)))
        self.polygon_item.setPen(QPen(Qt.NoPen))
        self.polygon_item.setZValue(300)
        self.scene.addItem(self.polygon_item)
    
    def add_robot(self, pose:list):
        pose_x = pose[0]
        pose_y = pose[1]
        yaw = pose[2] + math.pi

        def get_point(x, y):
            xr = (x * math.cos(yaw)) + (y * math.sin(yaw)) + pose_x
            yr = (x * math.sin(yaw)) - (y * math.cos(yaw)) + pose_y
            return xr, yr
        
        scale = 2.5
        x0, x1, x2 = -3*scale, -1*scale, 3*scale
        y0, y1, y2 = 0, 1*scale, 3*scale
        polygon = QPolygonF([
            QPointF(get_point(x0, -y2)[0], get_point(x0, -y2)[1]),
            QPointF(get_point(x2, y0)[0], get_point(x2, y0)[1]),
            QPointF(get_point(x0, y2)[0], get_point(x0, y2)[1]),
            QPointF(get_point(x1, y0)[0], get_point(x1, y0)[1])
        ])
        # self.polygon_item = QGraphicsPolygonItem()
        self.polygon_item.setPolygon(polygon)
        # self.scene.addItem(polygon_item)

        # self.remove_robot()
        # self.previous_arrow = polygon_item
    
    def remove_robot(self):
        if self.previous_arrow is not None:
            self.scene.removeItem(self.previous_arrow)