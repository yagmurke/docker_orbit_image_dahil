import math
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPolygonItem
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPolygonF, QBrush, QColor, QPen

class ArrowItem:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene
        self.previous_arrow = None
    
    def add_arrow(self, pose:list):
        pose_x = pose[0]
        pose_y = pose[1]
        yaw = pose[2]

        def get_point(x, y):
            xr = (x * math.cos(yaw)) + (y * math.sin(yaw)) + pose_x
            yr = (x * math.sin(yaw)) - (y * math.cos(yaw)) + pose_y
            return xr, yr
        
        scale = 1
        x0, x1, x2 = 0, 15*scale, 21*scale
        y0, y1, y2 = 0, 1*scale, 3*scale
        polygon = QPolygonF([
            QPointF(get_point(x0, -y1)[0], get_point(x0, -y1)[1]),
            QPointF(get_point(x1, -y1)[0], get_point(x1, -y1)[1]),
            QPointF(get_point(x1, -y2)[0], get_point(x1, -y2)[1]),
            QPointF(get_point(x2, y0)[0], get_point(x2, y0)[1]),
            QPointF(get_point(x1, y2)[0], get_point(x1, y2)[1]),
            QPointF(get_point(x1, y1)[0], get_point(x1, y1)[1]),
            QPointF(get_point(x0, y1)[0], get_point(x0, y1)[1])
        ])
        polygon_item = QGraphicsPolygonItem()
        polygon_item.setPolygon(polygon)
        polygon_item.setBrush(QBrush(QColor(30, 215, 96)))
        polygon_item.setPen(QPen(Qt.NoPen))
        polygon_item.setZValue(400)
        self.scene.addItem(polygon_item)

        self.remove_arrow()
        self.previous_arrow = polygon_item
    
    def remove_arrow(self):
        if self.previous_arrow is not None:
            self.scene.removeItem(self.previous_arrow)