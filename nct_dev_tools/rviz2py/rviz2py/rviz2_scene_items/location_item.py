# from PyQt5 import QtWidgets, QtCore, QtGui

# class DrawImage(QtWidgets.QWidget):    
#     def __init__(self):
#         super(DrawImage, self).__init__()

#         self.move(150,50)
#         # self.setFixedSize(100,100)   
#         self.startA    = 5
#         self.endA      = 30
#         self.linewidth = 1

#     def paintEvent(self, event):
#         paint = QtGui.QPainter()
#         path = QtGui.QPainterPath()
#         paint.begin(self)
#         paint.setRenderHint(QtGui.QPainter.Antialiasing) 

#         paint.setPen(QtCore.Qt.red)
#         paint.setBrush(QtCore.Qt.red)

#         scale = 0.5
#         x1, x2, x3, = 30*scale, 50 *scale, 70*scale
#         y1, y2, y3 = 0*scale, 20 *scale, 100*scale 
#         path.moveTo(x2, y3)
#         path.lineTo(x3, y2)
#         path.cubicTo(x3, y2, x2, y1, x1, y2)
#         path.lineTo(x2, y3)

#         paint.drawPath(path)

#         paint.end()    

# if __name__ == '__main__':
#     import sys
#     app = QtWidgets.QApplication(sys.argv)
#     w = DrawImage()
#     w.show()
#     sys.exit(app.exec_())

import math
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPolygonItem, QGraphicsPathItem
from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import QPolygonF, QBrush, QColor, QPen, QPainterPath

class LocationItem:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene
        self.previous_location = None
    
    def add_location(self, pose:list):
        pose_x = pose[0]
        pose_y = pose[1]
        path = QPainterPath()

        scale = 0.1
        x1, x2, x3, = -30*scale + pose_x, 0 *scale + pose_x, 30*scale + pose_x
        y1, y2, y3 = -70*scale + pose_y, 0 *scale + pose_y, 50*scale + pose_y
        path.moveTo(x2, y3)
        path.lineTo(x3, y2)
        # path.cubicTo(x3, y2, x2, y1, x1, y2)
        path.arcTo(QRectF(x3, y2), 0, 180)
        path.cubicTo()
        path.lineTo(x2, y3)

        path_item = QGraphicsPathItem()
        path_item.setPen(QPen(Qt.NoPen))
        path_item.setBrush(QBrush(Qt.red))
        path_item.setPath(path)

        self.scene.addItem(path_item)
        return path_item
    
    def remove_location(self, item):
        self.scene.removeItem(item)