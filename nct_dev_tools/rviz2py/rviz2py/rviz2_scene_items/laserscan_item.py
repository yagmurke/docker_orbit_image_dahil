import os
import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsRectItem, QGraphicsPixmapItem
from PyQt5.QtGui import QBrush, QColor, QPen, QImage, QPixmap
from PyQt5.QtCore import Qt, QRectF

class LaserScanVeiw:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene
        self.previous_items = []
        self.previous_map = None
        self.resolution = 0.05
        self.laser_size = 1.5
        self.laser_item = QGraphicsPixmapItem()
        self.scene.addItem(self.laser_item)
    
    def add_scan(self, scan:LaserScan, pose:list):
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        pose_x = pose[0]
        pose_y = pose[1]
        yaw = pose[2]
        num_pts = len(ranges)
        image_size = int(scan.range_max / self.resolution)

        blank_image = np.zeros((image_size, image_size, 4), dtype=np.uint8)

        for i in range(num_pts):
            if not isinstance(ranges[i], str) and ranges[i] != float('inf'):
                angle = angle_min + float(i) * angle_increment
                x = float(ranges[i] * math.cos(angle)) / self.resolution
                y = float(ranges[i] * math.sin(angle)) / self.resolution

                theta = yaw + math.pi
                xr = int((x * math.cos(theta)) + (y * math.sin(theta)) + image_size / 2)
                yr = int((x * math.sin(theta)) - (y * math.cos(theta)) + image_size / 2)

                if xr > 0 and yr> 0:
                    blank_image[yr, xr] = [255, 0, 0, 255]

        heigt, width, c = blank_image.shape
        rgba_image = QImage(blank_image.data, width, heigt, c*width, QImage.Format_RGBA8888)
        pixmap = QPixmap.fromImage(rgba_image)
        self.laser_item.setPixmap(pixmap)
        self.laser_item.setZValue(200)
        self.laser_item.setPos(pose_x - image_size/2, pose_y - image_size / 2)
        

        # if self.previous_map is not None:
        #     self.scene.removeItem(self.previous_map)
        # self.previous_map = self.laser_item 
