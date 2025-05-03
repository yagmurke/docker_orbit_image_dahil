import numpy as np
from nav_msgs.msg import OccupancyGrid
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtGui import QImage, QPixmap

class MapItem:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene
        self.previous_map = None
        self.map_item = QGraphicsPixmapItem()
        self.scene.addItem(self.map_item)
    
    def add_map(self, map:OccupancyGrid):
        map_data = map.data
        map_width = map.info.width
        map_height = map.info.height
        map_resolution = map.info.resolution
        map_array = np.array(map_data)
        map_array = np.where(map_array == 0, 255, map_array)
        map_array = np.where(map_array == 100, 0, map_array)
        map_array = np.array(map_array, dtype=np.int8).reshape((map_height, map_width))
        
        if map_array is not None:
            height, width = map_array.shape
            qimage = QImage(map_array.data, width, height, width, QImage.Format_Grayscale8)
            qimage = qimage.mirrored(horizontal=False, vertical=True)
            pixmap = QPixmap.fromImage(qimage)
            
            self.map_item.setPixmap(pixmap)
            self.map_item.setZValue(0)
            return self.map_item

class CostMapItem:
    def __init__(self, scene:QGraphicsScene):
        self.scene = scene
        self.previous_map = None
        self.costmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.costmap_item) 
    
    def add_costmap(self, costmap:OccupancyGrid):
        cost_data = costmap.data
        costmap_height = costmap.info.height
        costmap_width = costmap.info.width
        map_array = np.array(cost_data)
        map_array = np.array(map_array, dtype=np.int8).reshape((costmap_height, costmap_width))
        if map_array is not None:
            pallete = self.costmap_pallete()
            new_image = pallete[map_array]
            height, width, c = new_image.shape
            rgb_image = QImage(new_image.data, width, height, c*width, QImage.Format_RGBA8888)
            rgb_image = rgb_image.mirrored(horizontal=False, vertical=True)
            pixmap = QPixmap.fromImage(rgb_image) 
            self.costmap_item.setPixmap(pixmap)

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
