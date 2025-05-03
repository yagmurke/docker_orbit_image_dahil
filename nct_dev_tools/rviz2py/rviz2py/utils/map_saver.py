import pickle
import numpy as np

from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsLineItem, QGraphicsPolygonItem
from PyQt5.QtGui import QPixmap, QImage, QTransform, QCursor, QPainter, QColor, QBrush, QPen, QPolygonF, QKeySequence
from PyQt5.QtCore import Qt, QPointF, QLineF, QLine, QPoint, QRectF

from rviz2py.rviz2_scene_items import HovablePolygon

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

class SceneSaver:
    def __init__(self):
        pass
    
    def save_progress(self, scene:QGraphicsScene, filename="scene_data.nct"):
        """ Save the items in the scene to a file"""
        scene_items = []
        for item in scene.items():
            if isinstance(item, QGraphicsPolygonItem):
                points = []
                for point in range(item.polygon().size()):
                    points.append([item.polygon().value(point).x(), item.polygon().value(point).y()])
                
                color_name = item.brush().color().name()
                if color_name != "#ffffff":
                    scene_items.append(('polygon', points, color_name))
        
        with open(filename, 'wb') as file:
            pickle.dump(scene_items, file)
    
    def load_progress(self, filename="scene_data.nct"):
        """ Load the items from the file into the scene """
        try:
            polygon_items = []
            with open(filename, 'rb') as file:
                scene_items = pickle.load(file)

            for item_type, *item_data in scene_items:
                if item_type == 'polygon':
                    points = []
                    rect, color = item_data
                    for point in rect:
                        points.append(QPointF(point[0], point[1]))
                    polygon = QPolygonF(points)
                    polygon_item = HovablePolygon(polygon)
                    color_a = QColor(color)
                    color_a.setAlpha(80)
                    polygon_item.setBrush(QBrush(color_a))
                    polygon_items.append(polygon_item)
            return polygon_items

        except (FileNotFoundError, EOFError):
            print("No saved file found, starting fresh.")
        
    def load_map(self, filename="map1.pgm"):
        with open(filename, 'rb') as f:
            magic_number = f.readline().decode('ascii')
            if magic_number.strip() != 'P5':
                raise ValueError('File is not a binary PGM (P5) file')
            
            while True:
                line = f.readline().decode('ascii')
                if line.startswith('#'):
                    continue
                else:
                    width, height = map(int, line.split())
                    break
            max_gray = int(f.readline().decode('ascii'))
            pixel_data = list(f.read())
        
        map_array = np.array(pixel_data)
        map_array = np.array(map_array, dtype=np.int8).reshape((height, width))
        if map_array is not None:
            qimage = QImage(map_array.data, width, height, width, QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(qimage)
            map_item = QGraphicsPixmapItem(pixmap)
            return map_item
    
    def save_map(self, scene:QGraphicsScene, map_path, keepout_path, progress_path):
        map_item = self.load_map(map_path)
        map_scene = QGraphicsScene()
        keepout_scene = QGraphicsScene()
        map_scene.addItem(QGraphicsPixmapItem(map_item.pixmap()))
        keepout_scene.addItem(QGraphicsPixmapItem(map_item.pixmap()))
        for item in scene.items():
            if isinstance(item, QGraphicsPolygonItem):
                if item.brush().color().name() != '#ffffff':
                    keep_item = QGraphicsPolygonItem(item.polygon())
                    keep_item.setBrush(QBrush(QColor('#000000')))
                    keepout_scene.addItem(keep_item)
                else:
                    free_item = QGraphicsPolygonItem(item.polygon())
                    free_item.setPen(QPen(Qt.NoPen))
                    free_item.setBrush(QBrush(QColor('#ffffff')))
                    keepout_item = QGraphicsPolygonItem(item.polygon())
                    keepout_item.setPen(QPen(Qt.NoPen))
                    keepout_item.setBrush(QBrush(QColor('#ffffff')))
                    map_scene.addItem(free_item)
                    keepout_scene.addItem(keepout_item)
            
            elif isinstance(item, QGraphicsLineItem):
                map_line = QGraphicsLineItem(item.line())
                map_line.setPen(QPen(Qt.black, 3))
                keep_line = QGraphicsLineItem(item.line())
                keep_line.setPen(QPen(Qt.black, 3))
                map_scene.addItem(map_line)
                keepout_scene.addItem(keep_line)

        map_converter = GraphicsItemsToPixmap(map_scene, [map_item])
        keepout_converter = GraphicsItemsToPixmap(keepout_scene, [map_item])
        map_converter.save_pixmap_as_pgm(map_path)
        keepout_converter.save_pixmap_as_pgm(keepout_path)
        self.save_progress(scene, progress_path)