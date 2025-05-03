from PyQt5.QtWidgets import (
    QGraphicsPolygonItem, QMenu, QAction
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPen, QBrush, QColor

class HovablePolygon(QGraphicsPolygonItem):
    """ Custon QPolygon that reacts to hover and right-click event """
    def __init__(self, polygon, *args):
        super().__init__(*args)
        self.setPolygon(polygon)
        self.setAcceptHoverEvents(True)
        self.orginal_pen = QPen(Qt.NoPen)
        self.highlight_pen = QPen(QColor("yellow"), 2)
        self.setPen(self.orginal_pen)
    
    def hoverEnterEvent(self, event):
        self.setPen(self.highlight_pen)
        super().hoverEnterEvent(event)
    
    def hoverLeaveEvent(self, event):
        self.setPen(self.orginal_pen)
        super().hoverLeaveEvent(event)
    
    def mousePressEvent(self, event):
        """ Handle right-click event """
        if event.button() == Qt.RightButton:
            if self.acceptHoverEvents():
                self.show_context_menu(event.screenPos())
        else:
            super().mousePressEvent(event)
    
    # def mouseReleaseEvent(self, event):
    #     super().hoverLeaveEvent(event)
    
    def show_context_menu(self, pos):
        """ Show a context menu on right-click """
        menu = QMenu()
        delete_action = QAction("Delete", menu)
        delete_action.triggered.connect(self.delete_item)
        menu.addAction(delete_action)
        menu.exec_(pos)
    
    def delete_item(self):
        """  Delete this item """
        self.scene().removeItem(self)