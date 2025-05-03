import os
class NavigatorStyles:
    def __init__(self):
        self.primary_color = "#202020"
        self.secondary_color = "#272727"
        self.text_color = "#999c9e"
        self.active_btn_color = "#343434"
        self.active_stroke_color = "#4d4d4d"

    def container(self, container):
        return f"""
                QWidget#{container} {{
                    background-color: {self.secondary_color};
                }}
                QGraphicsView {{
                    background-color: {self.secondary_color};
                    border: 0px solid black;
                }}
                QLabel {{
                    color: {self.text_color};
                    font: 81 10pt "Mulish";
                }}

        """
    def top_frame(self):
        return f"""
                QFrame#top_frame {{
                    background-color: {self.secondary_color};
                    border-right: 0px;
                    border-left: 0px;
                    border-top: 0px;
                    border-bottom: 1px solid #4e5051;
                }}
                QPushButton {{
                    border: 0px;
                    background-color: transparent;
                }}
                QPushButton:hover {{
                    background-color: {self.active_btn_color};
                    border: 1px solid {self.active_stroke_color};
                    border-radius: 4px;
                }}

        """
    def left_frame(self):
        return f"""
                QFrame#left_frame {{
                    background-color: {self.primary_color};
                    border-right: 1px solid #4e5051;
                    border-left: 0px;
                    border-top: 0px;
                    border-bottom: 0px;
                }}
                QPushButton {{
                    border: 0px;
                    background-color: transparent;
                    color: white;
                    font-size: 10pt;
                    text-align: left;
                    padding: 0px 0px 0px 10px;
                }}
                QPushButton:hover {{
                    background-color: {self.active_btn_color}

                }}
        """
    
    def active_btn(self):
        return f"""
                QPushButton {{
                    background-color: {self.active_btn_color};
                    border: 1px solid {self.active_stroke_color};
                    border-radius: 4px;
                }}
        """
    
    def inactive_btn(self):
        return f"""
                QPushButton {{
                    border: 0px;
                    background-color: transparent;
                }}
                QPushButton:hover {{
                    background-color: {self.active_btn_color};
                    border: 1px solid {self.active_stroke_color};
                    border-radius: 4px;
                }}
        """
    def menu_frame(self, object_name):
        return f"""
                QFrame#{object_name} {{
                    border-right: 1px solid #4e5051;
                    border-left: 0px;
                    border-top: 0px;
                    border-bottom: 0px;
                }}
        """