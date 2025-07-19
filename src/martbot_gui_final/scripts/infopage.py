from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QStackedWidget, QGridLayout, QPushButton,QDialog,
    QWidget,QMessageBox, QToolButton, QLabel, QSizePolicy, QSpacerItem, QHBoxLayout, QLineEdit,QTableWidget,QTableWidgetItem, QFrame, QHeaderView 
)
from PyQt5.QtGui import QIcon, QFont, QMovie,QPixmap,QColor
from PyQt5.QtCore import QSize, Qt,QUrl
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import homepage,led_try
import sqlite3




class infowindow(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.switch_page_callback = switch_page_callback
        self.controller=led_try.LEDController()
        self.init_ui()
        

    def init_ui(self):
        layout = QVBoxLayout()

        # Title
        title_label = QLabel("product information", self)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 60, QFont.Bold))
        title_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)  # Expanding horizontally
        layout.addWidget(title_label)

        # Spacer
        layout.addSpacerItem(QSpacerItem(10, 100, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Grid Layout for Icons
        icon_grid_layout = QGridLayout()
        icon_grid_layout.setVerticalSpacing(50)  # Add more space between rows
        self.icons = ["/home/martbot/martbot_ws/src/martbot_gui/scripts/images/bread.png", "/home/martbot/martbot_ws/src/martbot_gui/scripts/images/cleaning_items2-removebg-preview (1) (1).png", "/home/martbot/martbot_ws/src/martbot_gui/scripts/images/cofee_tea-removebg-preview (1).png","/home/martbot/martbot_ws/src/martbot_gui/scripts/images/dairy.png","/home/martbot/martbot_ws/src/martbot_gui/scripts/images/drinks-removebg-preview.png","/home/martbot/martbot_ws/src/martbot_gui/scripts/images/snacks.png"]
        self.names = ["Bakery", "Cleaning items", "Coffee-Tea","Dairy","Drinks","Snacks"]
        
        for row in range(6):  # Example: 1 row
              # Example: 3 icons in a row
                button = QToolButton(self)
                button.setIcon(QIcon(self.icons[row]))
                button.setIconSize(QSize(150, 150))  # Icon size
                button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # Expand button
                button.setFixedSize(300, 300)  # Button size
                button.setStyleSheet(
                    """
                    QToolButton {
                        border: none;
                        background-color: #ADD8E6;
                        border-radius: 10px;
                    }
                    QToolButton:hover {
                        background-color: #87CEEB;
                    }
                    """
                )
                button.setText(self.names[row])
                button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
                button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
                button.clicked.connect(lambda checked, idx=row+5: self.switch_page_callback(idx))
                icon_grid_layout.addWidget(button, row//3, row%3)
        layout.addLayout(icon_grid_layout)
        # Spacer between grid layout and the heading label
        layout.addSpacerItem(QSpacerItem(10, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))

    # Heading text centered at the bottom of icons
        self.heading_label = QLabel("Search a Specific Item?", self)
        self.heading_label.setAlignment(Qt.AlignCenter)
        self.heading_label.setStyleSheet("font-size: 40px; font-weight: bold; color: #333;")
        layout.addWidget(self.heading_label)

    # Spacer for spacing between heading and search bar
        layout.addSpacerItem(QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Fixed))

    # Search bar
        self.search_bar = QLineEdit(self)
        self.search_bar.setPlaceholderText("Search...")
        self.search_bar.setStyleSheet("""
            QLineEdit {
                padding: 20px;
                font-size: 25px;
                background-color: white;
                border: 1px solid #ccc;
                border-radius: 10px;
            }
        """)
        self.search_bar.setAlignment(Qt.AlignCenter)
        self.search_bar.returnPressed.connect(self.handle_search_entered)
        layout.addWidget(self.search_bar)
    

        # Spacer
        layout.addSpacerItem(QSpacerItem(10, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Bottom layout for buttons
        # Bottom layout
        bottom_layout = QHBoxLayout()

        home_button = QToolButton(self)
        home_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/home.png"))
        home_button.setIconSize(QSize(60, 60))
        home_button.setFixedSize(120, 120)
        home_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #87CEEB;
            }
        """)
        home_button.setText("Home")
        home_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        home_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        home_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(1))
        middle_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #87CEEB;
            }
        """)
        # bottom_layout.addWidget(home_button, alignment=Qt.AlignLeft)

        # bottom_layout.addStretch()

        exit_button = QToolButton(self)
        exit_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        exit_button.setIconSize(QSize(100, 100))
        exit_button.setText("Exit")
        exit_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        exit_button.setFont(QFont("Arial", 30, QFont.Bold))
        exit_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #87CEEB;
            }
        """)
        exit_button.setFixedSize(150, 150)
        exit_button.clicked.connect(lambda: self.switch_page_callback(0))
        exit_button.clicked.connect(lambda: self.controller.set_color(1))
        # bottom_layout.addWidget(exit_button, alignment=Qt.AlignRight)
        bottom_left=QHBoxLayout()
        bottom_left.addWidget(home_button,alignment=Qt.AlignLeft )
        bottom_left.addWidget(middle_icon_button,alignment=Qt.AlignLeft)
        left_layout = QVBoxLayout()
        left_layout.addLayout(bottom_left)

        # Right Layout (Table + Button)
        right_layout = QVBoxLayout()
        right_layout.addWidget(exit_button, alignment=Qt.AlignRight)

        # Bottom Layout (Combining Left and Right)
        bottom_layout = QHBoxLayout()
        bottom_layout.addLayout(left_layout)
        bottom_layout.addLayout(right_layout)


        layout.addLayout(bottom_layout)
        self.setLayout(layout)


    def handle_search_entered(self):
      """Handle enter key on search bar."""
      self.last_searched_item = self.search_bar.text().lower().strip()
      print(f"User entered: {self.last_searched_item}")

    # Mapping of product keywords to page numbers
      product_pages = {
        "elarosa tea": 15,
        "elarosa": 15,
        "vcola": 11,
        "spuds chips": 12,
        "spuds": 12,
        "zeina tissues": 13,
        "zeina": 13,
        "oxi dish washer": 14,
        "oxi": 14
      }

      if self.last_searched_item in product_pages:
          self.search_bar.clear()
          self.switch_page_callback(product_pages[self.last_searched_item])
      else:
          self.search_bar.clear()
          msg_box1 = QMessageBox(self)
          msg_box1.setIcon(QMessageBox.Information)
          msg_box1.setText("Product not available")
          msg_box1.setWindowTitle("Warning")
          msg_box1.setStandardButtons(QMessageBox.Ok)
          msg_box1.buttonClicked.connect(lambda: self.switch_page_callback(3))
          msg_box1.exec_()
    
