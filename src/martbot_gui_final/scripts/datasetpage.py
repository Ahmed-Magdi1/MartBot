from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QStackedWidget, QGridLayout, QPushButton,
    QWidget, QToolButton, QLabel, QSizePolicy, QSpacerItem, QHBoxLayout, QLineEdit,QTableWidget,QTableWidgetItem, QFrame, QHeaderView,QMessageBox
)
from PyQt5.QtGui import QIcon, QFont, QMovie,QPixmap,QColor
from PyQt5.QtCore import QSize, Qt,QUrl
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import homepage,led_try
import sqlite3


        
class bakery(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.switch_page_callback = switch_page_callback
        self.controller=led_try.LEDController()

        # Create a main vertical layout for the entire window
        main_layout = QVBoxLayout(self)

        # Add the "Bakery" label at the top
        self.label = QLabel("Bakery")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 60, QFont.Bold))
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(self.label, stretch=1)

        # Create a horizontal layout for the table
        content_layout = QHBoxLayout()

        # Add the table
        self.tableWidget = QTableWidget()
        self.tableWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        content_layout.addWidget(self.tableWidget, stretch=2)

        # Enable horizontal and vertical headers
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(True)

        # Set the background color of cells to white
        self.tableWidget.setStyleSheet("QTableWidget::item { background-color: white; }")

        # Add the content layout (table) to the main layout
        main_layout.addLayout(content_layout, stretch=4)

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
        home_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        home_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        home_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
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
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(3))
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


        main_layout.addLayout(bottom_layout)
        # self.setLayout(layout)
        # Add the bottom layout to the main layout
      

        # Load data from the database
        self.load_data()

    def load_data(self):
        # Connect to the database
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        print("Connected to database")
        c = conn.cursor()

        # Fetch data from the database
        c.execute("SELECT * FROM products WHERE category = 'bakery'")
        rows = c.fetchall()

        if not rows:
            # Display message in the table instead of a pop-up on startup
            self.tableWidget.setRowCount(1)
            self.tableWidget.setColumnCount(1)
            self.tableWidget.setHorizontalHeaderLabels(['Message'])
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font)            

            no_data_item = QTableWidgetItem("No items found in the 'bakery' category.")
            no_data_item.setBackground(QColor(255, 255, 255))
            self.tableWidget.setItem(0, 0, no_data_item)
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.tableWidget.cellClicked.connect(self.show_message)  # Show message only when clicked
        else:
            # Populate the table with data
            self.tableWidget.setRowCount(len(rows))
            self.tableWidget.setColumnCount(len(rows[0]) - 1)
            self.tableWidget.setHorizontalHeaderLabels(['code', 'Name', 'price', 'category'])
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font) 


            for i, row in enumerate(rows):
                for j, item in enumerate(row):
                    table_item = QTableWidgetItem(str(item))
                    table_item.setBackground(QColor(255, 255, 255))
                    self.tableWidget.setItem(i, j, table_item)

            self.tableWidget.cellClicked.connect(self.on_cell_clicked)

        # Close the database connection
        conn.close()


    def on_cell_clicked(self, row, column):
        self.switch_page_callback(13 + row)

    def show_message(self):
        """
        Displays a message box when no bakery items are available.
        """
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("No bakery items available in the database.")
        msg_box.setWindowTitle("No Data")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

     

class cleaningitems(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Main layout
        main_layout = QVBoxLayout(self)

        # Title label
        self.label = QLabel("Cleaning Items")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 30, QFont.Bold))
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(self.label, stretch=1)

        # Content layout
        content_layout = QHBoxLayout()

        # Table
        self.tableWidget = QTableWidget()
        self.tableWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        content_layout.addWidget(self.tableWidget, stretch=2)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(True)
        self.tableWidget.setStyleSheet("QTableWidget::item { background-color: white; }")
        
        main_layout.addLayout(content_layout, stretch=4)

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
        home_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        home_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        home_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
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
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(3))
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


        main_layout.addLayout(bottom_layout)
        # self.setLayout(layout)
        # Add the bottom layout to the main layout
      

        # Load data from the database
        self.load_data()

    def load_data(self):
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        c = conn.cursor()
        c.execute("SELECT * FROM products WHERE category = 'cleaning items'")
        rows = c.fetchall()

        if not rows:
            self.tableWidget.setRowCount(1)
            self.tableWidget.setColumnCount(1)
            self.tableWidget.setHorizontalHeaderLabels(['Message'])
            no_data_item = QTableWidgetItem("No items found in 'cleaning items'.")
            no_data_item.setBackground(QColor(255, 255, 255))
            self.tableWidget.setItem(0, 0, no_data_item)
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.tableWidget.cellClicked.connect(lambda row, column: self.show_message())
        else:
            self.tableWidget.setRowCount(len(rows))
            self.tableWidget.setColumnCount(len(rows[0]) - 1)
            self.tableWidget.setHorizontalHeaderLabels(['Code', 'Name', 'Price', 'Category'])
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font)            
            for i, row in enumerate(rows):
                for j, item in enumerate(row):
                    table_item = QTableWidgetItem(str(item))
                    table_item.setBackground(QColor(255, 255, 255))
                    self.tableWidget.setItem(i, j, table_item)

            self.tableWidget.cellClicked.connect(self.on_cell_clicked)

        conn.close()

    def on_cell_clicked(self, row, column):
        self.switch_page_callback(13 + row)
    
    def show_message(self):
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("No cleaning items available.")
        msg_box.setWindowTitle("No Data")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()
             


class coffe_tea(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Create a main vertical layout for the entire window
        main_layout = QVBoxLayout(self)

        # Add the "Coffe & Tea" label at the top
        self.label = QLabel("Coffe & Tea")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 30, QFont.Bold))
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(self.label, stretch=1)

        # Create a horizontal layout for the table
        content_layout = QHBoxLayout()
        
        # Add the table
        self.tableWidget = QTableWidget()
        self.tableWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        content_layout.addWidget(self.tableWidget, stretch=2)
        
        # Enable headers
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(True)
        self.tableWidget.setStyleSheet("QTableWidget::item { background-color: white; }")
        main_layout.addLayout(content_layout, stretch=4)

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
        home_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        home_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        home_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
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
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(3))
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


        main_layout.addLayout(bottom_layout)
        # self.setLayout(layout)
        # Add the bottom layout to the main layout
      

        # Load data from the database
        self.load_data()

    def load_data(self):
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        c = conn.cursor()
        c.execute("SELECT * FROM products WHERE category = 'coffeetea'")
        rows = c.fetchall()

        if not rows:
            self.tableWidget.setRowCount(1)
            self.tableWidget.setColumnCount(1)
            self.tableWidget.setHorizontalHeaderLabels(['Message'])
            no_data_item = QTableWidgetItem("No items found in the 'Coffe & Tea' category.")
            no_data_item.setBackground(QColor(255, 255, 255))
            self.tableWidget.setItem(0, 0, no_data_item)
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.tableWidget.cellClicked.connect(self.show_message)
        else:
            self.tableWidget.setRowCount(len(rows))
            self.tableWidget.setColumnCount(len(rows[0])-1)
            self.tableWidget.setHorizontalHeaderLabels(['Code', 'Name', 'Price', 'Category'])
            self.tableWidget.cellClicked.connect(self.on_cell_clicked)
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font)            

            for i, row in enumerate(rows):
                for j, item in enumerate(row):
                    table_item = QTableWidgetItem(str(item))
                    table_item.setBackground(QColor(255, 255, 255))
                    self.tableWidget.setItem(i, j, table_item)

        conn.close()

    def on_cell_clicked(self, row, column):
        self.switch_page_callback(15 + row)

    def show_message(self):
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("no coffee and tea items available")
        msg_box.setWindowTitle("no data")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.buttonClicked.connect(self.switch_page_callback(7))
        msg_box.exec_()



class dairy(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Main layout
        main_layout = QVBoxLayout(self)

        # Header label
        self.label = QLabel("Dairy")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 30, QFont.Bold))
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(self.label, stretch=1)

        # Content layout
        content_layout = QHBoxLayout()
        
        # Table Widget
        self.tableWidget = QTableWidget()
        self.tableWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(True)
        self.tableWidget.setStyleSheet("QTableWidget::item { background-color: white; }")
        content_layout.addWidget(self.tableWidget, stretch=2)

        main_layout.addLayout(content_layout, stretch=4)

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
        home_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        home_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        home_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
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
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(3))
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


        main_layout.addLayout(bottom_layout)
        # self.setLayout(layout)
        # Add the bottom layout to the main layout
      

        # Load data from the database
        self.load_data()

    def load_data(self):
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        c = conn.cursor()
        c.execute("SELECT * FROM products WHERE category = 'dairy'")
        rows = c.fetchall()

        if not rows:
            self.tableWidget.setRowCount(1)
            self.tableWidget.setColumnCount(1)
            self.tableWidget.setHorizontalHeaderLabels(['Message'])
            no_data_item = QTableWidgetItem("No items found in the 'dairy' category.")
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font)            

            no_data_item.setBackground(QColor(255, 255, 255))
            self.tableWidget.setItem(0, 0, no_data_item)
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.tableWidget.cellClicked.connect(self.show_message)
        else:
            self.tableWidget.setRowCount(len(rows))
            self.tableWidget.setColumnCount(len(rows[0]) - 1)
            self.tableWidget.setHorizontalHeaderLabels(['Code', 'Name', 'Price', 'Category'])
            self.tableWidget.cellClicked.connect(self.on_cell_clicked)
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font)            

            for i, row in enumerate(rows):
                for j, item in enumerate(row):
                    table_item = QTableWidgetItem(str(item))
                    table_item.setBackground(QColor(255, 255, 255))
                    self.tableWidget.setItem(i, j, table_item)
        conn.close()

    def on_cell_clicked(self, row, column):
        self.switch_page_callback(13 + row)
    
    def show_message(self):
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("no dairy items available!")
        msg_box.setWindowTitle("no data")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.buttonClicked.connect(lambda: self.switch_page_callback(8))
        msg_box.exec_()   



class drinks(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Main layout
        main_layout = QVBoxLayout(self)

        # Title label
        self.label = QLabel("Drinks")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 30, QFont.Bold))
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(self.label, stretch=1)

        # Content layout
        content_layout = QHBoxLayout()

        # Table
        self.tableWidget = QTableWidget()
        self.tableWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        content_layout.addWidget(self.tableWidget, stretch=2)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(True)
        self.tableWidget.setStyleSheet("QTableWidget::item { background-color: white; }")
        
        main_layout.addLayout(content_layout, stretch=4)

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
        home_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        home_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        home_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
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
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(3))
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


        main_layout.addLayout(bottom_layout)
        # self.setLayout(layout)
        # Add the bottom layout to the main layout
      

        # Load data from the database
        self.load_data()

    def load_data(self):
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        c = conn.cursor()
        c.execute("SELECT * FROM products WHERE category = 'drinks'")
        rows = c.fetchall()

        if not rows:
            self.tableWidget.setRowCount(1)
            self.tableWidget.setColumnCount(1)
            self.tableWidget.setHorizontalHeaderLabels(['Message'])
            no_data_item = QTableWidgetItem("No items found in 'drinks'.")
            no_data_item.setBackground(QColor(255, 255, 255))
            self.tableWidget.setItem(0, 0, no_data_item)
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            
            self.tableWidget.cellClicked.connect(lambda row, column: self.show_message())
        else:
            self.tableWidget.setRowCount(len(rows))
            self.tableWidget.setColumnCount(len(rows[0]) - 1)
            self.tableWidget.setHorizontalHeaderLabels(['Code', 'Name', 'Price', 'Category'])
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font)            

            for i, row in enumerate(rows):
                for j, item in enumerate(row):
                    table_item = QTableWidgetItem(str(item))
                    table_item.setBackground(QColor(255, 255, 255))
                    self.tableWidget.setItem(i, j, table_item)
            
            self.tableWidget.cellClicked.connect(self.on_cell_clicked)

        conn.close()

    def on_cell_clicked(self, row, column):
        self.switch_page_callback(11 + row)
    
    def show_message(self):
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("No drinks available.")
        msg_box.setWindowTitle("No Data")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()


 
class snacks(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Create a main vertical layout for the entire window
        main_layout = QVBoxLayout(self)

        # Add the "Snacks" label at the top
        self.label = QLabel("Snacks")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 30, QFont.Bold))
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(self.label, stretch=1)

        # Create a horizontal layout for the images and table
        content_layout = QHBoxLayout()

        # Add the table on the right
        self.tableWidget = QTableWidget()
        self.tableWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        content_layout.addWidget(self.tableWidget, stretch=2)

        # Enable horizontal and vertical headers
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(True)

        # Set the background color of cells to white
        self.tableWidget.setStyleSheet("QTableWidget::item { background-color: white; }")

        # Add the content layout (images + table) to the main layout
        main_layout.addLayout(content_layout, stretch=4)

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
        home_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        home_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        home_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
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
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(3))
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


        main_layout.addLayout(bottom_layout)
        # self.setLayout(layout)
        # Add the bottom layout to the main layout
      

        # Load data from the database
        self.load_data()

    def load_data(self):
        # Connect to the database
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        print("Connected to database")
        c = conn.cursor()

        # Fetch data from the database
        c.execute("SELECT * FROM products WHERE category = 'snacks'")
        rows = c.fetchall()

        if not rows:
            # Show message box when no data is found
            QMessageBox.information(self, "No Data", "No items found in the 'snacks' category.")

            # Display message in the table
            self.tableWidget.setRowCount(1)
            self.tableWidget.setColumnCount(1)
            self.tableWidget.setHorizontalHeaderLabels(['Message'])
            no_data_item = QTableWidgetItem("No items found in the 'snacks' category.")
            no_data_item.setBackground(QColor(255, 255, 255))
            self.tableWidget.setItem(0, 0, no_data_item)
            self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
                       

            self.tableWidget.cellClicked.connect(self.show_message)
        else:
            # Populate the table with data
            self.tableWidget.setRowCount(len(rows))
            self.tableWidget.setColumnCount(len(rows[0]) - 1)
            self.tableWidget.setHorizontalHeaderLabels(['code', 'Name', 'price', 'category'])
            self.tableWidget.setFont(QFont("Arial", 20, QFont.Bold))
            header_font = QFont("Arial", 30, QFont.Bold)  # Change the font size to 16
            self.tableWidget.horizontalHeader().setFont(header_font)            

            for i, row in enumerate(rows):
                for j, item in enumerate(row):
                    table_item = QTableWidgetItem(str(item))
                    table_item.setBackground(QColor(255, 255, 255))
                    self.tableWidget.setItem(i, j, table_item)

            self.tableWidget.cellClicked.connect(self.on_cell_clicked)

        # Close the database connection
        conn.close()

    def on_cell_clicked(self, row, column):
        self.switch_page_callback(12 + row)

    def show_message(self):
        """
        Displays a message box when the table has no data.
        """
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("No snacks available in the database.")
        msg_box.setWindowTitle("No Data")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()