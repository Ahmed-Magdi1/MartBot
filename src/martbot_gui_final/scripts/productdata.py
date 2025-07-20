import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextBrowser, QSizePolicy, QToolButton,QTableWidget,QTableWidgetItem
)
from PyQt5.QtGui import QPixmap, QFont, QIcon,QColor
from PyQt5.QtCore import Qt, QSize
import sqlite3
import led_try

class vcolainfopage(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.switch_page_callback = switch_page_callback
        self.controller=led_try.LEDController()

        # Connect to the database
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        print("Connected to database")
        c = conn.cursor()

        # Fetch data from the database
        c.execute("SELECT * FROM products WHERE name = 'Vcola'")
        result = c.fetchone()
        if result:
            product_name = f"{result[1]}"  # Assuming second column is the product name
            product_price = f"Price: {result[2]} EGP"  # Assuming third column is the price
            product_description = f"{result[4]}"  # Assuming fourth column is the description
        else:
            product_name = "Unknown Product"
            product_price = "Price: N/A"
            product_description = "No description available."

        # Product Name (Title, Centered)
        self.product_name_title = QLabel(product_name)
        self.product_name_title.setFont(QFont("Arial", 38, QFont.Bold))
        self.product_name_title.setAlignment(Qt.AlignCenter)

        # Product Image (Centered)
        self.product_image = QLabel()
        pixmap = QPixmap("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/vcola_again-removebg-preview.png")
        self.product_image.setPixmap(pixmap)
        self.product_image.setAlignment(Qt.AlignCenter)
        self.product_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Product Price (Centered)
        self.product_price = QLabel(product_price)
        self.product_price.setFont(QFont("Arial", 25))
        self.product_price.setStyleSheet("color: black;")
        self.product_price.setAlignment(Qt.AlignCenter)

        # Section Titles
        self.product_title = QLabel("Description")
        self.product_title.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title.setStyleSheet("color: black;")
        self.product_title.setAlignment(Qt.AlignLeft)

        self.product_title1 = QLabel("Product Nutritions")
        self.product_title1.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title1.setStyleSheet("color: black;")
        self.product_title1.setAlignment(Qt.AlignRight)

        # Description Box
        self.description = QTextBrowser()
        self.description.setText(product_description)
        self.description.setFont(QFont("Arial", 20))
        self.description.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Nutrition Table (Replacing Additional Image)
        self.nutrition_table = QTableWidget(12, 3)
        self.nutrition_table.setFont(QFont("Arial", 20))
        self.nutrition_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Sample data for the table
        table_data = [
            ["UOM per 300 ml/ net weight 300 ml"],
            ["Nutrient", "Unit", "Amount"],
            ["Calories", "Kcal", "135"],
            ["Calories", "KJ", "564"],
            ["Protein", "g", "0"],
            ["Total Carbohydrate", "g", "33"],
            ["Sugars", "g", "33"],
            ["Total Fat", "g", "3"],
            ["Vitamin  B6", "mg", "1.27"],
            ["Vitamin  B12", "microg", "120"],
            ["Sodium", "g", "0.006"]
            
            
        ]
        for row, data in enumerate(table_data):
           for col, value in enumerate(data):
               item = QTableWidgetItem(value)
               font = QFont("Arial", 12)

        # Make the first row bold
               if row == 0 or row==1:
                  font.setBold(True)
                  if row==0:
                    self.nutrition_table.setSpan(row, 0, 1, 3)
                

               item.setFont(font)
               item.setBackground(QColor("white")) 
               self.nutrition_table.setItem(row, col, item)
        # for row, data in enumerate(table_data):
        #     for col, value in enumerate(data):
        #         self.nutrition_table.setItem(row, col, QTableWidgetItem(value))

        # Left Side Button (Below Description)
        left_icon_button = QToolButton(self)
        left_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/home.png"))
        left_icon_button.setIconSize(QSize(100, 100))
        left_icon_button.setText("Home")
        left_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        left_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        left_icon_button.setFixedSize(150, 150)
        left_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        left_icon_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(9))

        # Right Side Button (Below Table)
        right_icon_button = QToolButton(self)
        right_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        right_icon_button.setIconSize(QSize(100, 100))
        right_icon_button.setText("Exit")
        right_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        right_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        right_icon_button.setFixedSize(150, 150)
        right_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        right_icon_button.clicked.connect(lambda: self.switch_page_callback(0))
        right_icon_button.clicked.connect(lambda: self.controller.set_color(1))


        # Layout for Titles (Above Description & Table)
        title_layout = QHBoxLayout()
        title_layout.addWidget(self.product_title)
        title_layout.addWidget(self.product_title1)

        # Left Layout (Description + Button)
        
        
        bottom_left=QHBoxLayout()
        bottom_left.addWidget(left_icon_button,alignment=Qt.AlignLeft )
        bottom_left.addWidget(middle_icon_button,alignment=Qt.AlignLeft)
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.description, 2)
        left_layout.addLayout(bottom_left)
        # middle_layout = QVBoxLayout()
        # middle_layout.addWidget(self.nutrition_table, 3)
        

        # Right Layout (Table + Button)
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.nutrition_table, 3)
        right_layout.addWidget(right_icon_button, alignment=Qt.AlignRight)
        
        # Right Layout (Table + Button)
        

        # Bottom Layout (Combining Left and Right)
        bottom_layout = QHBoxLayout()
        bottom_layout.addLayout(left_layout)
        bottom_layout.addLayout(right_layout)
        # bottom_layout.addLayout(middle_layout)

        # Main Layout
        layout = QVBoxLayout()
        layout.addWidget(self.product_name_title)
        layout.addWidget(self.product_image)
        layout.addWidget(self.product_price)
        layout.addLayout(title_layout)
        layout.addLayout(bottom_layout)

        self.setLayout(layout)
        self.setWindowTitle("Product Information")
        self.setMinimumSize(800, 600)
        self.resize(1000, 700)



class spudsinfopage(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Connect to the database
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        print("Connected to database")
        c = conn.cursor()

        # Fetch data from the database
        c.execute("SELECT * FROM products WHERE name = 'lemon fusion spuds chips'")
        result = c.fetchone()
        if result:
            product_name = f"{result[1]}"  # Assuming second column is the product name
            product_price = f"Price: {result[2]} EGP"  # Assuming third column is the price
            product_description = f"{result[4]}"  # Assuming fourth column is the description
        else:
            product_name = "Unknown Product"
            product_price = "Price: N/A"
            product_description = "No description available."

        # Product Name (Title, Centered)
        self.product_name_title = QLabel(product_name)
        self.product_name_title.setFont(QFont("Arial", 38, QFont.Bold))
        self.product_name_title.setAlignment(Qt.AlignCenter)

        # Product Image (Centered)
        self.product_image = QLabel()
        pixmap = QPixmap("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/spuds_t-removebg-preview.png")
        self.product_image.setPixmap(pixmap)
        self.product_image.setAlignment(Qt.AlignCenter)
        self.product_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Product Price (Centered)
        self.product_price = QLabel(product_price)
        self.product_price.setFont(QFont("Arial", 25))
        self.product_price.setStyleSheet("color: black;")
        self.product_price.setAlignment(Qt.AlignCenter)

        # Section Titles
        self.product_title = QLabel("Description")
        self.product_title.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title.setStyleSheet("color: black;")
        self.product_title.setAlignment(Qt.AlignLeft)

        self.product_title1 = QLabel("Product Nutritions")
        self.product_title1.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title1.setStyleSheet("color: black;")
        self.product_title1.setAlignment(Qt.AlignRight)

        # Description Box
        self.description = QTextBrowser()
        self.description.setText(product_description)
        self.description.setFont(QFont("Arial", 20))
        self.description.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Nutrition Table (Replacing Additional Image)
        self.nutrition_table = QTableWidget(12, 3)
        self.nutrition_table.setFont(QFont("Arial", 20))
        self.nutrition_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Sample data for the table
        table_data = [
            ["UOM per 10 grams/ net weight 30 grams"],
            ["Nutrient", "Unit", "Amount"],
            ["Calories", "Kcal", "51"],
            ["Calories", "KJ", "213"],
            ["Protein", "g", "0.65"],
            ["Total Carbohydrate", "g", "4.9"],
            ["Sugars", "g", "0.21"],
            ["Total Fat", "g", "3"],
            ["Saturated Fat", "g", "1.27"],
            ["Fiber", "g", "0.49"],
            ["Sodium", "g", "0.05"]
            
            
        ]
        for row, data in enumerate(table_data):
           for col, value in enumerate(data):
               item = QTableWidgetItem(value)
               font = QFont("Arial", 12)

        # Make the first row bold
               if row == 0 or row==1:
                  font.setBold(True)
                  if row==0:
                    self.nutrition_table.setSpan(row, 0, 1, 3)
                

               item.setFont(font)
               item.setBackground(QColor("white")) 
               self.nutrition_table.setItem(row, col, item)
        # for row, data in enumerate(table_data):
        #     for col, value in enumerate(data):
        #         self.nutrition_table.setItem(row, col, QTableWidgetItem(value))

        # Left Side Button (Below Description)
        left_icon_button = QToolButton(self)
        left_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/home.png"))
        left_icon_button.setIconSize(QSize(100, 100))
        left_icon_button.setText("Home")
        left_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        left_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        left_icon_button.setFixedSize(150, 150)
        left_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        left_icon_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(10))

        # Right Side Button (Below Table)
        right_icon_button = QToolButton(self)
        right_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        right_icon_button.setIconSize(QSize(100, 100))
        right_icon_button.setText("Exit")
        right_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        right_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        right_icon_button.setFixedSize(150, 150)
        right_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        right_icon_button.clicked.connect(lambda: self.switch_page_callback(0))
        right_icon_button.clicked.connect(lambda: self.controller.set_color(1))


        # Layout for Titles (Above Description & Table)
        title_layout = QHBoxLayout()
        title_layout.addWidget(self.product_title)
        title_layout.addWidget(self.product_title1)

        bottom_left=QHBoxLayout()
        bottom_left.addWidget(left_icon_button,alignment=Qt.AlignLeft )
        bottom_left.addWidget(middle_icon_button,alignment=Qt.AlignLeft)
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.description, 2)
        left_layout.addLayout(bottom_left)

        # # Left Layout (Description + Button)
        # left_layout = QVBoxLayout()
        # left_layout.addWidget(self.description, 2)
        # left_layout.addWidget(left_icon_button, alignment=Qt.AlignLeft)

        # Right Layout (Table + Button)
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.nutrition_table, 3)
        right_layout.addWidget(right_icon_button, alignment=Qt.AlignRight)

        

        # Bottom Layout (Combining Left and Right)
        bottom_layout = QHBoxLayout()
        bottom_layout.addLayout(left_layout)
        bottom_layout.addLayout(right_layout)

        # Main Layout
        layout = QVBoxLayout()
        layout.addWidget(self.product_name_title)
        layout.addWidget(self.product_image)
        layout.addWidget(self.product_price)
        layout.addLayout(title_layout)
        layout.addLayout(bottom_layout)

        self.setLayout(layout)
        self.setWindowTitle("Product Information")
        self.setMinimumSize(800, 600)
        self.resize(1000, 700)




class zeinainfopage(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Connect to the database
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        print("Connected to database")
        c = conn.cursor()

        # Fetch data from the database
        c.execute("SELECT * FROM products WHERE name = 'zeina tissue'")
        result = c.fetchone()
        if result:
            product_name = f"{result[1]}"  # Assuming second column is the product name
            product_price = f"Price: {result[2]} EGP"  # Assuming third column is the price
            product_description = f"{result[4]}"  # Assuming fourth column is the description
        else:
            product_name = "Unknown Product"
            product_price = "Price: N/A"
            product_description = "No description available."

        # Product Name (Title, Centered)
        self.product_name_title = QLabel(product_name)
        self.product_name_title.setFont(QFont("Arial", 38, QFont.Bold))
        self.product_name_title.setAlignment(Qt.AlignCenter)

        # Product Image (Centered)
        self.product_image = QLabel()
        pixmap = QPixmap("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/tissues-removebg-preview.png")
        self.product_image.setPixmap(pixmap)
        self.product_image.setAlignment(Qt.AlignCenter)
        self.product_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Product Price (Centered)
        self.product_price = QLabel(product_price)
        self.product_price.setFont(QFont("Arial", 25))
        self.product_price.setStyleSheet("color: black;")
        self.product_price.setAlignment(Qt.AlignCenter)

        # Section Titles
        self.product_title = QLabel("Description")
        self.product_title.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title.setStyleSheet("color: black;")
        self.product_title.setAlignment(Qt.AlignLeft)

        self.product_title1 = QLabel("Product Nutritions")
        self.product_title1.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title1.setStyleSheet("color: black;")
        self.product_title1.setAlignment(Qt.AlignRight)

        # Description Box
        self.description = QTextBrowser()
        self.description.setText(product_description)
        self.description.setFont(QFont("Arial", 20))
        self.description.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Nutrition Table (Replacing Additional Image)
        self.nutrition_table = QTableWidget(12, 3)
        self.nutrition_table.setFont(QFont("Arial", 20))
        self.nutrition_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Sample data for the table
        table_data = [
            ["10 soft pocket tissues/net weight 3g"],
                      
            
        ]
        for row, data in enumerate(table_data):
           for col, value in enumerate(data):
               item = QTableWidgetItem(value)
               font = QFont("Arial", 12)

        # Make the first row bold
               if row == 0 or row==1:
                  font.setBold(True)
                  if row==0:
                    self.nutrition_table.setSpan(row, 0, 1, 3)
                

               item.setFont(font)
               item.setBackground(QColor("white")) 
               self.nutrition_table.setItem(row, col, item)
        # for row, data in enumerate(table_data):
        #     for col, value in enumerate(data):
        #         self.nutrition_table.setItem(row, col, QTableWidgetItem(value))

        # Left Side Button (Below Description)
        left_icon_button = QToolButton(self)
        left_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/home.png"))
        left_icon_button.setIconSize(QSize(100, 100))
        left_icon_button.setText("Home")
        left_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        left_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        left_icon_button.setFixedSize(150, 150)
        left_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        left_icon_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(6))

        # Right Side Button (Below Table)
        right_icon_button = QToolButton(self)
        right_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        right_icon_button.setIconSize(QSize(100, 100))
        right_icon_button.setText("Exit")
        right_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        right_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        right_icon_button.setFixedSize(150, 150)
        right_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        right_icon_button.clicked.connect(lambda: self.switch_page_callback(0))
        right_icon_button.clicked.connect(lambda: self.controller.set_color(1))


        # Layout for Titles (Above Description & Table)
        title_layout = QHBoxLayout()
        title_layout.addWidget(self.product_title)
        title_layout.addWidget(self.product_title1)

        bottom_left=QHBoxLayout()
        bottom_left.addWidget(left_icon_button,alignment=Qt.AlignLeft )
        bottom_left.addWidget(middle_icon_button,alignment=Qt.AlignLeft)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.description, 2)
        left_layout.addLayout(bottom_left)

        # Right Layout (Table + Button)
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.nutrition_table, 3)
        right_layout.addWidget(right_icon_button, alignment=Qt.AlignRight)

        # Bottom Layout (Combining Left and Right)
        bottom_layout = QHBoxLayout()
        bottom_layout.addLayout(left_layout)
        bottom_layout.addLayout(right_layout)

        # Main Layout
        layout = QVBoxLayout()
        layout.addWidget(self.product_name_title)
        layout.addWidget(self.product_image)
        layout.addWidget(self.product_price)
        layout.addLayout(title_layout)
        layout.addLayout(bottom_layout)

        self.setLayout(layout)
        self.setWindowTitle("Product Information")
        self.setMinimumSize(800, 600)
        self.resize(1000, 700)




class oxiinfopage(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Connect to the database
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        print("Connected to database")
        c = conn.cursor()

        # Fetch data from the database
        c.execute("SELECT * FROM products WHERE name = 'oxi dish soap'")
        result = c.fetchone()
        if result:
            product_name = f"{result[1]}"  # Assuming second column is the product name
            product_price = f"Price: {result[2]} EGP"  # Assuming third column is the price
            product_description = f"{result[4]}"  # Assuming fourth column is the description
        else:
            product_name = "Unknown Product"
            product_price = "Price: N/A"
            product_description = "No description available."

        # Product Name (Title, Centered)
        self.product_name_title = QLabel(product_name)
        self.product_name_title.setFont(QFont("Arial", 38, QFont.Bold))
        self.product_name_title.setAlignment(Qt.AlignCenter)

        # Product Image (Centered)
        self.product_image = QLabel()
        pixmap = QPixmap("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/oxi-removebg-preview.png")
        self.product_image.setPixmap(pixmap)
        self.product_image.setAlignment(Qt.AlignCenter)
        self.product_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Product Price (Centered)
        self.product_price = QLabel(product_price)
        self.product_price.setFont(QFont("Arial", 25))
        self.product_price.setStyleSheet("color: black;")
        self.product_price.setAlignment(Qt.AlignCenter)

        # Section Titles
        self.product_title = QLabel("Description")
        self.product_title.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title.setStyleSheet("color: black;")
        self.product_title.setAlignment(Qt.AlignLeft)

        self.product_title1 = QLabel("Product Nutriti18ons")
        self.product_title1.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title1.setStyleSheet("color: black;")
        self.product_title1.setAlignment(Qt.AlignRight)

        # Description Box
        self.description = QTextBrowser()
        self.description.setText(product_description)
        self.description.setFont(QFont("Arial", 20))
        self.description.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Nutrition Table (Replacing Additional Image)
        self.nutrition_table = QTableWidget(12, 3)
        self.nutrition_table.setFont(QFont("Arial", 20))
        self.nutrition_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Sample data for the table
        table_data = [
            ["net weight 250ml"]
                     
            
        ]
        for row, data in enumerate(table_data):
           for col, value in enumerate(data):
               item = QTableWidgetItem(value)
               font = QFont("Arial", 12)

        # Make the first row bold
               if row == 0 or row==1:
                  font.setBold(True)
                  if row==0:
                    self.nutrition_table.setSpan(row, 0, 1, 3)
                

               item.setFont(font)
               item.setBackground(QColor("white")) 
               self.nutrition_table.setItem(row, col, item)
        # for row, data in enumerate(table_data):
        #     for col, value in enumerate(data):
        #         self.nutrition_table.setItem(row, col, QTableWidgetItem(value))

        # Left Side Button (Below Description)
        left_icon_button = QToolButton(self)
        left_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/home.png"))
        left_icon_button.setIconSize(QSize(100, 100))
        left_icon_button.setText("Home")
        left_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        left_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        left_icon_button.setFixedSize(150, 150)
        left_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        left_icon_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(6))

        # Right Side Button (Below Table)
        right_icon_button = QToolButton(self)
        right_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        right_icon_button.setIconSize(QSize(100, 100))
        right_icon_button.setText("Exit")
        right_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        right_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        right_icon_button.setFixedSize(150, 150)
        right_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        right_icon_button.clicked.connect(lambda: self.switch_page_callback(0))
        right_icon_button.clicked.connect(lambda: self.controller.set_color(1))

        # Layout for Titles (Above Description & Table)
        title_layout = QHBoxLayout()
        title_layout.addWidget(self.product_title)
        title_layout.addWidget(self.product_title1)

        bottom_left=QHBoxLayout()
        bottom_left.addWidget(left_icon_button,alignment=Qt.AlignLeft )
        bottom_left.addWidget(middle_icon_button,alignment=Qt.AlignLeft)
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.description, 2)
        left_layout.addLayout(bottom_left)
        # Right Layout (Table + Button)
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.nutrition_table, 3)
        right_layout.addWidget(right_icon_button, alignment=Qt.AlignRight)

        # Bottom Layout (Combining Left and Right)
        bottom_layout = QHBoxLayout()
        bottom_layout.addLayout(left_layout)
        bottom_layout.addLayout(right_layout)

        # Main Layout
        layout = QVBoxLayout()
        layout.addWidget(self.product_name_title)
        layout.addWidget(self.product_image)
        layout.addWidget(self.product_price)
        layout.addLayout(title_layout)
        layout.addLayout(bottom_layout)

        self.setLayout(layout)
        self.setWindowTitle("Product Information")
        self.setMinimumSize(800, 600)
        self.resize(1000, 700)





class elarosainfopage(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback

        # Connect to the database
        conn = sqlite3.connect('/home/martbot/martbot_ws/src/martbot_gui/datatry.db')
        print("Connected to database")
        c = conn.cursor()

        # Fetch data from the database
        c.execute("SELECT * FROM products WHERE name = 'Elarosa'")
        result = c.fetchone()
        if result:
            product_name = f"{result[1]}"  # Assuming second column is the product name
            product_price = f"Price: {result[2]} EGP"  # Assuming third column is the price
            product_description = f"{result[4]}"  # Assuming fourth column is the description
        else:
            product_name = "Unknown Product"
            product_price = "Price: N/A"
            product_description = "No description available."

        # Product Name (Title, Centered)
        self.product_name_title = QLabel(product_name)
        self.product_name_title.setFont(QFont("Arial", 38, QFont.Bold))
        self.product_name_title.setAlignment(Qt.AlignCenter)

        # Product Image (Centered)
        self.product_image = QLabel()
        pixmap = QPixmap("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/elarosa-removebg-preview.png")
        self.product_image.setPixmap(pixmap)
        self.product_image.setAlignment(Qt.AlignCenter)
        self.product_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Product Price (Centered)
        self.product_price = QLabel(product_price)
        self.product_price.setFont(QFont("Arial", 25))
        self.product_price.setStyleSheet("color: black;")
        self.product_price.setAlignment(Qt.AlignCenter)

        # Section Titles
        self.product_title = QLabel("Description")
        self.product_title.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title.setStyleSheet("color: black;")
        self.product_title.setAlignment(Qt.AlignLeft)

        self.product_title1 = QLabel("Product Nutritions")
        self.product_title1.setFont(QFont("Arial", 25, QFont.Bold))
        self.product_title1.setStyleSheet("color: black;")
        self.product_title1.setAlignment(Qt.AlignRight)

        # Description Box
        self.description = QTextBrowser()
        self.description.setText(product_description)
        self.description.setFont(QFont("Arial", 25))
        self.description.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Nutrition Table (Replacing Additional Image)
        self.nutrition_table = QTableWidget(12, 3)
        self.nutrition_table.setFont(QFont("Arial", 12))
        self.nutrition_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Sample data for the table
        table_data = [
            ["UOM per 100 grams/ net weight 100 grams"],
                      
            
        ]
        for row, data in enumerate(table_data):
           for col, value in enumerate(data):
               item = QTableWidgetItem(value)
               font = QFont("Arial", 12)

        # Make the first row bold
               if row == 0 or row==1:
                  font.setBold(True)
                  if row==0:
                    self.nutrition_table.setSpan(row, 0, 1, 3)
                

               item.setFont(font)
               item.setBackground(QColor("white")) 
               self.nutrition_table.setItem(row, col, item)
        # for row, data in enumerate(table_data):
        #     for col, value in enumerate(data):
        #         self.nutrition_table.setItem(row, col, QTableWidgetItem(value))

        # Left Side Button (Below Description)
        left_icon_button = QToolButton(self)
        left_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/home.png"))
        left_icon_button.setIconSize(QSize(100, 100))
        left_icon_button.setText("Home")
        left_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        left_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        left_icon_button.setFixedSize(150, 150)
        left_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        left_icon_button.clicked.connect(lambda: self.switch_page_callback(1))

        # Left Side Button (Below Description)
        middle_icon_button = QToolButton(self)
        middle_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        middle_icon_button.setIconSize(QSize(100, 100))
        middle_icon_button.setText("back")
        middle_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        middle_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        middle_icon_button.setFixedSize(150, 150)
        middle_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(7))

        # Right Side Button (Below Table)
        right_icon_button = QToolButton(self)
        right_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        right_icon_button.setIconSize(QSize(100, 100))
        right_icon_button.setText("Exit")
        right_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        right_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        right_icon_button.setFixedSize(150, 150)
        right_icon_button.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color:  #ADD8E6;
            }
        """)
        right_icon_button.clicked.connect(lambda: self.switch_page_callback(0))
        right_icon_button.clicked.connect(lambda: self.controller.set_color(1))

        # Layout for Titles (Above Description & Table)
        title_layout = QHBoxLayout()
        title_layout.addWidget(self.product_title)
        title_layout.addWidget(self.product_title1)

        bottom_left=QHBoxLayout()
        bottom_left.addWidget(left_icon_button,alignment=Qt.AlignLeft )
        bottom_left.addWidget(middle_icon_button,alignment=Qt.AlignLeft)
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.description, 2)
        left_layout.addLayout(bottom_left)

        # Right Layout (Table + Button)
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.nutrition_table, 3)
        right_layout.addWidget(right_icon_button, alignment=Qt.AlignRight)

        # Bottom Layout (Combining Left and Right)
        bottom_layout = QHBoxLayout()
        bottom_layout.addLayout(left_layout)
        bottom_layout.addLayout(right_layout)

        # Main Layout
        layout = QVBoxLayout()
        layout.addWidget(self.product_name_title)
        layout.addWidget(self.product_image)
        layout.addWidget(self.product_price)
        layout.addLayout(title_layout)
        layout.addLayout(bottom_layout)

        self.setLayout(layout)
        self.setWindowTitle("Product Information")
        self.setMinimumSize(800, 600)
        self.resize(1000, 700)