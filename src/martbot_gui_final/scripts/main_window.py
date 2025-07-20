from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QStackedWidget, QGridLayout, QPushButton,
    QWidget, QToolButton, QLabel, QSizePolicy, QSpacerItem, QHBoxLayout, QLineEdit, QTableWidget, QTableWidgetItem, QFrame, QHeaderView
)
from PyQt5.QtGui import QIcon, QFont, QMovie, QPixmap, QColor
from PyQt5.QtCore import QSize, Qt, QUrl
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import homepage, navpage, mappage, infopage, datasetpage, productdata, Chechout_page,led_try
import sqlite3
from std_msgs.msg import UInt8
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Martbot Navigation")
        self.setStyleSheet("background-color: #ADD8E6;")  # White background color
        # Initialize ROS node only once when the class is instantiated
        # Automatically resize the window to fit the display
        self.setFixedSize(1920, 1080)
        # this will hide the title bar 
        self.setWindowFlag(Qt.FramelessWindowHint)
        # self.resize_to_screen()
        self.controller=led_try.LEDController()
        # Stacked Widget for managing pages
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        # Pages
        self.main_window = self.create_main_page()
        self.home_page = homepage.HomePage(self.switch_page)
        self.icons_page = navpage.navigation(self.switch_page)
        self.info_page = infopage.infowindow(self.switch_page)
        self.cart_page = Chechout_page.CheckoutPage(self.switch_page)
        self.bakery_page = datasetpage.bakery(self.switch_page)
        self.cleaningitems_page = datasetpage.cleaningitems(self.switch_page)
        self.coffeetea_page = datasetpage.coffe_tea(self.switch_page)
        self.dairy_page = datasetpage.dairy(self.switch_page)
        self.drinks_page = datasetpage.drinks(self.switch_page)
        self.snacks_page = datasetpage.snacks(self.switch_page)
        # self.search_page = infopage.searchwindow(self.switch_page)
        self.product1_page = productdata.vcolainfopage(self.switch_page)
        self.product2_page = productdata.spudsinfopage(self.switch_page)
        self.product3_page = productdata.zeinainfopage(self.switch_page)
        self.product4_page = productdata.oxiinfopage(self.switch_page)
        self.product5_page = productdata.elarosainfopage(self.switch_page)
        self.map_page = mappage.Map(self.switch_page)

        # Add pages to the stacked widget
        self.stacked_widget.addWidget(self.main_window)  # Index 0
        self.stacked_widget.addWidget(self.home_page)    # Index 1
        self.stacked_widget.addWidget(self.icons_page)   # Index 2
        self.stacked_widget.addWidget(self.info_page)   # Index 3
        self.stacked_widget.addWidget(self.cart_page)   # Index 4
        self.stacked_widget.addWidget(self.bakery_page)   # Index 5
        self.stacked_widget.addWidget(self.cleaningitems_page)   # Index 6
        self.stacked_widget.addWidget(self.coffeetea_page)   # Index 7
        self.stacked_widget.addWidget(self.dairy_page)   # Index 8
        self.stacked_widget.addWidget(self.drinks_page)   # Index 9
        self.stacked_widget.addWidget(self.snacks_page)   # Index 10
        # self.stacked_widget.addWidget(self.search_page)   # Index 11
        self.stacked_widget.addWidget(self.product1_page)  # Index 11
        self.stacked_widget.addWidget(self.product2_page)  # Index 12
        self.stacked_widget.addWidget(self.product3_page)  # Index 13
        self.stacked_widget.addWidget(self.product4_page)  # Index 14
        self.stacked_widget.addWidget(self.product5_page)  # Index 15
        self.stacked_widget.addWidget(self.map_page)  # Index 16



    def create_main_page(self):
        main_widget = QWidget(self)
        layout = QVBoxLayout(main_widget)

        # Title Label
        label = QLabel("Welcome to Martbot!")
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont("Arial", 70, QFont.Bold))
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)  # Expanding horizontally
        layout.addWidget(label)
        layout.addSpacing(50)  # Adjust this value for more/less space


        # Grid Layout for Icons
        gif_label = QLabel(self)
        gif_label.setAlignment(Qt.AlignCenter)
        movie = QMovie("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/Hi Robot Sticker - Hi Robot - Discover & Share GIFs.gif")  # Replace with the path to your GIF
        gif_label.setMovie(movie)
        movie.start()  # Start playing the GIF
        gif_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # Expand gif label
        layout.addWidget(gif_label, alignment=Qt.AlignCenter)

        # Spacer
        layout.addSpacerItem(QSpacerItem(10, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Bottom corner icons with buttons
        bottom_layout = QHBoxLayout()

        
        # Right Corner Icon and Button
        right_icon_button = QToolButton(self)
        right_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/start.png"))
        right_icon_button.setIconSize(QSize(300, 300))  # Smaller icon size

        right_icon_button.setStyleSheet(
            """
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #ADD8E6;
            }
            """
        )
        right_icon_button.setFixedSize(150, 150)
        right_icon_button.clicked.connect(lambda: self.switch_page(1))
        right_icon_button.clicked.connect(lambda: self.controller.set_color(2))
        bottom_layout.addWidget(right_icon_button, alignment=Qt.AlignCenter)

        layout.addLayout(bottom_layout)

        return main_widget  # Return the main_widget instead of setting layout directly

    def switch_page(self, index):
        """Switch between pages in the stacked widget."""
        self.stacked_widget.setCurrentIndex(index)

    def resize_to_screen(self):
        """Resize the window to fit the screen with a small margin."""
        screen = QApplication.primaryScreen()
        if screen:
            screen_geometry = screen.availableGeometry()
            # Reduce the size by 20 pixels on each side to avoid hiding parts of the window
            width = screen_geometry.width()/10
            height = screen_geometry.height()/10
            self.setGeometry(screen_geometry.x(), screen_geometry.y(), width, height)
            self.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())