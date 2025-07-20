import rospy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QGridLayout, QPushButton, QDialog,
    QWidget, QToolButton, QLabel, QSizePolicy, QSpacerItem, QHBoxLayout, QLineEdit, QTableWidget, QTableWidgetItem, QFrame, QHeaderView
)
from PyQt5.QtGui import QIcon, QFont, QPixmap, QColor
from PyQt5.QtCore import QSize, Qt
from PyQt5.QtGui import QIcon, QFont, QMovie,QPixmap,QColor
import sys
import navpage, led_try


class Map(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.switch_page_callback = switch_page_callback
        self.controller=led_try.LEDController()
        # Initialize ROS node
        # rospy.init_node('robot_control_node', anonymous=True)
        self.stop_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Set up the layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)  # Remove margins

        # Title Label
        self.label = QLabel("On My Way!")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 60, QFont.Bold))
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # Make label expand
        layout.addWidget(self.label)

        layout.addSpacerItem(QSpacerItem(10, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # GIF Label
        self.gif_label = QLabel(self)
        self.gif_label.setAlignment(Qt.AlignCenter)
        self.movie = QMovie("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/robot_w.gif")  # Replace with the path to your GIF
        self.gif_label.setMovie(self.movie)
        self.movie.start()  # Start playing the GIF
        self.gif_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # Make GIF label expand
        layout.addWidget(self.gif_label)

        # Spacer
        layout.addSpacerItem(QSpacerItem(10, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Stop Button
        self.stop_button = QToolButton(self)
        self.stop_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/stop.png"))
        self.stop_button.setIconSize(QSize(200, 200))  # Icon size
        self.stop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # Expand button
        self.stop_button.setStyleSheet(
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
        self.stop_button.clicked.connect(self.stop_robot)  # Connect to stop_robot method
        self.stop_button.clicked.connect(lambda: self.switch_page_callback(2))
        layout.addWidget(self.stop_button, alignment=Qt.AlignCenter)

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
        middle_icon_button.clicked.connect(lambda: self.switch_page_callback(2))
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
        # self.setLayout(layout)
        # Add the bottom layout to the main layout
      

        

    def stop_robot(self):
        # Create a stop command message
        stop_msg = PoseStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.header.frame_id = "base_link"  # Adjust frame_id as needed
        stop_msg.pose.position.x = 0
        stop_msg.pose.position.y = 0
        stop_msg.pose.position.z = 0
        stop_msg.pose.orientation.x = 0
        stop_msg.pose.orientation.y = 0
        stop_msg.pose.orientation.z = 0
        stop_msg.pose.orientation.w = 1

        # Publish the stop command
        self.stop_publisher.publish(stop_msg)
        print("Robot stopped.")  # For debugging


