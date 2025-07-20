import os
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem, QMessageBox, QSpacerItem, QSizePolicy, QToolButton, QLabel
)
from PyQt5.QtCore import QProcess, Qt, QTimer, pyqtSignal, QSize
from PyQt5.QtGui import QFont, QIcon  # Added for font styling
from detection_msgs.msg import BoundingBoxes
import rospy
from collections import defaultdict
import homepage,led_try


# Price dictionary for objects
OBJECT_PRICES = {
    "Elarosa_tea": 24.00,
    "Oxi Dish Soap": 24.75,
    "SPUDS Chips": 10.00,
    "V-cola": 10.00,
    "Zeina Tissues": 5.00,
    "unknown": 0.0
}

def load_ros_environment():
    """Source ROS setup files and set environment variables."""
    ros_setup_path = '/opt/ros/noetic/setup.bash'
    workspace_setup_path = '/home/martbot/martbot_ws/devel/setup.bash'

    temp_script = '/tmp/ros_env.sh'
    with open(temp_script, 'w') as f:
        f.write(f'source {ros_setup_path}\n')
        f.write(f'source {workspace_setup_path}\n')
        f.write('env > /tmp/ros_env.out\n')

    subprocess.run(['/bin/bash', '-c', f'source {temp_script}'])
    
    with open('/tmp/ros_env.out', 'r') as f:
        for line in f:
            if '=' in line:
                key, value = line.strip().split('=', 1)
                os.environ[key] = value

class CheckoutPage(QWidget):
    update_table_signal = pyqtSignal()

    def __init__(self, switch_page_callback):
        super().__init__()
        self.launch_process = QProcess(self)
        self.detected_objects = defaultdict(int)
        self.initUI()
        self.controller=led_try.LEDController()
        self.switch_page_callback = switch_page_callback
        self.launch_process.readyReadStandardOutput.connect(self.read_output)
        self.launch_process.readyReadStandardError.connect(self.read_error)
        self.subscriber = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bbox_callback)
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.process_ros_events)
        self.ros_timer.start(100)
        self.update_table_signal.connect(self.update_table)
        
    def initUI(self):
        """Set up the user interface with updated styling."""
        self.setWindowTitle("Launch YOLOv5 AI")
        self.setGeometry(100, 100, 800, 600)
        
        # Set background color for the main window
        self.setStyleSheet("background-color: #ADD8E6;")

        # Main layout (center everything)
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignCenter)
        main_layout.setContentsMargins(10, 10, 10, 10)  # Add padding around edges

        # Spacer to push the table content to the center
        spacer = QSpacerItem(4000, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        main_layout.addItem(spacer)

        # Title above the table
        title_label = QLabel("what is in your cart?")
        title_label.setFont(QFont("Arial", 40, QFont.Bold))  # Set font and style
        title_label.setAlignment(Qt.AlignCenter)  # Center align the title
        title_label.setStyleSheet("color: #0d1011;")
        main_layout.addWidget(title_label)

        # Add vertical space between title and table
        main_layout.addSpacing(50)  # Adjust this value for more/less space

        # Table Layout
        table_layout = QVBoxLayout()
        table_layout.setSpacing(10)

        # Table Widget for Detected Objects
        self.table_widget = QTableWidget()
        self.table_widget.setColumnCount(3)
        self.table_widget.setHorizontalHeaderLabels(['QTY', 'ITEM', 'Price'])
        
        # Set background color for the table widget
        self.table_widget.setStyleSheet("""
            QTableWidget {
                background-color: #fef9e7;
                border: 2px solid #0d1011;
                border-radius: 2px; 
            }
            QHeaderView::section {
                background-color: #fef9e7;
                padding: 4px;
                font-size: 18px;
                font-weight: bold;
            }
            QTableWidget::item {
                padding: 0px;
                border: none;
            }
        """)
        
        self.table_widget.setFixedSize(500, 500)  # Adjust these values as needed
        table_layout.addWidget(self.table_widget, alignment=Qt.AlignCenter)
        main_layout.addLayout(table_layout)

        # Spacer to push content down to the bottom
        bottom_spacer = QSpacerItem(20, 100, QSizePolicy.Minimum, QSizePolicy.Expanding)
        main_layout.addItem(bottom_spacer)

        # Horizontal layout for buttons at the bottom
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)  # Remove unnecessary margins

        # Home button (Far left) - Inspired by the provided code
        self.home_btn = QToolButton(self)
        self.home_btn.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/home.png"))
        self.home_btn.setIconSize(QSize(60, 60))
        self.home_btn.setFixedSize(120, 120)
        self.home_btn.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #ADD8E6;
            }
        """)
        self.home_btn.setText("Home")
        self.home_btn.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        self.home_btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.home_btn.clicked.connect(lambda: self.switch_page_callback(1))
        button_layout.addWidget(self.home_btn)

        # Back button (Beside the Home button)
        self.back_btn = QToolButton(self)
        self.back_btn.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/back.png"))
        self.back_btn.setIconSize(QSize(100, 100))
        self.back_btn.setText("Back")
        self.back_btn.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        self.back_btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.back_btn.setFixedSize(150, 150)
        self.back_btn.clicked.connect(lambda: self.switch_page_callback(1))
        self.back_btn.setStyleSheet("""
            QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #ADD8E6;
            }
        """)
        button_layout.addWidget(self.back_btn)

        # Spacer to push buttons to the right
        spacer_buttons = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        button_layout.addItem(spacer_buttons)

        # Checkout button (Far right beside exit)
        self.checkout_btn = QToolButton(self)
        self.checkout_btn.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/360_F_407017314_1SkaeMNjA2bKorKADHdEz2kBmklcmhYe-removebg-preview.png"))
        self.checkout_btn.setText("Checkout")
        self.checkout_btn.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        self.checkout_btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.checkout_btn.setIconSize(QSize(300, 300))
        self.checkout_btn.setFixedSize(300, 300)
        self.checkout_btn.setStyleSheet("""
              QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #ADD8E6;
            }
        """)
        self.checkout_btn.clicked.connect(lambda: self.checkout_button_clicked())
        button_layout.addWidget(self.checkout_btn)

        # Exit button
        exit_btn = QToolButton(self)
        exit_btn.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        exit_btn.setIconSize(QSize(100, 100))
        exit_btn.setFixedSize(150, 150)
        exit_btn.setStyleSheet("""
              QToolButton {
                border: none;
                background-color: #ADD8E6;
                border-radius: 10px;
            }
            QToolButton:hover {
                background-color: #ADD8E6;
            }
        """)
        exit_btn.setText("Exit")
        exit_btn.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        exit_btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        exit_btn.clicked.connect(lambda: self.switch_page_callback(0))
        exit_btn.clicked.connect(lambda: self.controller.set_color(1))
        button_layout.addWidget(exit_btn)

        # Add the button layout to the main layout (below the table)
        main_layout.addLayout(button_layout)

        # Set the main layout for the window
        self.setLayout(main_layout)

    def bbox_callback(self, msg):
        """Process incoming detection messages and update counts."""
        class_counts = defaultdict(int)
        for bbox in msg.bounding_boxes:
            class_name = bbox.Class
            class_counts[class_name] += 1
        self.detected_objects = class_counts
        self.update_table_signal.emit()

    def update_table(self):
        """Update the table widget with current detection counts, prices, and total."""
        total = sum(
            count * OBJECT_PRICES.get(obj, OBJECT_PRICES["unknown"])
            for obj, count in self.detected_objects.items()
        )
        
        row_count = len(self.detected_objects)
        self.table_widget.setRowCount(row_count + 1)
        
        # Style for total row
        total_font = QFont("Arial", 10, QFont.Bold)
        
        # Populate item rows
        for row, (obj, count) in enumerate(self.detected_objects.items()):
            price = OBJECT_PRICES.get(obj, OBJECT_PRICES["unknown"])
            total_price = count * price
            
            qty_item = QTableWidgetItem(str(count))
            item_name = QTableWidgetItem(obj)
            price_item = QTableWidgetItem(f"{total_price:.2f} EGP")
            
            price_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            self.table_widget.setItem(row, 0, qty_item)
            self.table_widget.setItem(row, 1, item_name)
            self.table_widget.setItem(row, 2, price_item)
        
        # Add total row
        total_label = QTableWidgetItem("Total")
        total_label.setFont(total_font)
        self.table_widget.setItem(row_count, 1, total_label)
        
        total_amount = QTableWidgetItem(f"{total:.2f} EGP")
        total_amount.setFont(total_font)
        total_amount.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.table_widget.setItem(row_count, 2, total_amount)
        
        # Adjust column widths
        self.table_widget.resizeColumnsToContents()
        self.table_widget.horizontalHeader().setStretchLastSection(True)

    def checkout_button_clicked(self):
        """Display a popup message and switch to the home page."""
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("Thank you for your purchase \n  hope to see you again!")
        msg_box.setWindowTitle("Checkout")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.buttonClicked.connect(lambda: self.switch_page_callback(0))
        msg_box.exec_()


    def process_ros_events(self):
        """Process ROS events periodically."""
        rospy.spinOnce()

    def launch_yolov5(self):
        """Launch YOLOv5 ROS node."""
        if self.launch_process.state() == QProcess.Running:
            return

        try:
            print("Launching YOLOv5 AI...")
            self.launch_process.setWorkingDirectory('/home/martbot/martbot_ws')
            self.launch_process.start('roslaunch', ['yolov5_ros', 'yolov5.launch'])
            
            if self.launch_process.waitForStarted():
                print("YOLOv5 started successfully")
            else:
                QMessageBox.critical(self, "Error", "Failed to start YOLOv5")

        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def exit_application(self):
        """Handle the Exit button press."""
        if self.launch_process.state() == QProcess.Running:
            QMessageBox.information(self, "Shutting Down YOLOv5", "YOLOv5 node is running. Shutting it down...")
            self.launch_process.kill()

        QApplication.quit()

    def read_output(self):
        """Handle the standard output of the QProcess (YOLOv5 node)."""
        output = self.launch_process.readAllStandardOutput().data().decode('utf-8')
        print(f"Process output: {output}")

    def read_error(self):
        """Handle any error output from the QProcess."""
        error_output = self.launch_process.readAllStandardError().data().decode('utf-8')
        print(f"Error output: {error_output}")

