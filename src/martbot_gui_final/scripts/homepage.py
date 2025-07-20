from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QGridLayout, QToolButton, QHBoxLayout, QSpacerItem, QSizePolicy, QMessageBox
)
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import Qt, QSize, QProcess
import led_try


class HomePage(QWidget):
    def __init__(self, switch_page_callback, parent=None):
        super().__init__(parent)
        self.switch_page_callback = switch_page_callback
        self.launch_process = QProcess(self)  # Initialize QProcess for YOLOv5
        self.controller=led_try.LEDController()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.addSpacerItem(QSpacerItem(10, 100, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Title label
        label = QLabel("How can I help?")
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont("Lobster", 60, QFont.Normal))
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)  # Expand horizontally
        layout.addWidget(label, stretch=1)  # Add stretch to push the grid to the center

        # Spacer below the title
        layout.addSpacerItem(QSpacerItem(10, 50, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Icon grid
        self.icons = [
            "/home/martbot/martbot_ws/src/martbot_gui/scripts/images/sup.png",
            "/home/martbot/martbot_ws/src/martbot_gui/scripts/images/barcode.png",
            "/home/martbot/martbot_ws/src/martbot_gui/scripts/images/cart.png"
        ]
        self.names = ["Category Guide", "Product Info", "Cart"]

        icon_grid_layout = QGridLayout()
        icon_grid_layout.setSpacing(10)  # Add spacing between icons

        for col in range(3):  # 3 icons in a single row
            button = QToolButton(self)
            button.setIcon(QIcon(self.icons[col]))
            button.setIconSize(QSize(150, 150))  # Larger icon size
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # Expand button
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
            button.setText(self.names[col])
            button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            button.setFixedSize(400, 400)  # Button size
            button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
            icon_grid_layout.addWidget(button, 0, col, alignment=Qt.AlignCenter)  # Center alignment
            button.clicked.connect(lambda checked, idx=col + 2: self.switch_page_callback(idx))

            # Connect buttons to their respective actions
            if col == 2:  # Cart button (column 2)
                button.clicked.connect(self.launch_yolov5)
                

        # Add the grid layout to the main layout with stretch to center it
        layout.addLayout(icon_grid_layout, stretch=2)  # Add stretch to center the grid vertically

        # Bottom corner icons with buttons
        bottom_layout = QHBoxLayout()
        bottom_layout.addStretch()  # Add stretch to push the button to the right

        # Right Corner Icon and Button (Exit)
        right_icon_button = QToolButton(self)
        right_icon_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui/scripts/images/exit.png"))
        right_icon_button.setIconSize(QSize(100, 100))  # Smaller icon size
        right_icon_button.setText("Exit")
        right_icon_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        right_icon_button.setFont(QFont("Arial", 30, QFont.Bold))  # Larger font for text
        right_icon_button.setStyleSheet(
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
        right_icon_button.setFixedSize(150, 150)
        right_icon_button.clicked.connect(lambda: self.switch_page_callback(0))
        right_icon_button.clicked.connect(lambda: self.controller.set_color(1))
        bottom_layout.addWidget(right_icon_button, alignment=Qt.AlignRight)

        layout.addLayout(bottom_layout)

        self.setLayout(layout)

    def launch_yolov5(self):
        """Launch YOLOv5 ROS node."""
        if self.launch_process.state() == QProcess.Running:
            return

        try:
            print("Launching YOLOv5 AI...")
            self.launch_process.setWorkingDirectory('/home/martbot/martbot_ws')  # Set working directory
            self.launch_process.start('roslaunch', ['yolov5_ros', 'yolov5.launch'])  # Launch YOLOv5
            if self.launch_process.waitForStarted():
                print("YOLOv5 started successfully")
            else:
                QMessageBox.critical(self, "Error", "Failed to start YOLOv5")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    def stop_yolov5(self):
        """Terminate YOLOv5 process."""
        if self.launch_process.state() != QProcess.Running:
            return

        print("Stopping YOLOv5 AI...")
        self.launch_process.terminate()
        self.launch_process.waitForFinished()