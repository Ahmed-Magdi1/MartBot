from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout, QToolButton, QLabel, QSizePolicy,
    QSpacerItem, QLineEdit, QHBoxLayout, QMessageBox
)
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QSize, Qt
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray


class navigation(QWidget):
    def __init__(self, switch_page_callback):
        super().__init__()
        self.switch_page_callback = switch_page_callback
        self.last_searched_item = ""  # To store the searched text
        with open('/home/martbot/martbot_ws/src/martbot_gui_final/config/goals.yaml', 'r') as file:
          config = yaml.safe_load(file)

        self.goals_dict = config['goals']
        # self.product_goals = config['products']
        self.init_ui()

        # ROS node and publisher setup
        rospy.init_node('navigation_gui', anonymous=True)
        self.current_goal_id = None
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_status_subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)

    def init_ui(self):
        layout = QVBoxLayout()

        # Title
        title_label = QLabel("Want to Go Somewhere?", self)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 40, QFont.Bold))
        title_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(title_label)

        layout.addSpacerItem(QSpacerItem(10, 100, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Icons grid
        icon_grid_layout = QGridLayout()
        icon_grid_layout.setVerticalSpacing(50)

        self.icons = [
            "/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/bread.png",
            "/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/cleaning_items2-removebg-preview (1) (1).png",
            "/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/cofee_tea-removebg-preview (1).png",
            "/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/dairy.png",
            "/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/drinks-removebg-preview.png",
            "/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/snacks.png"
        ]
        self.names = ["Bakery", "Cleaning items", "Coffee-Tea", "Dairy", "Drinks", "Snacks"]
       # self.goals = [(6.27, -1.26), (-5.83, 3.59), (3.0, 4.0), (4.0, 5.0), (5.0, 6.0), (6.0, 7.0)]
        self.goals = list(self.goals_dict.values())

        for row in range(6):
            button = QToolButton(self)
            button.setIcon(QIcon(self.icons[row]))
            button.setIconSize(QSize(150, 150))
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            button.setFixedSize(300, 300)
            button.setStyleSheet("""
                QToolButton {
                    border: none;
                    background-color: #ADD8E6;
                    border-radius: 10px;
                }
                QToolButton:hover {
                    background-color: #87CEEB;
                }
            """)
            button.setText(self.names[row])
            button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            button.clicked.connect(lambda checked, idx=row: self.send_navigation_goal(idx))
            button.clicked.connect(lambda: self.switch_page_callback(16))
            icon_grid_layout.addWidget(button, row // 3, row % 3)

        layout.addLayout(icon_grid_layout)
        layout.addSpacerItem(QSpacerItem(10, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Heading below icons
        heading_label = QLabel("Search a Specific Item?", self)
        heading_label.setAlignment(Qt.AlignCenter)
        heading_label.setStyleSheet("font-size: 40px; font-weight: bold; color: #333;")
        layout.addWidget(heading_label)

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

        layout.addSpacerItem(QSpacerItem(10, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # Bottom layout
        bottom_layout = QHBoxLayout()

        home_button = QToolButton(self)
        home_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/home.png"))
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
        home_button.clicked.connect(lambda: self.switch_page_callback(1))
        bottom_layout.addWidget(home_button, alignment=Qt.AlignLeft)

        bottom_layout.addStretch()

        exit_button = QToolButton(self)
        exit_button.setIcon(QIcon("/home/martbot/martbot_ws/src/martbot_gui_final/scripts/images/exit.png"))
        exit_button.setIconSize(QSize(100, 100))
        exit_button.setText("Exit")
        exit_button.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        exit_button.setFont(QFont("Arial", 10, QFont.Bold))
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
        bottom_layout.addWidget(exit_button, alignment=Qt.AlignRight)

        layout.addLayout(bottom_layout)
        self.setLayout(layout)

    def handle_search_entered(self):
        """Handle enter key on search bar."""
        self.last_searched_item = self.search_bar.text().lower().strip()
        print(f"User entered: {self.last_searched_item}")

        product_pages = {
            "elarosa tea": self.goals[2],
            "elarosa": self.goals[2],
            "vcola": self.goals[4],
            "spuds chips": self.goals[5],
            "spuds": self.goals[5],
            "zeina tissues": self.goals[3],
            "zeina": self.goals[3],
            "oxi dish washer": self.goals[3],
            "oxi": self.goals[3]
        }

        if self.last_searched_item in product_pages:
            self.search_bar.clear()
            goal_coordinates = product_pages[self.last_searched_item]
            self.send_navigation_goal(goal_coordinates)
            self.switch_page_callback(15)
        else:
            self.search_bar.clear()
            msg_box = QMessageBox(self)
            msg_box.setIcon(QMessageBox.Information)
            msg_box.setText("Product not available")
            msg_box.setWindowTitle("Warning")
            msg_box.setStandardButtons(QMessageBox.Ok)
            msg_box.buttonClicked.connect(lambda: self.switch_page_callback(2))
            msg_box.exec_()
        # goal_coordinates = self.product_goals.get(self.last_searched_item)
        # if goal_coordinates:
        #     self.search_bar.clear()
        #     self.send_navigation_goal(goal_coordinates)
        #     self.switch_page_callback(15)
        # else:
        #     self.search_bar.clear()
        #     msg_box = QMessageBox(self)
        #     msg_box.setIcon(QMessageBox.Information)
        #     msg_box.setText("Product not available")
        #     msg_box.setWindowTitle("Warning")
        #     msg_box.setStandardButtons(QMessageBox.Ok)
        #     msg_box.buttonClicked.connect(lambda: self.switch_page_callback(2))
        #     msg_box.exec_()

    def send_navigation_goal(self, goal):
        """Send a navigation goal. Accepts either an index or (x, y) tuple."""
        if isinstance(goal, int):  # If an index is passed
            x, y = self.goals[goal]
        else:  # If coordinates are passed
            x, y = goal

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "map"
        pose_goal.header.stamp = rospy.Time.now()
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = 0
        pose_goal.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.goal_publisher.publish(pose_goal)
        self.current_goal_id = pose_goal.header.stamp
        rospy.loginfo(f"Current goal ID: {self.current_goal_id}")

    def goal_status_callback(self, status_msg):
        if self.current_goal_id is None:
            return
        for status in status_msg.status_list:
            if abs((status.goal_id.stamp - self.current_goal_id).to_sec()) < 0.5:
                if status.status == 3:
                    rospy.loginfo("Goal reached! Showing message box.")
                    self.show_goal_reached_message()
                    self.current_goal_id = None

    def show_goal_reached_message(self):
        msg_box = QMessageBox(self)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText("The robot has reached its destination!")
        msg_box.setWindowTitle("Goal Reached")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.buttonClicked.connect(self.on_ok_button_clicked)
        msg_box.exec_()

    def on_ok_button_clicked(self):
        self.switch_page_callback(1)
