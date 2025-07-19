#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import UInt8
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout

class LEDController:
    def __init__(self):
        # rospy.init_node('led_controller')
        self.pub = rospy.Publisher('led_color', UInt8, queue_size=10)
        rospy.loginfo("LED Controller Ready")
        rospy.loginfo("Publish to /led_color with:")
        rospy.loginfo("0: OFF, 1: RED, 2: GREEN")

    def set_color(self, color_code):
        """Set LED color (0-3)"""
        if color_code in [0, 1, 2, 3]:
            msg = UInt8()
            msg.data = color_code
            self.pub.publish(msg)
            rospy.loginfo(f"Set LEDs to: {self.color_name(color_code)}")
        else:
            rospy.logerr(f"Invalid color code: {color_code}. Use 0-3")

    def color_name(self, code):
        names = ["OFF", "RED", "GREEN"]
        return names[code] if 0 <= code <= 3 else "UNKNOWN"


# class LEDControlGUI(QWidget):
#     def __init__(self, controller):
#         super().__init__()

#         self.controller = controller

#         self.init_ui()

#     def init_ui(self):
#         self.setWindowTitle('LED Controller')

#         # Create a button
#         self.button = QPushButton('Turn On LED', self)
#         self.button.clicked.connect(self.on_button_click)

#         # Set up the layout
#         layout = QVBoxLayout()
#         layout.addWidget(self.button)

#         self.setLayout(layout)

#     def on_button_click(self):
#         # Set the LED to Red (1) when the button is pressed
#         self.controller.set_color(1)

# if __name__ == '__main__':
#     try:
#         # Start the LED controller
#         controller = LEDController()

#         # Start the PyQt5 application
#         app = QApplication(sys.argv)

#         # Create the GUI
#         window = LEDControlGUI(controller)
#         window.show()

#         # Start the ROS spin in a separate thread
#         # This allows PyQt5 and ROS to run concurrently
#         rospy.spin()
#         app.exec_()

#     except rospy.ROSInterruptException:
#         pass
