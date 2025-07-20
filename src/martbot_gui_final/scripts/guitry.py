#!/usr/bin/env python3
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QMessageBox, QVBoxLayout, QWidget,QToolButton
from PyQt5.QtGui import QIcon
import rospy
from geometry_msgs.msg import PoseStamped
import sys
# import interface_fun
import main_window



if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = main_window.MainWindow()
    # main_window = try3.RobotNavigationApp()
    main_window.show()
    try:
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
