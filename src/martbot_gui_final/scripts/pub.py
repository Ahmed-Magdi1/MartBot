#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8

def publish_led_command(color_code):
    """
    Publish LED command (0-3)
    0: OFF, 1: RED, 2: GREEN, 3: YELLOW
    """
    rospy.init_node('led_publisher_node', anonymous=True)
    pub = rospy.Publisher('/led_color', UInt8, queue_size=10)
    
    # Wait for publisher to connect
    rospy.sleep(0.1)
    
    # Create and publish message
    msg = UInt8()
    msg.data = color_code
    pub.publish(msg)
    rospy.loginfo(f"Published LED command: {color_code}")

# if __name__ == '__main__':
#     try:
#         # Example usage:
#         publish_led_command(1)  # Turn RED on
#         rospy.sleep(2.0)
#         publish_led_command(2)  # Turn GREEN on
#         rospy.sleep(2.0)
#         publish_led_command(0)  # Turn OFF
#     except rospy.ROSInterruptException:
#         pass
