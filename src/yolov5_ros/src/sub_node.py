#!/usr/bin/env python3

import rospy
from detection_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from collections import defaultdict  # Add this import

class DetectionSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("detection_subscriber", anonymous=True)

        # Subscribe to the bounding boxes topic
        self.bbox_sub = rospy.Subscriber(
            rospy.get_param("~input_bounding_boxes_topic", "/yolov5/detections"),
            BoundingBoxes,
            self.bbox_callback
        )

        # Subscribe to the detected objects topic (optional)
        self.objects_sub = rospy.Subscriber(
            rospy.get_param("~input_detected_objects_topic", "/detected_objects"),
            String,
            self.objects_callback
        )

    def bbox_callback(self, msg):
        """
        Callback function for processing BoundingBoxes messages.
        Formats the data into a receipt-like structure with quantities.
        """
        rospy.loginfo("Received bounding boxes message.")

        # Initialize the receipt
        receipt = "MartBot Reciept: \n"
        receipt += "----------------------------------------\n"
        receipt += "       Name       |          Qty        \n"
        receipt += "----------------------------------------\n"

        # Count the occurrences of each class
        class_counts = defaultdict(int)
        for bbox in msg.bounding_boxes:
            class_name = bbox.Class
            class_counts[class_name] += 1

        # Format the receipt with class names and quantities
        for class_name, count in class_counts.items():
            receipt += f"{class_name:<30} {count}\n"

        # Add separator at the end
        receipt += "----------------------------------------\n"
        receipt += "THANK YOU!\n"
        receipt += "Glad to see you again!\n"

        # Print the formatted receipt
        rospy.loginfo("Formatted Receipt:\n" + receipt)

    def objects_callback(self, msg):
        """
        Callback function for processing detected objects (String messages).
        Formats the data into a simple receipt-like structure.
        """
        rospy.loginfo("Received detected objects message.")



if __name__ == "__main__":
    try:
        # Start the subscriber node
        subscriber = DetectionSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Node terminated.")