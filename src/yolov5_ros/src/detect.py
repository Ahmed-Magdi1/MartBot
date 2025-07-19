#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
from collections import defaultdict

from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes

# Add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# Import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        # Load parameters
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        self.merge_distance = rospy.get_param("~merge_distance", 0)  # New parameter

        # Initialize weights
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device", "")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h", 480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half precision
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup

        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking=True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )

        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def merge_boxes(self, detections, distance_threshold):
        """Merge nearby bounding boxes based on distance threshold and average confidence."""
        if len(detections) == 0:
            return detections

        n = len(detections)
        parent = list(range(n))

        def find(u):
            while parent[u] != u:
                parent[u] = parent[parent[u]]  # Path compression
                u = parent[u]
            return u

        def union(u, v):
            pu, pv = find(u), find(v)
            if pu != pv:
                parent[pv] = pu

        # Check pairwise gaps between boxes
        for i in range(n):
            for j in range(i + 1, n):
                box1, box2 = detections[i], detections[j]

                # Only merge boxes of the same class
                if box1[5] != box2[5]:
                    continue

                # Horizontal gap calculation
                if box1[2] < box2[0]:
                    h_gap = box2[0] - box1[2]
                elif box2[2] < box1[0]:
                    h_gap = box1[0] - box2[2]
                else:
                    h_gap = -1  # Overlap or adjacent

                # Vertical gap calculation
                if box1[3] < box2[1]:
                    v_gap = box2[1] - box1[3]
                elif box2[3] < box1[1]:
                    v_gap = box1[1] - box2[3]
                else:
                    v_gap = -1  # Overlap or adjacent

                # Merge if gaps within threshold
                if (h_gap <= distance_threshold or h_gap < 0) and (v_gap <= distance_threshold or v_gap < 0):
                    union(i, j)

        # Group boxes into clusters
        clusters = defaultdict(list)
        for idx in range(n):
            clusters[find(idx)].append(idx)

        # Create merged boxes
        merged_detections = []
        for indices in clusters.values():
            cluster_boxes = [detections[i] for i in indices]
            x1 = min(box[0] for box in cluster_boxes)
            y1 = min(box[1] for box in cluster_boxes)
            x2 = max(box[2] for box in cluster_boxes)
            y2 = max(box[3] for box in cluster_boxes)
            avg_conf = sum(box[4] for box in cluster_boxes) / len(cluster_boxes)
            cls = cluster_boxes[0][5]  # All boxes in the cluster have the same class
            merged_detections.append([x1, y1, x2, y2, avg_conf, cls])

        return np.array(merged_detections) if merged_detections else np.empty((0, 6))

    def callback(self, data):
        """Adapted from yolov5/detect.py"""
        try:
            if self.compressed_input:
                im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
            else:
                im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            im, im0 = self.preprocess(im)

            # Run inference
            im = torch.from_numpy(im).to(self.device)
            im = im.half() if self.half else im.float()
            im /= 255
            if len(im.shape) == 3:
                im = im[None]

            pred = self.model(im, augment=False, visualize=False)
            pred = non_max_suppression(
                pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
            )

            # Process predictions
            det = pred[0].cpu().numpy()

            # Apply box merging
            if self.merge_distance > 0:
                det = self.merge_boxes(det, self.merge_distance)

            bounding_boxes = BoundingBoxes()
            bounding_boxes.header = data.header
            bounding_boxes.image_header = data.header

            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    bounding_box = BoundingBox()
                    c = int(cls)
                    # Fill in bounding box message
                    bounding_box.Class = self.names[c]
                    bounding_box.probability = float(conf)
                    bounding_box.xmin = int(xyxy[0])
                    bounding_box.ymin = int(xyxy[1])
                    bounding_box.xmax = int(xyxy[2])
                    bounding_box.ymax = int(xyxy[3])

                    bounding_boxes.bounding_boxes.append(bounding_box)

                    # Annotate the image
                    if self.publish_image or self.view_image:
                        label = f"{self.names[c]} {conf:.2f}"
                        annotator.box_label(xyxy, label, color=colors(c, True))

                # Stream results
                im0 = annotator.result()

            # Publish prediction
            self.pred_pub.publish(bounding_boxes)

            # Publish & visualize images
            if self.view_image:
                cv2.imshow(str(0), im0)
                cv2.waitKey(1)  # 1 millisecond
            if self.publish_image:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))

        except Exception as e:
            rospy.logerr(f"Detection error: {e}")

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0


if __name__ == "__main__":
    check_requirements(exclude=("tensorboard", "thop"))
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    rospy.spin()
