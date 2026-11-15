#!/usr/bin/env python3
"""OpenCV HSV colour detector for the RViz pick-place scene."""

import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from sfd_jixiebi.demo_scene import (
    BLOCK_SIZE,
    COLOR_ORDER,
    COLOR_RGB,
    detect_colored_blocks,
    synthetic_workspace_image,
)


class ColorDetector:
    def __init__(self):
        self.source = rospy.get_param("~source", "synthetic")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.rate = float(rospy.get_param("~rate", 5.0))
        self.bridge = CvBridge()
        self.announced = False

        self.pose_pub = rospy.Publisher(
            "/sfd_jixiebi/detected_blocks", PoseArray, queue_size=1, latch=True
        )
        self.image_pub = rospy.Publisher(
            "/sfd_jixiebi/vision/debug_image", Image, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/sfd_jixiebi/vision_markers", MarkerArray, queue_size=1
        )

        if self.source == "camera":
            self.image_sub = rospy.Subscriber(
                self.image_topic, Image, self.image_callback, queue_size=1
            )
            rospy.loginfo("OpenCV detector waiting for camera image: %s", self.image_topic)
        else:
            self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate, 0.5)), self.timer_callback)
            rospy.loginfo("OpenCV detector using the synthetic overhead workspace image")

    def timer_callback(self, _event):
        image = synthetic_workspace_image()
        self.process(image, rospy.Time.now())

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge raises several message types
            rospy.logerr_throttle(2.0, "Cannot convert camera image: %s", exc)
            return
        stamp = message.header.stamp if not message.header.stamp.is_zero() else rospy.Time.now()
        self.process(image, stamp)

    def process(self, image, stamp):
        detections = detect_colored_blocks(image)
        annotated = self.annotate(image, detections)

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.frame_id
        for color in COLOR_ORDER:
            pose = Pose()
            detection = detections.get(color)
            if detection is None:
                pose.position.z = -999.0
            else:
                pose.position.x = detection["x"]
                pose.position.y = detection["y"]
                pose.position.z = 0.0
                pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(self.make_markers(detections, stamp))

        image_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        image_message.header.stamp = stamp
        image_message.header.frame_id = self.frame_id
        self.image_pub.publish(image_message)

        if not self.announced and len(detections) == len(COLOR_ORDER):
            self.announced = True
            rospy.loginfo(
                "OpenCV detected 4/4 blocks: %s",
                ", ".join(
                    "%s=(%.3f, %.3f)" % (color, detections[color]["x"], detections[color]["y"])
                    for color in COLOR_ORDER
                ),
            )

    @staticmethod
    def annotate(image, detections):
        output = image.copy()
        for color, detection in detections.items():
            x0, y0, width, height = detection["bbox"]
            bgr = tuple(int(round(255.0 * value)) for value in COLOR_RGB[color])
            cv2.rectangle(output, (x0 - 4, y0 - 4), (x0 + width + 4, y0 + height + 4), (255, 255, 255), 3)
            cv2.rectangle(output, (x0, y0), (x0 + width, y0 + height), bgr, 2)
            text = "%s  (%.3f, %.3f)" % (color.upper(), detection["x"], detection["y"])
            cv2.putText(
                output,
                text,
                (max(4, x0 - 4), min(output.shape[0] - 8, y0 + height + 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (25, 25, 25),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                output,
                text,
                (max(4, x0 - 4), min(output.shape[0] - 8, y0 + height + 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (245, 245, 245),
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            output,
            "OpenCV HSV color recognition",
            (18, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (35, 35, 35),
            2,
            cv2.LINE_AA,
        )
        return output

    def make_markers(self, detections, stamp):
        markers = MarkerArray()
        half = BLOCK_SIZE * 0.70
        for index, color in enumerate(COLOR_ORDER):
            detection = detections.get(color)
            if detection is None:
                continue
            x = detection["x"]
            y = detection["y"]

            outline = Marker()
            outline.header.stamp = stamp
            outline.header.frame_id = self.frame_id
            outline.ns = "opencv_detections"
            outline.id = index * 2
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.scale.x = 0.0035
            outline.color.r, outline.color.g, outline.color.b = COLOR_RGB[color]
            outline.color.a = 0.95
            corners = (
                (x - half, y - half),
                (x + half, y - half),
                (x + half, y + half),
                (x - half, y + half),
                (x - half, y - half),
            )
            for px, py in corners:
                point = Point()
                point.x = px
                point.y = py
                point.z = 0.004
                outline.points.append(point)
            markers.markers.append(outline)

            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = self.frame_id
            label.ns = "opencv_labels"
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.055
            label.pose.orientation.w = 1.0
            label.scale.z = 0.018
            label.color.r, label.color.g, label.color.b = COLOR_RGB[color]
            label.color.a = 1.0
            label.text = "%s DETECTED" % color.upper()
            markers.markers.append(label)

        return markers


def main():
    rospy.init_node("sfd_color_vision")
    ColorDetector()
    rospy.spin()


if __name__ == "__main__":
    main()