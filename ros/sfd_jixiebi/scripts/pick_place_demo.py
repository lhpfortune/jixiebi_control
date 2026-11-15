#!/usr/bin/env python3
"""RViz pick-and-place demo driven by OpenCV colour detections."""

import math
import threading

import rospy
from geometry_msgs.msg import Point, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray

from sfd_jixiebi.demo_scene import (
    BIN_SIZE,
    BLOCK_SIZE,
    BLOCK_Z,
    COLOR_ORDER,
    COLOR_RGB,
    GRASP_TOOL_Z,
    LIFT_TOOL_Z,
    PICK_POSITIONS,
    PLACE_POSITIONS,
    cube_center_from_tool,
    solve_tool_ik,
)


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.010]
BLOCK_IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)

STATUS_TOPIC = "/sfd_jixiebi/demo_status"
SCENE_TOPIC = "/sfd_jixiebi/scene_markers"
DETECTION_TOPIC = "/sfd_jixiebi/detected_blocks"


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class PickPlaceDemo:
    def __init__(self):
        self.loop = as_bool(rospy.get_param("~loop", True))
        self.speed = max(0.25, float(rospy.get_param("~speed", 1.0)))
        self.wait_for_vision = as_bool(rospy.get_param("~wait_for_vision", True))
        self.frame_id = rospy.get_param("~frame_id", "base_link")

        self.lock = threading.RLock()
        self.q = list(HOME)
        self.last_status = ""
        self.detected_positions = {}
        self.detection_event = threading.Event()
        self.sequence_started = False
        self.attached_color = None
        self.placed_colors = set()
        self.blocks = {}

        self.reset_blocks()

        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.scene_pub = rospy.Publisher(SCENE_TOPIC, MarkerArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)
        self.detection_sub = rospy.Subscriber(
            DETECTION_TOPIC, PoseArray, self.detection_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / 20.0), self.publish_state)

        self.set_status("Waiting for OpenCV colour detections...")
        rospy.loginfo("Pick-place demo ready; waiting for %s", DETECTION_TOPIC)

    def reset_blocks(self):
        with self.lock:
            self.attached_color = None
            self.placed_colors.clear()
            self.blocks = {}
            for color in COLOR_ORDER:
                x, y = self.detected_positions.get(color, PICK_POSITIONS[color])
                self.blocks[color] = (x, y, BLOCK_Z) + BLOCK_IDENTITY_QUAT

    def detection_callback(self, message):
        if len(message.poses) != len(COLOR_ORDER):
            return
        with self.lock:
            if self.sequence_started:
                return
            positions = {}
            for color, pose in zip(COLOR_ORDER, message.poses):
                if pose.position.z < -100.0:
                    continue
                positions[color] = (pose.position.x, pose.position.y)
            if len(positions) != len(COLOR_ORDER):
                return
            self.detected_positions = positions
        if not self.detection_event.is_set():
            rospy.loginfo(
                "Demo received 4 OpenCV detections: %s",
                ", ".join(
                    "%s=(%.3f, %.3f)" % (color, positions[color][0], positions[color][1])
                    for color in COLOR_ORDER
                ),
            )
        self.detection_event.set()

    def set_status(self, text):
        if text == self.last_status:
            return
        self.last_status = text
        self.status_pub.publish(String(data=text))
        rospy.loginfo(text)

    def publish_state(self, _event=None):
        with self.lock:
            q = list(self.q)
            attached = self.attached_color
            blocks = dict(self.blocks)
            placed = set(self.placed_colors)

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = JOINT_NAMES + ["finger_joint_right"]
        joint_state.position = q + [-q[5]]
        joint_state.velocity = [0.0] * len(joint_state.name)
        joint_state.effort = [0.0] * len(joint_state.name)
        self.joint_pub.publish(joint_state)

        now = rospy.Time.now()
        markers = MarkerArray()
        # Coloured floor target outlines.  These are intentionally line
        # strips rather than flat cubes so a drop target cannot be mistaken
        # for a second block when viewed at an oblique RViz angle.
        for index, color in enumerate(COLOR_ORDER):
            x, y = PLACE_POSITIONS[color]
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.frame_id
            marker.ns = "drop_zones"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.0035
            marker.color.r, marker.color.g, marker.color.b = COLOR_RGB[color]
            marker.color.a = 0.95
            half = BIN_SIZE / 2.0
            for px, py in (
                (x - half, y - half),
                (x + half, y - half),
                (x + half, y + half),
                (x - half, y + half),
                (x - half, y - half),
            ):
                point = Point()
                point.x = px
                point.y = py
                point.z = 0.002
                marker.points.append(point)
            markers.markers.append(marker)

            # A small inner cross further distinguishes the target from a cube.
            cross = Marker()
            cross.header.stamp = now
            cross.header.frame_id = self.frame_id
            cross.ns = "drop_zone_cross"
            cross.id = index
            cross.type = Marker.LINE_LIST
            cross.action = Marker.ADD
            cross.pose.orientation.w = 1.0
            cross.scale.x = 0.0025
            cross.color.r, cross.color.g, cross.color.b = COLOR_RGB[color]
            cross.color.a = 0.70
            cross_half = BIN_SIZE * 0.18
            for ax, ay, bx, by in (
                (x - cross_half, y - cross_half, x + cross_half, y + cross_half),
                (x - cross_half, y + cross_half, x + cross_half, y - cross_half),
            ):
                for px, py in ((ax, ay), (bx, by)):
                    point = Point()
                    point.x = px
                    point.y = py
                    point.z = 0.003
                    cross.points.append(point)
            markers.markers.append(cross)

            label = Marker()
            label.header.stamp = now
            label.header.frame_id = self.frame_id
            label.ns = "drop_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.022
            label.pose.orientation.w = 1.0
            label.scale.z = 0.014
            label.color.r, label.color.g, label.color.b = COLOR_RGB[color]
            label.color.a = 0.95
            label.text = "%s TARGET" % color.upper()
            markers.markers.append(label)

        # Cubes. The held cube follows tool0 while the gripper is closed.
        tool_matrix = None
        if attached is not None:
            _, tool_matrix = cube_center_from_tool(q)

        for index, color in enumerate(COLOR_ORDER):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.frame_id
            marker.ns = "blocks"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            if attached == color and tool_matrix is not None:
                position, _ = cube_center_from_tool(q)
                marker.pose.position.x = float(position[0])
                marker.pose.position.y = float(position[1])
                marker.pose.position.z = float(position[2])
                quaternion = quaternion_from_matrix(tool_matrix)
                marker.pose.orientation.x = quaternion[0]
                marker.pose.orientation.y = quaternion[1]
                marker.pose.orientation.z = quaternion[2]
                marker.pose.orientation.w = quaternion[3]
            else:
                x, y, z, qx, qy, qz, qw = blocks[color]
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.pose.orientation.x = qx
                marker.pose.orientation.y = qy
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw
            marker.scale.x = BLOCK_SIZE
            marker.scale.y = BLOCK_SIZE
            marker.scale.z = BLOCK_SIZE
            marker.color.r, marker.color.g, marker.color.b = COLOR_RGB[color]
            marker.color.a = 1.0
            markers.markers.append(marker)

            if color in placed:
                done = Marker()
                done.header.stamp = now
                done.header.frame_id = self.frame_id
                done.ns = "placed"
                done.id = index
                done.type = Marker.TEXT_VIEW_FACING
                done.action = Marker.ADD
                done.pose.position.x, done.pose.position.y = PLACE_POSITIONS[color]
                done.pose.position.z = 0.065
                done.pose.orientation.w = 1.0
                done.scale.z = 0.018
                done.color.r = 1.0
                done.color.g = 1.0
                done.color.b = 1.0
                done.color.a = 1.0
                done.text = "OK"
                markers.markers.append(done)

        self.scene_pub.publish(markers)

    def move_joints(self, target, duration):
        target = list(target)
        with self.lock:
            start = list(self.q)

        if len(target) != len(JOINT_NAMES):
            raise ValueError("Expected six joint targets")

        # joint1 is continuous; take the short angular path.
        target[0] = start[0] + math.atan2(
            math.sin(target[0] - start[0]), math.cos(target[0] - start[0])
        )

        duration = max(0.1, duration / self.speed)
        steps = max(2, int(math.ceil(duration * 30.0)))
        interval = duration / float(steps)

        for step in range(1, steps + 1):
            if rospy.is_shutdown():
                return False
            blend = 0.5 - 0.5 * math.cos(math.pi * step / steps)
            q = [a + (b - a) * blend for a, b in zip(start, target)]
            with self.lock:
                self.q = q
            rospy.sleep(interval)
        return True

    def move_tool(self, x, y, z, duration, gripper):
        target = solve_tool_ik(x, y, z, pitch=math.pi, elbow=-1)
        if target is None:
            rospy.logerr("IK has no valid solution for (%.3f, %.3f, %.3f)", x, y, z)
            return False, None
        target[5] = gripper
        moved = self.move_joints(target, duration)
        return moved, target

    def set_gripper(self, value, duration):
        with self.lock:
            target = list(self.q)
        target[5] = max(0.0, min(0.010, value))
        return self.move_joints(target, duration)

    def pick_and_place(self, color):
        with self.lock:
            x, y = self.blocks[color][:2]
        target_x, target_y = PLACE_POSITIONS[color]

        self.set_status("%s: open gripper and approach block" % color.upper())
        self.set_gripper(0.010, 0.35)
        moved, _ = self.move_tool(x, y, LIFT_TOOL_Z, 2.0, 0.010)
        if not moved:
            return False
        moved, _ = self.move_tool(x, y, GRASP_TOOL_Z, 1.15, 0.010)
        if not moved:
            return False

        self.set_status("%s: close gripper" % color.upper())
        self.set_gripper(0.009, 0.55)
        with self.lock:
            self.attached_color = color
        rospy.sleep(0.25)

        self.set_status("%s: lift and transfer to %s target" % (color.upper(), color.upper()))
        moved, _ = self.move_tool(x, y, LIFT_TOOL_Z, 1.0, 0.009)
        if not moved:
            return False
        moved, _ = self.move_tool(target_x, target_y, LIFT_TOOL_Z, 2.2, 0.009)
        if not moved:
            return False
        moved, _ = self.move_tool(target_x, target_y, GRASP_TOOL_Z, 1.1, 0.009)
        if not moved:
            return False

        self.set_status("%s: release block" % color.upper())
        self.set_gripper(0.010, 0.55)
        with self.lock:
            released_q = list(self.q)
        _, released_tool_matrix = cube_center_from_tool(released_q)
        released_quaternion = quaternion_from_matrix(released_tool_matrix)
        with self.lock:
            self.attached_color = None
            # Keep the cube yaw it had between the fingers.  Resetting to
            # identity here caused the visible "snap upright" on release.
            self.blocks[color] = (
                target_x,
                target_y,
                BLOCK_Z,
                float(released_quaternion[0]),
                float(released_quaternion[1]),
                float(released_quaternion[2]),
                float(released_quaternion[3]),
            )
            self.placed_colors.add(color)
        rospy.sleep(0.2)

        moved, _ = self.move_tool(target_x, target_y, LIFT_TOOL_Z, 0.9, 0.010)
        return moved

    def run(self):
        rospy.sleep(0.5)
        if self.wait_for_vision:
            self.detection_event.wait(5.0)
            if not self.detection_event.is_set():
                rospy.logwarn(
                    "No complete OpenCV result after 5 s; using built-in block positions."
                )
        else:
            rospy.loginfo("Vision wait disabled; using built-in block positions.")

        while not rospy.is_shutdown():
            with self.lock:
                # Accept the latest latched OpenCV frame at the start of a
                # cycle, then freeze it while the arm is moving.
                self.sequence_started = False
            rospy.sleep(0.15)
            with self.lock:
                self.sequence_started = True
            self.reset_blocks()
            self.set_status("Returning arm to home pose")
            self.move_joints(HOME, 2.2)

            for color in COLOR_ORDER:
                if rospy.is_shutdown():
                    break
                if not self.pick_and_place(color):
                    rospy.logerr("Pick-place sequence stopped at %s", color)
                    return

            self.set_status("All four colours placed successfully")
            rospy.sleep(2.5)
            if not self.loop:
                rospy.loginfo("One-shot demo complete; keeping final state and markers published")
                rospy.spin()
                return

            self.set_status("Demo complete; resetting for the next cycle")
            self.move_joints(HOME, 2.0)
            rospy.sleep(0.8)


def main():
    rospy.init_node("sfd_pick_place_demo")
    demo = PickPlaceDemo()
    demo.run()


if __name__ == "__main__":
    main()