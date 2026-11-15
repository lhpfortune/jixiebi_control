#!/usr/bin/env python3
"""Shared scene, kinematics and OpenCV helpers for the sfd pick-place demo."""

import math

import cv2
import numpy as np


COLOR_ORDER = ("red", "green", "yellow", "blue")

# RGB values are used by RViz; BGR values are used by the synthetic camera.
COLOR_RGB = {
    "red": (0.92, 0.08, 0.08),
    "green": (0.08, 0.78, 0.16),
    "yellow": (1.00, 0.82, 0.05),
    "blue": (0.08, 0.28, 0.95),
}
COLOR_BGR = {
    "red": (0, 0, 235),
    "green": (40, 200, 20),
    "yellow": (0, 210, 255),
    "blue": (235, 70, 0),
}

BLOCK_SIZE = 0.030
BIN_SIZE = 0.065
TABLE_Z = 0.0
BLOCK_Z = BLOCK_SIZE / 2.0
GRASP_TOOL_Z = 0.035
LIFT_TOOL_Z = 0.080
ATTACH_OFFSET_Z = 0.020

# Workspace used by the overhead synthetic camera.
WORKSPACE_MIN = -0.28
WORKSPACE_MAX = 0.28
CAMERA_SIZE = 640

# Robot dimensions taken from urdf/sfd_arm.xacro.
BASE_TO_SHOULDER_Z = 0.03 + 0.07
LINK2_LENGTH = 0.16
LINK3_LENGTH = 0.14
LINK4_TO_TOOL = 0.08 + 0.052 + 0.096
SHOULDER_Z = BASE_TO_SHOULDER_Z

JOINT_LIMITS = {
    "joint2": (-1.5708, 1.5708),
    "joint3": (-2.35619449, 2.35619449),
    "joint4": (-2.35619449, 2.35619449),
    "joint6": (0.0, 0.010),
}


def _polar(radius, degrees):
    angle = math.radians(degrees)
    return radius * math.cos(angle), radius * math.sin(angle)


# The objects are deliberately close to the base: with the long wrist/tool
# chain in this URDF, a fully top-down grasp is reachable at these offsets.
PICK_POSITIONS = {
    "red": _polar(0.165, 60.0),
    "green": _polar(0.185, 135.0),
    "yellow": _polar(0.185, 225.0),
    "blue": _polar(0.165, 300.0),
}
PLACE_POSITIONS = {
    "red": _polar(0.215, 45.0),
    "green": _polar(0.215, 150.0),
    "yellow": _polar(0.215, 210.0),
    "blue": _polar(0.215, 315.0),
}


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _translation_z(distance):
    matrix = np.eye(4)
    matrix[2, 3] = distance
    return matrix


def _rotation_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, -s, 0.0],
            [0.0, s, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def _rotation_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [
            [c, -s, 0.0, 0.0],
            [s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def forward_tool_matrix(joints):
    """Return base_link -> tool0 for [joint1..joint6]."""
    q1, q2, q3, q4, q5, _q6 = joints
    transform = _translation_z(0.03)
    transform = transform.dot(_rotation_z(q1))
    transform = transform.dot(_translation_z(0.07))
    transform = transform.dot(_rotation_x(q2))
    transform = transform.dot(_translation_z(LINK2_LENGTH))
    transform = transform.dot(_rotation_x(q3))
    transform = transform.dot(_translation_z(LINK3_LENGTH))
    transform = transform.dot(_rotation_x(q4))
    transform = transform.dot(_translation_z(0.08))
    transform = transform.dot(_rotation_z(q5))
    transform = transform.dot(_translation_z(0.052 + 0.096))
    return transform


def forward_tool_position(joints):
    return forward_tool_matrix(joints)[:3, 3].copy()


def cube_center_from_tool(joints):
    """Cube centre while it is held between the fingers."""
    tool = forward_tool_matrix(joints)
    point = tool.dot(np.array([0.0, 0.0, ATTACH_OFFSET_Z, 1.0]))
    return point[:3].copy(), tool


def solve_tool_ik(x, y, z, pitch=math.pi, elbow=-1):
    """Analytic IK for a top-down tool pose, with q5 fixed to zero.

    pitch is the total pitch of joint2+joint3+joint4 measured from +Z.
    The default pitch=pi points the tool axis down.  elbow=+1 selects the
    solution branch used by this demonstration.
    """
    radius = math.hypot(x, y)
    if radius < 1.0e-9:
        azimuth = 0.0
    else:
        azimuth = math.atan2(y, x)

    q1 = wrap_angle(azimuth - math.pi / 2.0)

    # Position of joint4 (the start of the long wrist/tool link).
    wrist_x = radius + LINK4_TO_TOOL * math.sin(pitch)
    wrist_z = z - LINK4_TO_TOOL * math.cos(pitch) - SHOULDER_Z
    distance = math.hypot(wrist_x, wrist_z)
    maximum = LINK2_LENGTH + LINK3_LENGTH

    if distance > maximum + 1.0e-6 or distance < 1.0e-6:
        return None

    cosine = (
        distance * distance - LINK2_LENGTH * LINK2_LENGTH - LINK3_LENGTH * LINK3_LENGTH
    ) / (2.0 * LINK2_LENGTH * LINK3_LENGTH)
    if cosine < -1.0 - 1.0e-6 or cosine > 1.0 + 1.0e-6:
        return None
    cosine = max(-1.0, min(1.0, cosine))

    beta = math.atan2(wrist_x, wrist_z)
    alpha_cosine = (
        distance * distance + LINK2_LENGTH * LINK2_LENGTH - LINK3_LENGTH * LINK3_LENGTH
    ) / (2.0 * distance * LINK2_LENGTH)
    alpha = math.acos(max(-1.0, min(1.0, alpha_cosine)))
    delta = math.acos(cosine)
    sign = 1.0 if elbow >= 0 else -1.0

    gamma1 = beta + sign * alpha
    gamma2 = gamma1 - sign * delta

    q2 = -gamma1
    q3 = gamma1 - gamma2
    q4 = wrap_angle(pitch - q2 - q3)
    q5 = 0.0

    joints = [q1, q2, q3, q4, q5, 0.01]
    for name, value in zip(("joint2", "joint3", "joint4"), (q2, q3, q4)):
        lower, upper = JOINT_LIMITS[name]
        if value < lower - 1.0e-5 or value > upper + 1.0e-5:
            return None

    # Reject a numerically invalid solution and verify the requested point.
    actual = forward_tool_position(joints)
    if math.sqrt((actual[0] - x) ** 2 + (actual[1] - y) ** 2 + (actual[2] - z) ** 2) > 1.0e-4:
        return None
    return joints


def world_to_pixel(x, y, size=CAMERA_SIZE):
    scale = size / (WORKSPACE_MAX - WORKSPACE_MIN)
    u = int(round((x - WORKSPACE_MIN) * scale))
    v = int(round((WORKSPACE_MAX - y) * scale))
    return u, v


def pixel_to_world(u, v, size=CAMERA_SIZE):
    scale = (WORKSPACE_MAX - WORKSPACE_MIN) / float(size)
    x = WORKSPACE_MIN + u * scale
    y = WORKSPACE_MAX - v * scale
    return x, y


def synthetic_workspace_image(size=CAMERA_SIZE):
    """Render the top-down block scene consumed by the OpenCV detector."""
    image = np.full((size, size, 3), (238, 238, 238), dtype=np.uint8)

    # Metric grid, 0.05 m spacing.
    step = int(round(0.05 * size / (WORKSPACE_MAX - WORKSPACE_MIN)))
    for pixel in range(0, size, step):
        cv2.line(image, (pixel, 0), (pixel, size - 1), (220, 220, 220), 1)
        cv2.line(image, (0, pixel), (size - 1, pixel), (220, 220, 220), 1)

    # Approximate robot base footprint for orientation context.
    centre = world_to_pixel(0.0, 0.0, size)
    radius = int(round(0.065 * size / (WORKSPACE_MAX - WORKSPACE_MIN)))
    cv2.circle(image, centre, radius, (115, 125, 120), -1)
    cv2.circle(image, centre, radius, (75, 85, 80), 2)

    half = BLOCK_SIZE / 2.0
    for color in COLOR_ORDER:
        x, y = PICK_POSITIONS[color]
        u0, v0 = world_to_pixel(x - half, y + half, size)
        u1, v1 = world_to_pixel(x + half, y - half, size)
        cv2.rectangle(image, (u0, v0), (u1, v1), COLOR_BGR[color], -1)
        cv2.rectangle(image, (u0, v0), (u1, v1), (45, 45, 45), 1)
        cv2.putText(
            image,
            color.upper(),
            (u0, max(18, v0 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (45, 45, 45),
            1,
            cv2.LINE_AA,
        )

    # A small amount of deterministic noise/blur makes this exercise the
    # normal OpenCV filtering path rather than exact-pixel colour matching.
    image = cv2.GaussianBlur(image, (3, 3), 0.6)
    noise = np.random.default_rng(7).normal(0.0, 1.4, image.shape)
    image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return image


HSV_THRESHOLDS = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((40, 95, 55), (88, 255, 255))],
    "yellow": [((19, 95, 90), (36, 255, 255))],
    "blue": [((94, 110, 55), (136, 255, 255))],
}


def detect_colored_blocks(image):
    """Detect red/green/yellow/blue squares and return metric centres."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    detections = {}

    for color in COLOR_ORDER:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in HSV_THRESHOLDS[color]:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)),
            )

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < 80.0:
            continue

        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1.0e-9:
            continue
        u = moments["m10"] / moments["m00"]
        v = moments["m01"] / moments["m00"]
        x, y = pixel_to_world(u, v, image.shape[1])
        x0, y0, width, height = cv2.boundingRect(contour)
        detections[color] = {
            "x": x,
            "y": y,
            "area": area,
            "bbox": (x0, y0, width, height),
        }

    return detections