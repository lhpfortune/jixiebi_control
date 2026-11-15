"""Gesture-driven P4 sorting controller for the 5-DoF robot arm.

Run this file on the robot/Jupyter host, not on the PC. The PC gesture
application sends JSON commands to http://<robot-ip>:8765/command.
"""

from __future__ import annotations

import csv
import hmac
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd()
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

HOST = "0.0.0.0"
PORT = int(os.environ.get("ROBOT_GESTURE_PORT", "8765"))
CONTROL_TOKEN = os.environ.get("ROBOT_CONTROL_TOKEN", "")
MOVE_ON_START = os.environ.get("ROBOT_MOVE_ON_START", "1").lower() not in {"0", "false", "no"}
ALLOW_MOTION = os.environ.get("ROBOT_ALLOW_MOTION", "1").lower() not in {"0", "false", "no"}

CAMERA_ID = 0
ROI = (160, 100, 320, 300)
P4_PIXEL_CENTER = (346.0, 303.0)
P4_MAX_PIXEL_DISTANCE = 110.0
MIN_AREA = 800
MAX_AREA = 120000
MIN_WIDTH = 25
MAX_WIDTH = 320
MIN_HEIGHT = 25
MAX_HEIGHT = 320
MIN_FILL = 0.55
VISION_PERIOD_S = 0.08
SELECT_WAIT_S = 2.0

P4_WORKSPACE_XY = (0.0, 0.0)
P4_PICK_Z = 10.0
P4_EXIT_LIFT_DZ = 80.0
HIGH_HOME_Z = 225.0
DIRECT_MOVE_SECONDS = 1.5
DROP_SETTLE_S = 1.0
HOME_POSE_NAME = "home"
CAPTURE_POSE_NAME = "capture_image"

COLOR_ORDER = ["red", "yellow", "green", "blue"]
COLOR_CN = {"red": "红色", "yellow": "黄色", "green": "绿色", "blue": "蓝色", "none": "未识别"}
DESTINATIONS = {
    "yellow": {"point": "P9", "pose": "object_yellow"},
    "green": {"point": "P10", "pose": "object_green"},
    "blue": {"point": "P11", "pose": "object_blue"},
    "red": {"point": "P12", "pose": "object_red"},
}
DEFAULT_HSV = {
    "red": ((0, 70, 50), (10, 255, 255)),
    "yellow": ((26, 70, 50), (34, 255, 255)),
    "green": ((35, 70, 50), (77, 255, 255)),
    "blue": ((100, 70, 50), (124, 255, 255)),
}

STATE_READY = "READY"
STATE_ARMED = "ARMED"
STATE_EXECUTING = "EXECUTING"
STATE_FAULT = "FAULT"
STATE_ESTOP = "ESTOP"
STATE_STOPPED = "STOPPED"


def _load_hsv_params():
    path = SCRIPT_DIR / "hsv_config.json"
    if not path.is_file():
        return {color: (tuple(low), tuple(high)) for color, (low, high) in DEFAULT_HSV.items()}
    with path.open("r", encoding="utf-8") as handle:
        saved = json.load(handle)
    return {color: (tuple(saved[color][0]), tuple(saved[color][1])) for color in COLOR_ORDER}


def _to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def _build_mask(hsv, color, params):
    lower, upper = params[color]
    lower = np.array(lower, dtype=np.uint8)
    upper = np.array(upper, dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    if color == "red" and int(lower[0]) <= 10:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(
                hsv,
                np.array([170, int(lower[1]), int(lower[2])], dtype=np.uint8),
                np.array([180, int(upper[1]), int(upper[2])], dtype=np.uint8),
            ),
        )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return mask


def _find_blocks(mask, offset=(0, 0)):
    ox, oy = offset
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    blocks = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < MIN_AREA or area > MAX_AREA:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w < MIN_WIDTH or w > MAX_WIDTH or h < MIN_HEIGHT or h > MAX_HEIGHT:
            continue
        fill = area / float(max(1, w * h))
        if fill < MIN_FILL:
            continue
        blocks.append(
            {
                "rect": (x + ox, y + oy, w, h),
                "center": (x + ox + w // 2, y + oy + h // 2),
                "area": area,
                "fill": round(fill, 3),
            }
        )
    blocks.sort(key=lambda item: item["area"], reverse=True)
    return blocks


class P4ColorRecognizer:
    def __init__(self, params):
        self.params = params

    def classify(self, image_bgr):
        x, y, w, h = ROI
        roi_hsv = _to_hsv(image_bgr[y : y + h, x : x + w])
        candidates = []
        for color in COLOR_ORDER:
            for block in _find_blocks(_build_mask(roi_hsv, color, self.params), offset=(x, y)):
                distance = math.hypot(
                    block["center"][0] - P4_PIXEL_CENTER[0],
                    block["center"][1] - P4_PIXEL_CENTER[1],
                )
                if distance <= P4_MAX_PIXEL_DISTANCE:
                    item = dict(block)
                    item["color"] = color
                    item["distance_to_p4"] = round(distance, 1)
                    candidates.append(item)
        if not candidates:
            return "none", None
        best = max(candidates, key=lambda item: item["area"])
        return best["color"], best


class GestureRobotController:
    """Manual, confirm-before-motion controller.

    ONE  SELECT_GRAB : detect the color at P4 and arm one task (no motion)
    TWO  SELECT_RESET: arm a capture-pose reset (no motion)
    OK   CONFIRM     : execute the armed task once
    OPEN_PALM CANCEL : clear the armed task
    FIST STOP        : immediate damping/emergency stop
    """

    def __init__(self, move_on_start=MOVE_ON_START, allow_motion=ALLOW_MOTION):
        self.move_on_start = bool(move_on_start)
        self.allow_motion = bool(allow_motion)
        self.camera = None
        self.arm = None
        self.arm_app = None
        self.robot_config = None
        self.recognizer = P4ColorRecognizer(_load_hsv_params())

        self._lock = threading.RLock()
        self._command_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._camera_lock = threading.Lock()
        self._job_queue = queue.Queue(maxsize=1)
        self._vision_thread = None
        self._worker_thread = None

        self.state = STATE_STOPPED
        self.detail = "尚未启动"
        self.selection = None
        self.last_color = "none"
        self.last_best = None
        self.last_frame_at = 0.0
        self.tasks_done = 0
        self.last_result = ""
        self.started_at = None
        self.log_path = SCRIPT_DIR / "gesture_robot_commands.csv"

    def _set_state(self, state, detail):
        with self._lock:
            old = self.state
            self.state = state
            self.detail = detail
        if old != state:
            print("[ROBOT] %s -> %s: %s" % (old, state, detail))

    def snapshot(self):
        with self._lock:
            selection = dict(self.selection) if self.selection else None
            return {
                "ok": True,
                "state": self.state,
                "detail": self.detail,
                "selected_action": selection.get("kind") if selection else None,
                "selected_color": selection.get("color") if selection else None,
                "last_color": self.last_color,
                "tasks_done": self.tasks_done,
                "last_result": self.last_result,
                "motion_enabled": self.allow_motion,
                "started_at": self.started_at,
            }

    def connect(self):
        from arm5dof_uservo import Arm5DoFUServo
        from arm_application import ArmApplication
        from camera import Camera
        import config as robot_config

        self.robot_config = robot_config
        required_poses = [CAPTURE_POSE_NAME, HOME_POSE_NAME]
        required_poses.extend(item["pose"] for item in DESTINATIONS.values())
        missing = [name for name in required_poses if name not in robot_config.ARM_POSE_DICT]
        if missing:
            raise RuntimeError("config.py 缺少位姿: %s" % ", ".join(missing))

        camera = None
        arm = None
        try:
            camera = Camera(id=CAMERA_ID, config_folder=str(SCRIPT_DIR / "vconfig"))
            if camera.capture is None or not camera.capture.isOpened():
                raise RuntimeError("摄像头未打开")
            camera.load_cam_calib_data()
            for _ in range(8):
                camera.read()

            arm = Arm5DoFUServo(config_folder=str(SCRIPT_DIR / "config"))
            arm_app = ArmApplication(arm, config_folder=str(SCRIPT_DIR / "config"))
        except BaseException as exc:
            if camera is not None:
                try:
                    camera.capture.release()
                except BaseException:
                    pass
            if arm is not None:
                try:
                    arm.set_damping(0)
                except BaseException:
                    pass
            raise RuntimeError("相机或机械臂连接失败: %s" % repr(exc)) from exc

        self.camera = camera
        self.arm = arm
        self.arm_app = arm_app
        self._preflight_transfer_paths()

        if self.move_on_start:
            if not self.allow_motion:
                raise RuntimeError("ROBOT_MOVE_ON_START=1 但 ROBOT_ALLOW_MOTION=0，配置冲突")
            result = self.arm.set_tool_pose(pose_name=CAPTURE_POSE_NAME, T=2.0)
            if result is False:
                raise RuntimeError("无法移动到 capture_image 拍摄位")
            time.sleep(0.4)
            print("[ROBOT] 机械臂已到达 capture_image 拍摄位")

        self._stop_event.clear()
        self.started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self._set_state(STATE_READY, "等待手势指令")
        self._vision_thread = threading.Thread(
            target=self._vision_loop, name="robot-vision", daemon=True
        )
        self._worker_thread = threading.Thread(
            target=self._worker_loop, name="robot-motion", daemon=True
        )
        self._vision_thread.start()
        self._worker_thread.start()
        return self.snapshot()

    def _fresh_color(self, timeout=SELECT_WAIT_S):
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            with self._lock:
                color = self.last_color
                fresh = time.monotonic() - self.last_frame_at <= 0.8
            if fresh and color != "none":
                return color
            if time.monotonic() >= deadline:
                return "none"
            time.sleep(0.08)

    def select_grab(self):
        with self._command_lock:
            with self._lock:
                if self.state not in (STATE_READY, STATE_ARMED):
                    return 409, "当前状态 %s 不允许选择抓取" % self.state
            color = self._fresh_color()
            if color == "none":
                return 409, "P4 未识别到稳定色块，请调整物块位置和光照"
            destination = DESTINATIONS[color]
            with self._lock:
                if self.state not in (STATE_READY, STATE_ARMED):
                    return 409, "选择过程中状态已变化，请重试"
                self.selection = {"kind": "GRAB", "color": color, "point": destination["point"]}
                self._set_state(
                    STATE_ARMED,
                    "已选择 P4 %s -> %s，等待 OK 确认"
                    % (COLOR_CN[color], destination["point"]),
                )
            return 200, "已选择 %s -> %s，请用 OK 确认" % (COLOR_CN[color], destination["point"])

    def select_reset(self):
        with self._command_lock:
            with self._lock:
                if self.state not in (STATE_READY, STATE_ARMED):
                    return 409, "当前状态 %s 不允许选择复位" % self.state
                self.selection = {"kind": "RESET", "color": None, "point": CAPTURE_POSE_NAME}
                self._set_state(STATE_ARMED, "已选择复位，等待 OK 确认")
            return 200, "已选择复位，请用 OK 确认"

    def confirm(self):
        with self._command_lock:
            with self._lock:
                if self.state != STATE_ARMED or not self.selection:
                    return 409, "没有待确认动作，请先做 ONE 或 TWO"
                selection = dict(self.selection)
                self._set_state(STATE_EXECUTING, "已确认，动作排队执行")
            try:
                self._job_queue.put_nowait(selection)
            except queue.Full:
                with self._lock:
                    self._set_state(STATE_ARMED, "动作队列忙，请稍后重试")
                return 409, "动作队列忙，请稍后重试"
            label = selection["kind"]
            if label == "GRAB":
                return 200, "已确认抓取 %s，开始执行" % COLOR_CN[selection["color"]]
            return 200, "已确认复位，开始执行"

    def cancel(self):
        with self._command_lock:
            with self._lock:
                if self.state == STATE_EXECUTING:
                    return 409, "机械臂正在执行，不能取消"
                if self.state not in (STATE_READY, STATE_ARMED):
                    return 409, "当前状态 %s 不允许取消" % self.state
                self.selection = None
                self._set_state(STATE_READY, "已取消待确认动作")
            return 200, "已取消待确认动作"

    def emergency_stop(self):
        self._stop_event.set()
        with self._lock:
            self.selection = None
            self._set_state(STATE_ESTOP, "收到 STOP，机械臂卸力")
        error = None
        try:
            if self.arm is not None:
                self.arm.set_damping(0)
        except BaseException as exc:
            error = repr(exc)
        if error:
            return 500, "急停已触发，但卸力命令报错: %s" % error
        return 200, "急停已触发，机械臂已卸力；请排除风险后重启控制端"

    def handle_command(self, command):
        if command == "SELECT_GRAB":
            return self.select_grab()
        if command == "SELECT_RESET":
            return self.select_reset()
        if command == "CONFIRM":
            return self.confirm()
        if command == "CANCEL":
            return self.cancel()
        if command == "STOP":
            return self.emergency_stop()
        return 400, "不支持的命令: %s" % command

    def _vision_loop(self):
        while not self._stop_event.is_set():
            try:
                with self._camera_lock:
                    ok, frame = self.camera.read()
                if not ok or frame is None:
                    raise RuntimeError("相机读帧失败")
                frame = cv2.resize(frame, (640, 480))
                color, best = self.recognizer.classify(frame)
                with self._lock:
                    self.last_color = color
                    self.last_best = best
                    self.last_frame_at = time.monotonic()
            except BaseException as exc:
                try:
                    if self.arm is not None:
                        self.arm.set_damping(0)
                except BaseException:
                    pass
                self._stop_event.set()
                self._set_state(STATE_FAULT, "视觉线程异常: %s" % repr(exc)[:120])
                return
            time.sleep(VISION_PERIOD_S)

    def _worker_loop(self):
        while not self._stop_event.is_set():
            try:
                selection = self._job_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if selection["kind"] == "GRAB":
                    self._execute_grab(selection["color"])
                else:
                    self._execute_reset()
                with self._lock:
                    self.selection = None
            except BaseException as exc:
                try:
                    self.arm.set_damping(0)
                except BaseException:
                    pass
                with self._lock:
                    self.last_result = "FAILED: %s" % repr(exc)[:160]
                    self.selection = None
                    if self.state != STATE_ESTOP:
                        self._set_state(
                            STATE_FAULT,
                            "动作失败，机械臂已卸力: %s" % repr(exc)[:100],
                        )
                self._append_log(
                    {
                        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "action": "GRAB" if selection["kind"] == "GRAB" else "RESET",
                        "color": selection.get("color") or "",
                        "target": selection.get("point") or "",
                        "result": "FAILED",
                        "detail": repr(exc)[:200],
                    }
                )
                self._stop_event.set()
                return

    def _execute_grab(self, color):
        if not self.allow_motion:
            raise RuntimeError("ROBOT_ALLOW_MOTION=0，拒绝真实动作")
        destination = DESTINATIONS[color]
        target_pose = self.robot_config.ARM_POSE_DICT[destination["pose"]]
        target_x, target_y = float(target_pose[0]), float(target_pose[1])
        pick_wz = float(P4_PICK_Z) + float(self.arm_app.GRAB_CUBE_DZ)
        lift_wz = pick_wz + max(20.0, float(P4_EXIT_LIFT_DZ))

        print("[ROBOT] 执行 P4 %s -> %s" % (COLOR_CN[color], destination["point"]))
        self.arm_app.gripper_open()
        arm_x, arm_y, _ = self.arm_app.tf_ws2arm(
            P4_WORKSPACE_XY[0], P4_WORKSPACE_XY[1], pick_wz
        )
        self.arm.set_joint_angle_soft("joint1", math.atan2(arm_y, arm_x), T=0.8)
        self.arm_app.move2ws(P4_WORKSPACE_XY[0], P4_WORKSPACE_XY[1], lift_wz, t=0.8)
        self.arm_app.move2ws(P4_WORKSPACE_XY[0], P4_WORKSPACE_XY[1], pick_wz, t=0.45)
        self.arm_app.gripper_close()
        time.sleep(0.25)
        self.arm_app.move2ws(P4_WORKSPACE_XY[0], P4_WORKSPACE_XY[1], lift_wz, t=0.65)

        self._move_to_high_home()
        self._move_direct_overhead(target_x, target_y)
        self.arm_app.gripper_open()
        time.sleep(DROP_SETTLE_S)
        self._move_to_high_home()
        self.arm_app.gripper_close()
        result = self.arm.set_tool_pose(pose_name=CAPTURE_POSE_NAME, T=1.5)
        if result is False:
            raise RuntimeError("返回 capture_image 失败")
        if self._stop_event.is_set():
            raise RuntimeError("控制端已停止")

        with self._lock:
            self.tasks_done += 1
            self.last_result = "OK: %s -> %s" % (color, destination["point"])
            self._set_state(STATE_READY, "抓取完成，等待下一次手势")
        self._append_log(
            {
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "action": "GRAB",
                "color": color,
                "target": destination["point"],
                "result": "OK",
                "detail": "P4 -> high home -> bin -> capture_image",
            }
        )

    def _execute_reset(self):
        if not self.allow_motion:
            raise RuntimeError("ROBOT_ALLOW_MOTION=0，拒绝真实动作")
        print("[ROBOT] 执行复位到 capture_image")
        self.arm_app.gripper_close()
        result = self.arm.set_tool_pose(pose_name=CAPTURE_POSE_NAME, T=2.0)
        if result is False:
            raise RuntimeError("复位到 capture_image 失败")
        if self._stop_event.is_set():
            raise RuntimeError("控制端已停止")
        with self._lock:
            self.last_result = "OK: RESET"
            self._set_state(STATE_READY, "复位完成，等待下一次手势")
        self._append_log(
            {
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "action": "RESET",
                "color": "",
                "target": CAPTURE_POSE_NAME,
                "result": "OK",
                "detail": "capture_image",
            }
        )

    def _ik_solution(self, x, y, z, last_joint_angle=None):
        tool_posi = self.arm.adjust_tool_posi([float(x), float(y), float(z)])
        pitch = self.arm.auto_gen_pitch(tool_posi)
        if hasattr(self.arm, "kinematic"):
            solutions = self.arm.kinematic.inverse_kinematic(
                tool_posi[0], tool_posi[1], tool_posi[2], pitch, math.pi,
                last_joint_angle=last_joint_angle,
            )
            if solutions:
                return True, pitch, solutions[-1]
            return False, pitch, None
        ok, solution = self.arm.inverse_kinematic(tool_posi, pitch, math.pi)
        return bool(ok), pitch, solution

    def _preflight_transfer_paths(self):
        errors = []
        home_x, home_y = self.robot_config.ARM_POSE_DICT[HOME_POSE_NAME][:2]
        current_joints = None
        try:
            current_joints = self.arm.get_joint_angle_list()[:5]
        except BaseException:
            pass
        ok, pitch, home_solution = self._ik_solution(home_x, home_y, HIGH_HOME_Z, current_joints)
        print("[ROBOT] 高位 home 校验: %s pitch=%.1f" % ("OK" if ok else "不可达", math.degrees(pitch)))
        if not ok:
            errors.append("high_home")
        for color in COLOR_ORDER:
            destination = DESTINATIONS[color]
            pose = self.robot_config.ARM_POSE_DICT[destination["pose"]]
            target_ok, target_pitch, _ = self._ik_solution(
                float(pose[0]), float(pose[1]), HIGH_HOME_Z, home_solution
            )
            print(
                "[ROBOT] %s %s 上空校验: %s pitch=%.1f"
                % (destination["point"], COLOR_CN[color], "OK" if target_ok else "不可达", math.degrees(target_pitch))
            )
            if not target_ok:
                errors.append("%s(%s)" % (destination["point"], color))
        if errors:
            raise RuntimeError("高位路径不可达: %s" % ", ".join(errors))

    def _move_to_high_home(self):
        home_x, home_y = self.robot_config.ARM_POSE_DICT[HOME_POSE_NAME][:2]
        result = self.arm.set_tool_pose(
            [float(home_x), float(home_y), HIGH_HOME_Z], tool_roll=math.pi, T=DIRECT_MOVE_SECONDS
        )
        if result is False:
            raise RuntimeError("移动到高位 home 失败")

    def _move_direct_overhead(self, target_x, target_y):
        result = self.arm.set_tool_pose(
            [float(target_x), float(target_y), HIGH_HOME_Z], tool_roll=math.pi, T=DIRECT_MOVE_SECONDS
        )
        if result is False:
            raise RuntimeError("直接移动到垃圾桶上空失败")

    def _append_log(self, row):
        fields = ["time", "action", "color", "target", "result", "detail"]
        new_file = not self.log_path.is_file()
        try:
            with self.log_path.open("a", encoding="utf-8-sig", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                if new_file:
                    writer.writeheader()
                writer.writerow(row)
        except BaseException as exc:
            print("[ROBOT][WARN] 日志写入失败: %s" % repr(exc))

    def shutdown(self):
        self._stop_event.set()
        try:
            if self.arm is not None:
                self.arm.set_damping(0)
        except BaseException:
            pass
        try:
            if self.camera is not None:
                self.camera.capture.release()
        except BaseException:
            pass
        with self._lock:
            if self.state not in (STATE_ESTOP, STATE_FAULT):
                self._set_state(STATE_STOPPED, "控制端已关闭")
        print("[ROBOT] 控制端已关闭")


def build_http_server(controller, host=HOST, port=PORT):
    dedup = {}
    dedup_lock = threading.Lock()

    class GestureRobotHandler(BaseHTTPRequestHandler):
        server_version = "GestureRobot/1.0"

        def _send_json(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def _authorized(self):
            if not CONTROL_TOKEN:
                return True
            supplied = self.headers.get("X-Control-Token", "")
            return hmac.compare_digest(supplied, CONTROL_TOKEN)

        def do_GET(self):
            if not self._authorized():
                self._send_json(401, {"ok": False, "message": "invalid control token"})
                return
            if self.path == "/health":
                snap = controller.snapshot()
                snap.update({"ok": True, "service": "gesture-robot", "time": time.time()})
                self._send_json(200, snap)
                return
            if self.path == "/status":
                self._send_json(200, controller.snapshot())
                return
            self._send_json(404, {"ok": False, "message": "not found"})

        def do_POST(self):
            if self.path != "/command":
                self._send_json(404, {"ok": False, "message": "not found"})
                return
            if not self._authorized():
                self._send_json(401, {"ok": False, "message": "invalid control token"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 65536:
                    raise ValueError("invalid Content-Length")
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
            except BaseException as exc:
                self._send_json(400, {"ok": False, "message": "invalid JSON: %s" % repr(exc)})
                return

            command = str(payload.get("command", "")).upper().strip()
            request_id = str(payload.get("request_id", "")).strip()
            if not command:
                self._send_json(400, {"ok": False, "message": "missing command"})
                return

            with dedup_lock:
                now = time.monotonic()
                for key in [key for key, item in dedup.items() if now - item[0] > 60.0]:
                    dedup.pop(key, None)
                if request_id and request_id in dedup:
                    self._send_json(dedup[request_id][1], dedup[request_id][2])
                    return

            status, message = controller.handle_command(command)
            response = controller.snapshot()
            response.update(
                {
                    "ok": 200 <= status < 300,
                    "message": message,
                    "command": command,
                    "gesture": payload.get("gesture", ""),
                    "request_id": request_id,
                }
            )
            print("[HTTP] %s %s -> %d %s" % (command, payload.get("gesture", ""), status, message))
            if request_id:
                with dedup_lock:
                    dedup[request_id] = (time.monotonic(), status, response)
            self._send_json(status, response)

        def log_message(self, fmt, *args):
            return

    server = ThreadingHTTPServer((host, int(port)), GestureRobotHandler)
    server.daemon_threads = True
    server.controller = controller
    return server


_GLOBAL_SERVER = None
_GLOBAL_CONTROLLER = None
_GLOBAL_THREAD = None


def start_gesture_robot_server(host=HOST, port=PORT, block=False):
    if not CONTROL_TOKEN:
        raise RuntimeError("Set ROBOT_CONTROL_TOKEN before starting the HTTP controller")
    global _GLOBAL_SERVER, _GLOBAL_CONTROLLER, _GLOBAL_THREAD
    if _GLOBAL_SERVER is not None:
        print("[ROBOT] 控制端已经在运行: http://%s:%s" % (host, port))
        return _GLOBAL_SERVER

    controller = GestureRobotController()
    controller.connect()
    server = build_http_server(controller, host=host, port=port)
    thread = threading.Thread(
        target=server.serve_forever,
        name="gesture-robot-http",
        daemon=not block,
    )
    thread.start()
    _GLOBAL_SERVER = server
    _GLOBAL_CONTROLLER = controller
    _GLOBAL_THREAD = thread
    print("[ROBOT] HTTP 控制端已启动: http://%s:%s" % (host, port))
    print("[ROBOT] PC 端地址应为: http://<机械臂主机IP>:%s" % port)
    print("[ROBOT] 命令: ONE=选择抓取, TWO=选择复位, OK=确认, FIST=急停, OPEN_PALM=取消")
    if block:
        try:
            while thread.is_alive():
                thread.join(timeout=1.0)
        except KeyboardInterrupt:
            pass
        finally:
            stop_gesture_robot_server()
    return server


def stop_gesture_robot_server():
    global _GLOBAL_SERVER, _GLOBAL_CONTROLLER, _GLOBAL_THREAD
    if _GLOBAL_SERVER is not None:
        _GLOBAL_SERVER.shutdown()
        _GLOBAL_SERVER.server_close()
    if _GLOBAL_CONTROLLER is not None:
        _GLOBAL_CONTROLLER.shutdown()
    _GLOBAL_SERVER = None
    _GLOBAL_CONTROLLER = None
    _GLOBAL_THREAD = None


if __name__ == "__main__":
    start_gesture_robot_server(block=True)
