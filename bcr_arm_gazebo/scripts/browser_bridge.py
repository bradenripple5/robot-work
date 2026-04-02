#!/usr/bin/env python3

import argparse
import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

PREDEFINED_POSES = {
    "home": [0.0, -1.57, 0.0, 3.14, 0.0, 3.14, 0.0],
    "ready": [0.0, -0.785, 0.0, -1.57, 0.0, 0.785, 0.0],
    "stretch_up": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "forward_low": [0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0],
}

ROS_STATE_FRESHNESS_SEC = 1.5


class BrowserBridge(Node):
    def __init__(self):
        super().__init__("browser_bridge")
        self._lock = threading.Lock()
        self._ros_joint_state = {
            "names": JOINT_NAMES,
            "positions": [0.0] * len(JOINT_NAMES),
            "stamp": 0.0,
            "source": "ros",
        }
        self._virtual_positions = [0.0] * len(JOINT_NAMES)
        self._virtual_motion = None
        self._last_command = {
            "type": "none",
            "label": None,
            "target": None,
            "duration": None,
            "stamp": None,
        }
        self._command_history = []

        self.publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

    def _joint_state_callback(self, msg: JointState):
        positions = []
        for joint_name in JOINT_NAMES:
            try:
                idx = msg.name.index(joint_name)
                positions.append(float(msg.position[idx]))
            except (ValueError, IndexError):
                positions.append(0.0)

        with self._lock:
            self._ros_joint_state = {
                "names": JOINT_NAMES,
                "positions": positions,
                "stamp": time.time(),
                "source": "ros",
            }

    def _ros_state_is_fresh_locked(self, now):
        return (now - float(self._ros_joint_state["stamp"])) < ROS_STATE_FRESHNESS_SEC

    def _current_virtual_positions_locked(self, now):
        if not self._virtual_motion:
            return list(self._virtual_positions)

        progress = min(
            1.0,
            max(
                0.0,
                (now - self._virtual_motion["started_at"]) / self._virtual_motion["duration"],
            ),
        )
        start = self._virtual_motion["start"]
        target = self._virtual_motion["target"]
        positions = [start[i] + (target[i] - start[i]) * progress for i in range(len(target))]

        if progress >= 1.0:
            self._virtual_positions = list(target)
            self._virtual_motion = None
            return list(self._virtual_positions)

        return positions

    def _effective_joint_state_locked(self):
        now = time.time()
        if self._ros_state_is_fresh_locked(now):
            return dict(self._ros_joint_state)

        positions = self._current_virtual_positions_locked(now)
        return {
            "names": JOINT_NAMES,
            "positions": positions,
            "stamp": now,
            "source": "virtual",
        }

    def status(self):
        with self._lock:
            return {
                "ok": True,
                "joint_state": self._effective_joint_state_locked(),
                "last_command": self._last_command,
                "command_history": list(self._command_history),
                "predefined_poses": PREDEFINED_POSES,
            }

    def send_trajectory(self, positions, duration_sec, label="trajectory", source="browser"):
        if len(positions) != len(JOINT_NAMES):
            raise ValueError(f"Expected {len(JOINT_NAMES)} joint positions")

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9)
        )
        msg.points = [point]
        self.publisher.publish(msg)

        with self._lock:
            now = time.time()
            start_positions = self._effective_joint_state_locked()["positions"]
            self._virtual_motion = {
                "start": [float(p) for p in start_positions],
                "target": [float(p) for p in positions],
                "started_at": now,
                "duration": max(0.2, float(duration_sec)),
            }
            self._last_command = {
                "type": "trajectory",
                "label": str(label),
                "target": [float(p) for p in positions],
                "duration": float(duration_sec),
                "stamp": now,
                "source": str(source),
            }
            self._command_history.append(self._last_command.copy())
            self._command_history = self._command_history[-50:]

    def send_pose(self, pose_name, duration_sec, source="browser"):
        if pose_name not in PREDEFINED_POSES:
            raise ValueError(f"Unknown pose: {pose_name}")
        self.send_trajectory(
            PREDEFINED_POSES[pose_name],
            duration_sec,
            label=pose_name,
            source=source,
        )


class BrowserRequestHandler(BaseHTTPRequestHandler):
    bridge = None

    def do_GET(self):
        if self.path == "/api/health":
            self._send_json(HTTPStatus.OK, {"ok": True})
            return

        if self.path == "/api/status":
            self._send_json(HTTPStatus.OK, self.bridge.status())
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        if self.path not in ("/api/pose", "/api/joints"):
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)

        try:
            payload = json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "Invalid JSON"})
            return

        try:
            if self.path == "/api/pose":
                pose_name = payload["pose"]
                duration = float(payload.get("duration", 1.0))
                source = payload.get("source", "browser")
                self.bridge.send_pose(pose_name, duration, source=source)
            else:
                positions = payload["positions"]
                duration = float(payload.get("duration", 1.0))
                label = payload.get("label", "trajectory")
                source = payload.get("source", "browser")
                self.bridge.send_trajectory(positions, duration, label=label, source=source)
        except (KeyError, TypeError, ValueError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            return

        self._send_json(HTTPStatus.OK, self.bridge.status())

    def log_message(self, fmt, *args):
        return

    def _send_json(self, status_code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def parse_args():
    parser = argparse.ArgumentParser(description="Run the BCR Arm browser bridge.")
    parser.add_argument("--host", default="0.0.0.0", help="Host interface to bind")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port to bind")
    return parser.parse_args()


def main(args=None):
    cli_args = parse_args()
    rclpy.init(args=args)
    bridge = BrowserBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    BrowserRequestHandler.bridge = bridge

    host = cli_args.host
    port = cli_args.port
    server = ThreadingHTTPServer((host, port), BrowserRequestHandler)
    bridge.get_logger().info(f"Browser API available at http://{host}:{port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
