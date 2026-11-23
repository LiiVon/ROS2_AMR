#!/usr/bin/env python3
"""操作台 REST 网关：把 tools/front_end.html 的 HTTP 请求桥接到 ROS 2 服务。

设计原则：
  * 只依赖 Python 标准库与 rclpy，不引入 Web 框架；
  * **不实现业务逻辑**，每个端点都是对现有 ROS 服务的一对一映射；
  * 服务不可达时返回 503 + JSON 说明，前端可直接展示。

端点：
  GET  /                          -> 返回 front_end.html（同目录）
  GET  /api/health                -> 网关与 ROS 连接状态
  GET  /api/operator/snapshot     -> /business/get_operator_snapshot
  POST /api/business_orders       -> /business/submit_order（order_type=business）
  POST /api/workflows/pause       -> /business/pause_workflow
  POST /api/workflows/resume      -> /business/resume_workflow
  POST /api/workflows/cancel      -> /business/cancel_workflow

用法：
  python3 tools/operator_api.py [--host 127.0.0.1] [--port 18080]
"""

from __future__ import annotations

import argparse
import json
import pathlib
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_interfaces_business.srv import (
    CancelWorkflow,
    GetOperatorSnapshot,
    PauseWorkflow,
    ResumeWorkflow,
    SubmitOrder,
)

HTML_PATH = pathlib.Path(__file__).resolve().parent / "front_end.html"
SERVICE_TIMEOUT_S = 5.0
RESPONSE_TIMEOUT_S = 10.0


class RosBridge:
    """持有 rclpy 节点与执行器，向 HTTP 层提供同步调用接口。"""

    def __init__(self) -> None:
        rclpy.init()
        self.node = Node("operator_api_gateway")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        self._clients: dict[str, object] = {}

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

    def service_ready(self, name: str, srv_type) -> bool:
        """服务是否已就绪（非阻塞，供 /api/health 使用）。"""
        try:
            client = self._client(name, srv_type)
            return bool(client.service_is_ready())
        except Exception:  # noqa: BLE001 - 健康检查不应抛异常
            return False

    def _client(self, name: str, srv_type):
        client = self._clients.get(name)
        if client is None:
            client = self.node.create_client(srv_type, name)
            self._clients[name] = client
        return client

    def call(self, name: str, srv_type, request):
        client = self._client(name, srv_type)
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise ServiceUnavailable(f"service not available: {name}")
        future = client.call_async(request)
        deadline = time.monotonic() + RESPONSE_TIMEOUT_S
        while not future.done():
            if time.monotonic() > deadline:
                raise ServiceUnavailable(f"service timeout: {name}")
            time.sleep(0.01)
        if future.exception() is not None:
            raise ServiceUnavailable(f"service failed: {name}: {future.exception()}")
        return future.result()

    # ---------------- 端点实现（一对一映射） ----------------
    def snapshot(self, event_limit: int = 20) -> dict:
        request = GetOperatorSnapshot.Request()
        request.event_limit = int(event_limit)
        response = self.call(
            "/business/get_operator_snapshot", GetOperatorSnapshot, request
        )
        events = [
            {
                "stamp": stamp,
                "mission_id": mission_id,
                "state": state,
                "message": message,
            }
            for stamp, mission_id, state, message in zip(
                response.recent_event_stamp,
                response.recent_event_mission_ids,
                response.recent_event_states,
                response.recent_event_messages,
            )
        ]
        remote_tasks = [
            {"task_id": task_id, "robot_id": robot_id, "state": state}
            for task_id, robot_id, state in zip(
                response.remote_task_ids,
                response.remote_task_robot_ids,
                response.remote_task_states,
            )
        ]
        facilities = [
            {
                "resource_id": resource_id,
                "resource_type": resource_type,
                "station_id": station_id,
                "available": available,
                "status": status,
            }
            for resource_id, resource_type, station_id, available, status in zip(
                response.facility_resource_ids,
                response.facility_resource_types,
                response.facility_station_ids,
                response.facility_available,
                response.facility_status,
            )
        ]
        return {
            "success": bool(response.success),
            "message": response.message,
            "mission": {
                "state": response.mission_state,
                "previous_state": response.previous_mission_state,
                "active_mission_id": response.active_mission_id,
                "message": response.mission_message,
                "recoverable": bool(response.recoverable),
                "active": bool(response.mission_active),
                "paused": bool(response.paused),
                "queue_size": response.queue_size,
            },
            "queue": {
                "size": response.queue_size,
                "mission_ids": list(response.queued_mission_ids),
                "priorities": list(response.queued_priorities),
            },
            "paused_workflow_order_ids": list(response.paused_workflow_order_ids),
            "fleet_robot_ids": list(response.fleet_robot_ids),
            "fleet": [
                {
                    "robot_id": robot_id,
                    "enabled": enabled,
                    "state": state,
                    "battery_voltage": voltage,
                    "station_id": station_id,
                    "capabilities": capabilities,
                }
                for robot_id, enabled, state, voltage, station_id, capabilities in zip(
                    response.fleet_robot_ids,
                    response.fleet_enabled,
                    response.fleet_states,
                    response.fleet_battery_voltage,
                    response.fleet_current_station_ids,
                    response.fleet_capabilities,
                )
            ],
            "remote_tasks": remote_tasks,
            "facility_resource_ids": list(response.facility_resource_ids),
            "facilities": facilities,
            "business_types": list(response.business_types),
            "recent_mission_events": events,
        }

    def submit_business_order(
        self, business_order_id: str, business_type: str, priority: int = 50
    ) -> dict:
        """经唯一订单入口提交业务单（order_type=business 由路由内部展开）。"""
        request = SubmitOrder.Request()
        request.order_id = business_order_id
        request.order_type = "business"
        request.priority = int(priority)
        request.payload_json = (
            f"business_order_id={business_order_id};business_type={business_type}"
            ";start_if_idle=true;preempt_current=false"
        )
        response = self.call("/business/submit_order", SubmitOrder, request)
        return {
            "success": bool(response.accepted),
            "message": response.message if response.accepted else response.reject_reason,
            "business_order_id": business_order_id,
            "business_type": business_type,
            # 实现约定：业务单展开后，工作流的 order_id 为 business_<业务单号>
            "workflow_order_id": f"business_{business_order_id}",
            "mission_ids": list(response.mission_ids),
            "mission_id": response.mission_id,
            "queue_size": response.queue_size,
        }

    def workflow_pause(self, order_id: str) -> dict:
        request = PauseWorkflow.Request()
        request.order_id = order_id
        request.pause_active = True
        response = self.call("/business/pause_workflow", PauseWorkflow, request)
        return {
            "success": bool(response.success),
            "message": response.message,
            "affected_mission_ids": list(response.affected_mission_ids),
            "active_pause_requested": bool(response.active_pause_requested),
        }

    def workflow_resume(self, order_id: str) -> dict:
        request = ResumeWorkflow.Request()
        request.order_id = order_id
        request.resume_active = True
        response = self.call("/business/resume_workflow", ResumeWorkflow, request)
        return {
            "success": bool(response.success),
            "message": response.message,
            "affected_mission_ids": list(response.affected_mission_ids),
            "active_resume_requested": bool(response.active_resume_requested),
        }

    def workflow_cancel(self, order_id: str) -> dict:
        request = CancelWorkflow.Request()
        request.order_id = order_id
        request.cancel_active = True
        response = self.call("/business/cancel_workflow", CancelWorkflow, request)
        return {
            "success": bool(response.success),
            "message": response.message,
            "canceled_mission_ids": list(response.canceled_mission_ids),
            "active_cancel_requested": bool(response.active_cancel_requested),
        }


class ServiceUnavailable(RuntimeError):
    """ROS 服务不可达或超时。"""


class Handler(BaseHTTPRequestHandler):
    bridge: RosBridge  # 由 main() 注入
    server_version = "OperatorApi/1.0"

    # ---------- 基础工具 ----------
    def _send(self, status: int, payload: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _json(self, status: int, data: dict) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self._send(status, body, "application/json; charset=utf-8")

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return {}

    def log_message(self, fmt: str, *args) -> None:  # 降低日志噪声
        print(f"[operator_api] {self.address_string()} {fmt % args}")

    # ---------- 路由 ----------
    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send(204, b"", "text/plain")

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/index.html", "/front_end.html"):
            if not HTML_PATH.exists():
                self._json(500, {"success": False, "message": f"missing {HTML_PATH}"})
                return
            self._send(200, HTML_PATH.read_bytes(), "text/html; charset=utf-8")
            return
        if path == "/api/health":
            bridge = self.bridge
            ros_ok = bool(rclpy.ok())
            snapshot_ready = bridge.service_ready(
                "/business/get_operator_snapshot", GetOperatorSnapshot
            )
            submit_ready = bridge.service_ready("/business/submit_order", SubmitOrder)
            self._json(
                200,
                {
                    "success": True,
                    "ros": ros_ok,
                    "services_ready": snapshot_ready and submit_ready,
                    "snapshot_service": snapshot_ready,
                    "submit_service": submit_ready,
                    "html": HTML_PATH.exists(),
                },
            )
            return
        if path == "/api/operator/snapshot":
            limit = 20
            query = urlparse(self.path).query
            if "event_limit=" in query:
                try:
                    limit = int(query.split("event_limit=")[1].split("&")[0])
                except ValueError:
                    limit = 20
            self._guarded(lambda: self.bridge.snapshot(limit))
            return
        self._json(404, {"success": False, "message": f"unknown path: {path}"})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        body = self._body()
        if path == "/api/business_orders":
            order_id = str(body.get("business_order_id") or body.get("order_id") or "")
            business_type = str(body.get("business_type") or "")
            try:
                priority = int(body.get("priority") or 50)
            except (TypeError, ValueError):
                self._json(400, {"success": False, "message": "priority must be an integer"})
                return
            if not business_type:
                self._json(400, {"success": False, "message": "business_type is required"})
                return
            if not order_id:
                order_id = f"console_{int(time.time())}"
            self._guarded(
                lambda: self.bridge.submit_business_order(order_id, business_type, priority)
            )
            return
        if path in (
            "/api/workflows/pause",
            "/api/workflows/resume",
            "/api/workflows/cancel",
        ):
            order_id = str(body.get("order_id") or "")
            if not order_id:
                self._json(400, {"success": False, "message": "order_id is required"})
                return
            action = path.rsplit("/", 1)[-1]
            call = {
                "pause": self.bridge.workflow_pause,
                "resume": self.bridge.workflow_resume,
                "cancel": self.bridge.workflow_cancel,
            }[action]
            self._guarded(lambda: call(order_id))
            return
        self._json(404, {"success": False, "message": f"unknown path: {path}"})

    def _guarded(self, func) -> None:
        try:
            self._json(200, func())
        except ServiceUnavailable as exc:
            self._json(503, {"success": False, "message": str(exc)})
        except Exception as exc:  # noqa: BLE001 - 网关不因单次调用崩溃
            self._json(500, {"success": False, "message": f"{type(exc).__name__}: {exc}"})


def main() -> int:
    parser = argparse.ArgumentParser(description="Operator console REST gateway")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    args = parser.parse_args()

    bridge = RosBridge()
    Handler.bridge = bridge
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(
        f"[operator_api] serving http://{args.host}:{args.port}/ "
        f"(REST: /api/operator/snapshot, /api/business_orders, /api/workflows/*)"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[operator_api] shutting down")
    finally:
        server.server_close()
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
