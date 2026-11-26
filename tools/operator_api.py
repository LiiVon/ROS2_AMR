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
  GET  /api/missions/events       -> /mission/list_events
  GET  /api/fleet/robots          -> /fleet/list_robots
  GET  /api/facilities/resources  -> /facility/list_resources
  GET  /api/workflows/status      -> /business/get_workflow_status
  POST /api/business_orders       -> /business/submit_order（order_type=business）
  POST /api/station_order_batches -> /mission/submit_station_batch
  POST /api/workflows/pause       -> /business/pause_workflow
  POST /api/workflows/resume      -> /business/resume_workflow
  POST /api/workflows/cancel      -> /business/cancel_workflow
  POST /api/missions/reprioritize       -> /mission/reprioritize
  POST /api/missions/reprioritize_batch -> /mission/reprioritize_batch
  POST /api/remote_tasks/state    -> /fleet/update_remote_task

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
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_interfaces_business.srv import (
    CancelWorkflow,
    GetOperatorSnapshot,
    GetWorkflowStatus,
    PauseWorkflow,
    ResumeWorkflow,
    SubmitOrder,
)
from robot_interfaces_facility.srv import ListFacilityResources
from robot_interfaces_fleet.srv import ListFleetRobots, UpdateRemoteTaskState
from robot_interfaces_mission.srv import (
    ListMissionEvents,
    ReprioritizeBatch,
    ReprioritizeQueuedMission,
    SubmitStationOrderBatch,
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

    def mission_events(
        self, limit: int = 20, state_filter: str = "", mission_id_filter: str = ""
    ) -> dict:
        request = ListMissionEvents.Request()
        request.limit = int(limit)
        request.state_filter = state_filter
        request.mission_id_filter = mission_id_filter
        response = self.call("/mission/list_events", ListMissionEvents, request)
        events = [
            {
                "stamp": stamp,
                "mission_id": mission_id,
                "state": state,
                "previous_state": previous_state,
                "message": message,
                "recoverable": bool(recoverable),
            }
            for stamp, mission_id, state, previous_state, message, recoverable in zip(
                response.stamp,
                response.mission_ids,
                response.states,
                response.previous_states,
                response.messages,
                response.recoverable,
            )
        ]
        return {
            "success": bool(response.success),
            "message": response.message,
            "events": events,
        }

    def fleet_robots(self, capability: str = "", include_disabled: bool = False) -> dict:
        request = ListFleetRobots.Request()
        request.capability = capability
        request.include_disabled = bool(include_disabled)
        response = self.call("/fleet/list_robots", ListFleetRobots, request)
        robots = [
            {
                "robot_id": robot_id,
                "enabled": bool(enabled),
                "state": state,
                "battery_voltage": battery_voltage,
                "current_station_id": station_id,
                "capabilities": capabilities,
            }
            for robot_id, enabled, state, battery_voltage, station_id, capabilities in zip(
                response.robot_ids,
                response.enabled,
                response.states,
                response.battery_voltage,
                response.current_station_ids,
                response.capabilities,
            )
        ]
        return {
            "success": bool(response.success),
            "message": response.message,
            "robots": robots,
        }

    def facility_resources(
        self, resource_type: str = "", include_disabled: bool = False
    ) -> dict:
        request = ListFacilityResources.Request()
        request.resource_type = resource_type
        request.include_disabled = bool(include_disabled)
        response = self.call("/facility/list_resources", ListFacilityResources, request)
        resources = [
            {
                "resource_id": resource_id,
                "resource_type": item_type,
                "station_id": station_id,
                "enabled": bool(enabled),
                "available": bool(available),
                "holder_id": holder_id,
                "status": status,
            }
            for resource_id, item_type, station_id, enabled, available, holder_id, status in zip(
                response.resource_ids,
                response.resource_types,
                response.station_ids,
                response.enabled,
                response.available,
                response.holder_ids,
                response.status,
            )
        ]
        return {
            "success": bool(response.success),
            "message": response.message,
            "resources": resources,
        }

    def workflow_status(self, order_id: str) -> dict:
        request = GetWorkflowStatus.Request()
        request.order_id = order_id
        response = self.call(
            "/business/get_workflow_status", GetWorkflowStatus, request
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "order_id": response.order_id,
            "state": response.state,
            "total_steps": response.total_steps,
            "finished_steps": response.finished_steps,
            "queued_steps": response.queued_steps,
            "running_mission_id": response.running_mission_id,
            "queued_mission_ids": list(response.queued_mission_ids),
            "finished_mission_ids": list(response.finished_mission_ids),
            "paused": bool(response.paused),
        }

    def submit_station_batch(self, payload: dict) -> dict:
        request = SubmitStationOrderBatch.Request()
        request.batch_id = str(payload.get("batch_id") or "")
        request.order_ids = [str(value) for value in payload.get("order_ids") or []]
        request.pickup_station_ids = [
            str(value) for value in payload.get("pickup_station_ids") or []
        ]
        request.dropoff_station_ids = [
            str(value) for value in payload.get("dropoff_station_ids") or []
        ]
        request.priority = int(payload.get("priority") or 50)
        request.start_if_idle = bool(payload.get("start_if_idle", True))
        request.preempt_current = bool(payload.get("preempt_current", False))
        request.continue_on_error = bool(payload.get("continue_on_error", True))
        response = self.call(
            "/mission/submit_station_batch", SubmitStationOrderBatch, request
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "mission_ids": list(response.mission_ids),
            "rejected_order_ids": list(response.rejected_order_ids),
            "rejected_messages": list(response.rejected_messages),
            "accepted_count": response.accepted_count,
            "rejected_count": response.rejected_count,
            "queue_size": response.queue_size,
        }

    def reprioritize(self, mission_id: str, priority: int) -> dict:
        request = ReprioritizeQueuedMission.Request()
        request.mission_id = mission_id
        request.priority = int(priority)
        response = self.call(
            "/mission/reprioritize", ReprioritizeQueuedMission, request
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "queue_size": response.queue_size,
        }

    def reprioritize_batch(self, mission_ids: list, priorities: list) -> dict:
        request = ReprioritizeBatch.Request()
        request.mission_ids = [str(value) for value in mission_ids]
        request.priorities = [int(value) for value in priorities]
        response = self.call(
            "/mission/reprioritize_batch", ReprioritizeBatch, request
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "updated_mission_ids": list(response.updated_mission_ids),
            "queue_size": response.queue_size,
        }

    def remote_task_state(self, task_id: str, state: str, message: str = "") -> dict:
        request = UpdateRemoteTaskState.Request()
        request.task_id = task_id
        request.state = state
        request.message = message
        response = self.call(
            "/fleet/update_remote_task", UpdateRemoteTaskState, request
        )
        return {
            "success": bool(response.success),
            "message": response.message,
            "task_id": response.task_id,
            "assigned_robot_id": response.assigned_robot_id,
            "state": response.state,
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
        if path == "/api/missions/events":
            query = parse_qs(urlparse(self.path).query)
            try:
                limit = int(query.get("limit", ["20"])[0])
            except ValueError:
                limit = 20
            self._guarded(
                lambda: self.bridge.mission_events(
                    limit,
                    query.get("state", [""])[0],
                    query.get("mission_id", [""])[0],
                )
            )
            return
        if path == "/api/fleet/robots":
            query = parse_qs(urlparse(self.path).query)
            include_disabled = query.get("include_disabled", ["false"])[0].lower() in (
                "1",
                "true",
                "yes",
            )
            self._guarded(
                lambda: self.bridge.fleet_robots(
                    query.get("capability", [""])[0], include_disabled
                )
            )
            return
        if path == "/api/facilities/resources":
            query = parse_qs(urlparse(self.path).query)
            include_disabled = query.get("include_disabled", ["false"])[0].lower() in (
                "1",
                "true",
                "yes",
            )
            self._guarded(
                lambda: self.bridge.facility_resources(
                    query.get("resource_type", [""])[0], include_disabled
                )
            )
            return
        if path == "/api/workflows/status":
            query = parse_qs(urlparse(self.path).query)
            order_id = query.get("order_id", [""])[0]
            if not order_id:
                self._json(400, {"success": False, "message": "order_id is required"})
                return
            self._guarded(lambda: self.bridge.workflow_status(order_id))
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
        if path == "/api/station_order_batches":
            order_ids = body.get("order_ids") or []
            pickup_ids = body.get("pickup_station_ids") or []
            dropoff_ids = body.get("dropoff_station_ids") or []
            if not (
                isinstance(order_ids, list)
                and isinstance(pickup_ids, list)
                and isinstance(dropoff_ids, list)
            ):
                self._json(
                    400,
                    {
                        "success": False,
                        "message": "order_ids/pickup_station_ids/dropoff_station_ids must be arrays",
                    },
                )
                return
            if not (len(order_ids) == len(pickup_ids) == len(dropoff_ids)):
                self._json(
                    400,
                    {
                        "success": False,
                        "message": "order/pickup/dropoff arrays must have the same length",
                    },
                )
                return
            try:
                body["priority"] = int(body.get("priority") or 50)
            except (TypeError, ValueError):
                self._json(400, {"success": False, "message": "priority must be an integer"})
                return
            self._guarded(lambda: self.bridge.submit_station_batch(body))
            return
        if path == "/api/missions/reprioritize":
            mission_id = str(body.get("mission_id") or "")
            if not mission_id:
                self._json(400, {"success": False, "message": "mission_id is required"})
                return
            try:
                priority = int(body.get("priority"))
            except (TypeError, ValueError):
                self._json(400, {"success": False, "message": "priority must be an integer"})
                return
            self._guarded(lambda: self.bridge.reprioritize(mission_id, priority))
            return
        if path == "/api/missions/reprioritize_batch":
            mission_ids = body.get("mission_ids") or []
            priorities = body.get("priorities") or []
            if not (isinstance(mission_ids, list) and isinstance(priorities, list)):
                self._json(
                    400,
                    {"success": False, "message": "mission_ids/priorities must be arrays"},
                )
                return
            if len(mission_ids) != len(priorities):
                self._json(
                    400,
                    {
                        "success": False,
                        "message": "mission_ids and priorities must have the same length",
                    },
                )
                return
            try:
                priorities = [int(value) for value in priorities]
            except (TypeError, ValueError):
                self._json(400, {"success": False, "message": "priorities must be integers"})
                return
            self._guarded(lambda: self.bridge.reprioritize_batch(mission_ids, priorities))
            return
        if path == "/api/remote_tasks/state":
            task_id = str(body.get("task_id") or "")
            state = str(body.get("state") or "")
            if not task_id or not state:
                self._json(
                    400,
                    {"success": False, "message": "task_id and state are required"},
                )
                return
            self._guarded(
                lambda: self.bridge.remote_task_state(
                    task_id, state, str(body.get("message") or "")
                )
            )
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
        f"(REST: /api/operator/snapshot, /api/missions/*, /api/fleet/robots, "
        f"/api/facilities/resources, /api/business_orders, /api/station_order_batches, "
        f"/api/workflows/*, /api/remote_tasks/state)"
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
