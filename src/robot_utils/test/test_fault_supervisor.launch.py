import time
import unittest

import launch
import launch_testing.actions
import launch_ros.actions
import rclpy
from rclpy.executors import SingleThreadedExecutor
from robot_interfaces_core.msg import RobotState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def generate_test_description():
    # 只启动被测节点：急停服务由测试进程内的桩提供（见 StubEstopServer），
    # 避免 robot_utils 反过来依赖 robot_teleop（否则两包互相依赖无法拓扑排序）。
    fault_supervisor = launch_ros.actions.Node(
        package="robot_utils",
        executable="fault_supervisor_node",
        parameters=[{"auto_clear": True, "command_cooldown_ms": 50, "startup_grace_ms": 0}],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [
                fault_supervisor,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"fault_supervisor": fault_supervisor},
    )


class StubEstopServer:
    """测试内桩：提供急停/解除服务，并周期发布 /system/emergency_stop。

    行为与安全闸门一致：
      * enable 服务收到 data=true → 急停置位；
      * clear  服务收到 data=true → 急停解除；
      * 周期发布当前状态，供测试观察。
    """

    def __init__(self, node):
        self._active = False
        self._publisher = node.create_publisher(Bool, "/system/emergency_stop", 10)
        self._enable = node.create_service(
            SetBool, "/teleop/enable_emergency_stop", self._make_handler(True)
        )
        self._clear = node.create_service(
            SetBool, "/teleop/clear_emergency_stop", self._make_handler(False)
        )
        self._timer = node.create_timer(0.05, self._publish)

    def _make_handler(self, target_state):
        def handler(request, response):
            if not request.data:
                response.success = False
                response.message = "data must be true"
                return response
            self._active = target_state
            response.success = True
            response.message = "stub emergency stop " + ("active" if target_state else "cleared")
            return response

        return handler

    def _publish(self):
        msg = Bool()
        msg.data = self._active
        self._publisher.publish(msg)

    def destroy(self):
        self._enable.destroy()
        self._clear.destroy()


class TestFaultSupervisor(unittest.TestCase):
    def test_error_health_triggers_and_ok_clears_estop(self):
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("fault_supervisor_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        health_pub = node.create_publisher(RobotState, "/system/health", 10)
        stub = StubEstopServer(node)
        estop = {"seen": False, "active": False}
        estop_sub = node.create_subscription(
            Bool,
            "/system/emergency_stop",
            lambda msg: estop.update({"seen": True, "active": msg.data}),
            10,
        )
        try:
            self._publish_health_until_estop(
                node, executor, health_pub, estop, "ERROR", True, timeout_sec=10.0
            )
            self._publish_health_until_estop(
                node, executor, health_pub, estop, "OK", False, timeout_sec=10.0
            )
        finally:
            stub.destroy()
            node.destroy_subscription(estop_sub)
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _publish_health_until_estop(
        self, node, executor, publisher, estop, state, expected_active, timeout_sec
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            msg = RobotState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.state = state
            msg.message = f"test health {state}"
            msg.recoverable = state != "ERROR"
            publisher.publish(msg)
            executor.spin_once(timeout_sec=0.1)
            if estop["seen"] and estop["active"] == expected_active:
                return
        self.fail(
            f"emergency stop did not become {expected_active} after health {state}; "
            f"last={estop}"
        )
