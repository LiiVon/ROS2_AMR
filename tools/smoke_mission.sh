#!/usr/bin/env bash
# 端到端任务链路冒烟：bringup(mock) + mission_runner，提交订单并校验暂停/恢复。
# 用法：tools/smoke_mission.sh
set -eo pipefail

WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"
cd "${WORKSPACE}"

# ROS 的 setup.bash 会引用未定义变量，source 期间必须临时关闭 -u。
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}

LOG_FILES=(/tmp/smoke_bringup.log /tmp/smoke_mission.log /tmp/smoke_services.txt)
pids=()

# 每个 launch 用 setsid 启动在独立进程组里，退出时杀整个进程组：
# 只 kill 父进程会留下 bringup 的子节点，污染后续测试。
cleanup() {
  for pid in "${pids[@]:-}"; do
    if [[ -n "${pid}" ]]; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${pids[@]:-}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
  rm -f "${LOG_FILES[@]}"
}
trap cleanup EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

setsid ros2 launch robot_bringup bringup.launch.py backend:=mock > /tmp/smoke_bringup.log 2>&1 &
pids+=($!)
setsid ros2 launch robot_tasks mission_runner.launch.py autostart:=false preflight_enabled:=false > /tmp/smoke_mission.log 2>&1 &
pids+=($!)

# 等待业务入口出现（有界轮询，不使用固定 sleep）
ready=0
for _ in $(seq 1 20); do
  timeout 5 ros2 service list > /tmp/smoke_services.txt 2>/dev/null || true
  if grep -q "/business/submit_order" /tmp/smoke_services.txt; then
    ready=1
    break
  fi
done
[[ "${ready}" == "1" ]] || fail "business service not available"

response=$(timeout 20 ros2 service call /business/submit_order \
  robot_interfaces_business/srv/SubmitOrder \
  "{order_type: 'transport', priority: 50, payload_json: 'dropoff_x=1.5;dropoff_y=0.8;dropoff_yaw=0'}" 2>&1)
grep -q "accepted=True" <<<"${response}" || fail "order not accepted: ${response}"
mission_id=$(sed -n "s/.*mission_id='\([^']*\)'.*/\1/p" <<<"${response}" | sed -n '1p')
echo "OK: order accepted, mission_id=${mission_id:-unknown}"

pause=$(timeout 15 ros2 service call /mission/pause std_srvs/srv/Trigger 2>&1)
grep -q "success=True" <<<"${pause}" || fail "pause failed: ${pause}"
echo "OK: mission pause"

resume=$(timeout 15 ros2 service call /mission/resume std_srvs/srv/Trigger 2>&1)
grep -q "success=True" <<<"${resume}" || fail "resume failed: ${resume}"
echo "OK: mission resume"

snapshot=$(timeout 15 ros2 service call /business/get_operator_snapshot \
  robot_interfaces_business/srv/GetOperatorSnapshot "{event_limit: 3}" 2>&1)
grep -q "success=True" <<<"${snapshot}" || fail "operator snapshot failed"
echo "OK: operator snapshot"

echo "SMOKE PASSED"
