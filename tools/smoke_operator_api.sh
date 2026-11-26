#!/usr/bin/env bash
# 操作台 REST 网关冒烟：bringup(mock) + mission_runner + operator_api。
# 用法：tools/smoke_operator_api.sh
set -eo pipefail

WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"
cd "${WORKSPACE}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}

PORT="${OPERATOR_API_PORT:-18084}"
LOG_DIR="$(mktemp -d /tmp/operator_api_smoke.XXXXXX)"
pids=()

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
  rm -rf "${LOG_DIR}"
}
trap cleanup EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

json_check() {
  local file="$1"
  local expr="$2"
  python3 - "${file}" "${expr}" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1], encoding="utf-8"))
ok = bool(eval(sys.argv[2], {"data": data}))  # noqa: S307 - 固定表达式
if not ok:
    print(json.dumps(data, ensure_ascii=False, indent=2))
    raise SystemExit(1)
PY
}

setsid ros2 launch robot_bringup bringup.launch.py backend:=mock >"${LOG_DIR}/bringup.log" 2>&1 &
pids+=($!)
setsid ros2 launch robot_tasks mission_runner.launch.py autostart:=false preflight_enabled:=false \
  auto_start_queue:=false >"${LOG_DIR}/mission.log" 2>&1 &
pids+=($!)

ready=0
for _ in $(seq 1 40); do
  if timeout 5 ros2 service list 2>/dev/null | grep -q "/business/submit_order"; then
    ready=1
    break
  fi
done
[[ "${ready}" == "1" ]] || fail "business service not available"

setsid python3 tools/operator_api.py --port "${PORT}" >"${LOG_DIR}/api.log" 2>&1 &
pids+=($!)

ready=0
for _ in $(seq 1 40); do
  if curl -sf --max-time 5 "http://127.0.0.1:${PORT}/api/health" >"${LOG_DIR}/health.json"; then
    ready=1
    break
  fi
  sleep 0.5
done
[[ "${ready}" == "1" ]] || fail "operator api not ready"
json_check "${LOG_DIR}/health.json" "data.get('success') and data.get('html')"
echo "OK: health"

curl -sf --max-time 20 "http://127.0.0.1:${PORT}/api/fleet/robots" >"${LOG_DIR}/fleet.json"
json_check "${LOG_DIR}/fleet.json" "data.get('success') and len(data.get('robots', [])) >= 1"
echo "OK: fleet robots"

curl -sf --max-time 20 "http://127.0.0.1:${PORT}/api/facilities/resources" >"${LOG_DIR}/facilities.json"
json_check "${LOG_DIR}/facilities.json" "data.get('success') and len(data.get('resources', [])) >= 1"
echo "OK: facility resources"

curl -sf --max-time 20 "http://127.0.0.1:${PORT}/api/missions/events?limit=5" >"${LOG_DIR}/events.json"
json_check "${LOG_DIR}/events.json" "data.get('success')"
echo "OK: mission events"

curl -sf --max-time 25 -X POST "http://127.0.0.1:${PORT}/api/station_order_batches" \
  -H 'Content-Type: application/json' \
  -d '{"batch_id":"api_smoke","order_ids":["api_smoke_1"],"pickup_station_ids":["receiving"],"dropoff_station_ids":["packing"],"priority":60,"start_if_idle":false}' \
  >"${LOG_DIR}/batch.json"
json_check "${LOG_DIR}/batch.json" "data.get('success') and data.get('accepted_count') == 1"
mission_id="$(python3 -c "import json;print(json.load(open('${LOG_DIR}/batch.json'))['mission_ids'][0])")"
echo "OK: station batch (${mission_id})"

curl -sf --max-time 20 -X POST "http://127.0.0.1:${PORT}/api/missions/reprioritize" \
  -H 'Content-Type: application/json' \
  -d "{\"mission_id\":\"${mission_id}\",\"priority\":90}" >"${LOG_DIR}/reprioritize.json"
json_check "${LOG_DIR}/reprioritize.json" "data.get('success')"
echo "OK: reprioritize"

curl -sf --max-time 20 -X POST "http://127.0.0.1:${PORT}/api/missions/reprioritize_batch" \
  -H 'Content-Type: application/json' \
  -d "{\"mission_ids\":[\"${mission_id}\"],\"priorities\":[80]}" >"${LOG_DIR}/reprioritize_batch.json"
json_check "${LOG_DIR}/reprioritize_batch.json" "data.get('success')"
echo "OK: reprioritize batch"

code="$(curl -s -o "${LOG_DIR}/bad_batch.json" -w '%{http_code}' --max-time 10 \
  -X POST "http://127.0.0.1:${PORT}/api/station_order_batches" \
  -H 'Content-Type: application/json' \
  -d '{"order_ids":["a"],"pickup_station_ids":[],"dropoff_station_ids":["x"]}')"
[[ "${code}" == "400" ]] || fail "bad batch expected 400, got ${code}"
echo "OK: 400 validation"

echo "OPERATOR API SMOKE PASSED"
