#!/usr/bin/env bash
# Pre-flight verifier for the yaw-diagnostic field session.
# Run AFTER launching navigation.launch.py.
#
# Usage:
#   bash scripts/preflight_check.sh
#
# Exits non-zero if any required check fails. Each check prints a line
# like "[PASS] ..." or "[FAIL] ... — <hint>".

set -u
FAILED=0
PASSED=0

check() {
    local desc="$1"; shift
    if "$@" > /dev/null 2>&1; then
        echo "[PASS] $desc"
        PASSED=$((PASSED + 1))
    else
        echo "[FAIL] $desc"
        FAILED=$((FAILED + 1))
    fi
}

note() {
    echo "  → $*"
}

echo "=== IGVC yaw-diag pre-flight ==="
echo

# --- 1. environment ---
if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" ]]; then
    echo "[PASS] RMW_IMPLEMENTATION = rmw_cyclonedds_cpp"
    PASSED=$((PASSED + 1))
else
    echo "[FAIL] RMW_IMPLEMENTATION ≠ rmw_cyclonedds_cpp"
    note "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    FAILED=$((FAILED + 1))
fi

if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    echo "[PASS] CYCLONEDDS_URI set"
    PASSED=$((PASSED + 1))
else
    echo "[FAIL] CYCLONEDDS_URI not set"
    note "export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml"
    FAILED=$((FAILED + 1))
fi

# --- 2. webui must be stopped (holds /dev/ttyACM0) ---
if systemctl is-active --quiet avros-webui 2>/dev/null; then
    echo "[FAIL] avros-webui systemd service is running"
    note "sudo systemctl stop avros-webui"
    FAILED=$((FAILED + 1))
else
    echo "[PASS] avros-webui stopped"
    PASSED=$((PASSED + 1))
fi

# --- 3. /dev/ttyACM0 free (should be held by actuator_node only) ---
# If actuator_node is running (from navigation.launch.py), it holds the port.
# That's expected — we just want it not held by webui or duplicate processes.
ACM_HOLDERS=$(lsof /dev/ttyACM0 2>/dev/null | tail -n +2 | wc -l)
if (( ACM_HOLDERS == 0 )); then
    echo "[WARN] /dev/ttyACM0 has 0 holders — actuator_node not running?"
elif (( ACM_HOLDERS == 1 )); then
    echo "[PASS] /dev/ttyACM0 held by exactly 1 process (actuator_node)"
    PASSED=$((PASSED + 1))
else
    echo "[FAIL] /dev/ttyACM0 held by $ACM_HOLDERS processes — collision!"
    lsof /dev/ttyACM0 2>/dev/null | tail -n +2
    note "kill duplicate holders; only actuator_node should hold it"
    FAILED=$((FAILED + 1))
fi

# --- 4. topic publishing rates ---
# ros2 topic hz needs ~2s to compute. We sample 3s and parse rate.
check_hz() {
    local topic="$1"
    local min_hz="$2"
    local rate
    rate=$(timeout 3 ros2 topic hz "$topic" 2>&1 | \
           grep -oP 'average rate: \K[0-9.]+' | head -1)
    if [[ -z "$rate" ]]; then
        echo "[FAIL] $topic — not publishing"
        FAILED=$((FAILED + 1))
        return
    fi
    if awk -v r="$rate" -v m="$min_hz" 'BEGIN{exit !(r >= m)}'; then
        echo "[PASS] $topic @ ${rate} Hz (≥ ${min_hz})"
        PASSED=$((PASSED + 1))
    else
        echo "[FAIL] $topic @ ${rate} Hz (< ${min_hz} required)"
        FAILED=$((FAILED + 1))
    fi
}

check_hz "/imu/data" 50
check_hz "/wheel_odom" 10
check_hz "/odometry/filtered" 15
check_hz "/avros/wheel_debug" 25
check_hz "/gnss" 1

# /zed_front/zed_node/odom is optional (only if enable_zed_front:=true)
ZED_TOPICS=$(ros2 topic list 2>/dev/null | grep -c '^/zed_front/' || true)
if (( ZED_TOPICS > 0 )); then
    check_hz "/zed_front/zed_node/odom" 5
else
    echo "[WARN] no /zed_front topics found — relaunch with enable_zed_front:=true if Test B needed"
fi

# --- 5. IMU stationary bias check (sample 5s, expect |mean(wz)| < 0.05 rad/s) ---
echo
echo "Sampling /imu/data.wz for 5s (robot must be motionless)..."
WZ_FILE=$(mktemp)
timeout 5 ros2 topic echo /imu/data --field angular_velocity.z 2>/dev/null \
    | grep -E '^[-0-9.e]+$' > "$WZ_FILE" || true
N=$(wc -l < "$WZ_FILE")
if (( N < 50 )); then
    echo "[FAIL] only $N samples in 5s — /imu/data rate too low or stopped"
    FAILED=$((FAILED + 1))
else
    MEAN=$(awk '{s+=$1; n++} END{print s/n}' "$WZ_FILE")
    STD=$(awk -v m="$MEAN" '{d=$1-m; s+=d*d; n++} END{print sqrt(s/n)}' "$WZ_FILE")
    DEG_MEAN=$(awk -v r="$MEAN" 'BEGIN{print r * 57.2958}')
    if awk -v m="$MEAN" 'BEGIN{exit !(m*m < 0.05*0.05)}'; then
        echo "[PASS] IMU stationary bias: mean=${DEG_MEAN}°/s (|mean|<2.86°/s)"
        PASSED=$((PASSED + 1))
    else
        echo "[FAIL] IMU stationary bias: mean=${DEG_MEAN}°/s — STUCK BIAS"
        note "USB power-cycle the Xsens (see CLAUDE.md known-issues)"
        FAILED=$((FAILED + 1))
    fi
fi
rm -f "$WZ_FILE"

# --- summary ---
echo
echo "=== summary: $PASSED passed, $FAILED failed ==="
if (( FAILED == 0 )); then
    echo "READY — proceed with bag recording."
    exit 0
else
    echo "NOT READY — fix the [FAIL] items above before recording."
    exit 1
fi
