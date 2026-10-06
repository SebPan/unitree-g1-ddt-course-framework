#!/usr/bin/env bash

set -e

IMAGE="g1-course-sim:dev"

echo "=========================================="
echo " Unitree G1 - Installation Check"
echo "=========================================="
echo

# ------------------------------------------------------------
# Launcher
# ------------------------------------------------------------

bash -n ./g1
echo "[OK] Launcher g1"

# ------------------------------------------------------------
# Docker image
# ------------------------------------------------------------

docker image inspect "$IMAGE" >/dev/null
echo "[OK] Docker image: $IMAGE"

# ------------------------------------------------------------
# SDK2 Python + CRC
# ------------------------------------------------------------

docker run --rm \
    "$IMAGE" \
    bash -lc '
        source /opt/g1-unitree-venv/bin/activate

        python3 - <<PY
from unitree_sdk2py.utils.crc import CRC

CRC()

print("[OK] Unitree SDK2 Python + CRC")
PY
    '

# ------------------------------------------------------------
# HighLevel SIM controller
# ------------------------------------------------------------

test -x \
controller/unitree_rl_lab_23/deploy/robots/g1_23dof/build/g1_ctrl

echo "[OK] g1_ctrl"

# ------------------------------------------------------------
# Core source
# ------------------------------------------------------------

test -f \
src/g1_core/scripts/g1_high_core.py

echo "[OK] g1_high_core"

test -f \
src/g1_core/src/g1_core.cpp

echo "[OK] g1_core"

# ------------------------------------------------------------
# Student interfaces
# ------------------------------------------------------------

test -f \
src/g1_interface/g1_interface/g1_high_level.py

echo "[OK] G1HighLevel"

test -f \
src/g1_interface/g1_interface/g1_low_level.py

echo "[OK] G1LowLevel"

# ------------------------------------------------------------
# ROS build
# ------------------------------------------------------------

test -f \
install_docker/setup.bash

echo "[OK] ROS workspace compiled"

echo
echo "=========================================="
echo " G1 INSTALLATION OK"
echo "=========================================="
