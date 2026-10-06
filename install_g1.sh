#!/usr/bin/env bash

set -e

echo "=========================================="
echo " Unitree G1 Course - Installation"
echo "=========================================="
echo

if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] Docker no esta disponible."
    echo "Instala Docker Desktop y habilita integracion con WSL."
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "[ERROR] Docker Engine no esta activo."
    exit 1
fi

echo "[OK] Docker"

chmod +x ./g1

echo
echo "=== Construyendo imagen Docker ==="
docker build -t g1-course-sim:dev docker_image

echo
echo "=== Preparando ONNX Runtime 1.22.0 ==="

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/g1-home \
    -v "$(pwd):/workspace" \
    --workdir /workspace \
    g1-course-sim:dev \
    bash -lc '
        set -e

        ORT_BASE="/workspace/controller/unitree_rl_lab_23/deploy/thirdparty"
        ORT_DIR="$ORT_BASE/onnxruntime-linux-x64-1.22.0"
        ORT_ARCHIVE="/tmp/onnxruntime-linux-x64-1.22.0.tgz"
        ORT_URL="https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz"

        if [ ! -f "$ORT_DIR/include/onnxruntime_cxx_api.h" ] || \
           [ ! -f "$ORT_DIR/lib/libonnxruntime.so.1.22.0" ]; then

            echo "Descargando ONNX Runtime 1.22.0..."

            mkdir -p "$ORT_BASE"
            rm -rf "$ORT_DIR"

            python3 -c "import urllib.request; urllib.request.urlretrieve(\"$ORT_URL\", \"$ORT_ARCHIVE\")"

            tar -xzf "$ORT_ARCHIVE" -C "$ORT_BASE"

            rm -f "$ORT_ARCHIVE"
        fi

        test -f "$ORT_DIR/include/onnxruntime_cxx_api.h"
        test -f "$ORT_DIR/lib/libonnxruntime.so.1.22.0"

        echo "[OK] ONNX Runtime 1.22.0"
    '

echo
echo "=== Preparando ONNX Runtime 1.22.0 ==="

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp/g1-home \
    -v "$(pwd):/workspace" \
    --workdir /workspace \
    g1-course-sim:dev \
    bash -lc '
        set -e

        ORT_BASE="/workspace/controller/unitree_rl_lab_23/deploy/thirdparty"
        ORT_DIR="$ORT_BASE/onnxruntime-linux-x64-1.22.0"
        ORT_ARCHIVE="/tmp/onnxruntime-linux-x64-1.22.0.tgz"
        ORT_URL="https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz"

        if [ ! -f "$ORT_DIR/include/onnxruntime_cxx_api.h" ] || \
           [ ! -f "$ORT_DIR/lib/libonnxruntime.so.1.22.0" ]; then

            echo "Descargando ONNX Runtime 1.22.0..."

            mkdir -p "$ORT_BASE"
            rm -rf "$ORT_DIR"

            python3 -c "import urllib.request; urllib.request.urlretrieve(\"$ORT_URL\", \"$ORT_ARCHIVE\")"

            tar -xzf "$ORT_ARCHIVE" -C "$ORT_BASE"

            rm -f "$ORT_ARCHIVE"
        fi

        test -f "$ORT_DIR/include/onnxruntime_cxx_api.h"
        test -f "$ORT_DIR/lib/libonnxruntime.so.1.22.0"

        echo "[OK] ONNX Runtime 1.22.0"
    '

echo
echo "=== Compilando controlador HighLevel SIM ==="

docker run --rm     --user "$(id -u):$(id -g)"     -e HOME=/tmp/g1-home     -v "$(pwd):/workspace"     --workdir /workspace     g1-course-sim:dev     bash -lc '
        set -e

        cd /workspace/controller/unitree_rl_lab_23/deploy/robots/g1_23dof

        rm -rf build

        cmake             -S .             -B build             -DCMAKE_BUILD_TYPE=Release

        cmake --build build             -j$(nproc)

        test -x build/g1_ctrl

        echo "[OK] g1_ctrl compilado"
    '

echo
echo "=== Compilando workspace ROS ==="
./g1 build

echo
echo "=========================================="
echo " INSTALACION COMPLETA"
echo "=========================================="
echo
echo "Pruebas:"
echo
echo "  ./g1 sim --mode low"
echo "  ./g1 sim --mode high"
echo
echo "Robot real:"
echo
echo "  ./g1 real --mode low"
echo "  ./g1 real --mode high"
echo
echo "El launcher detecta automaticamente la interfaz 192.168.123.x."
echo "Si quieres seleccionarla manualmente:"
echo
echo "  ./g1 real --mode high --interface ethX"
echo
