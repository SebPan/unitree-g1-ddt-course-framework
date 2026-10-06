FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

SHELL ["/bin/bash", "-c"]


# ============================================================
# Dependencias generales
# ============================================================

RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    unzip \
    nano \
    build-essential \
    cmake \
    g++ \
    pkg-config \
    python3-pip \
    python3-venv  \
    python3-colcon-common-extensions \
    python3-rosdep \
    libyaml-cpp-dev \
    libspdlog-dev \
    libboost-all-dev \
    libeigen3-dev \
    libfmt-dev \
    libglfw3-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-utils \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    xterm \
    x11-utils \
    dbus-x11 \
    python3-xdg \
    menu \
    iproute2 \
    iputils-ping \
    net-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Unitree SDK2
# ============================================================

RUN git clone \
    https://github.com/unitreerobotics/unitree_sdk2.git \
    /opt/unitree_sdk2 \
    && mkdir -p /opt/unitree_sdk2/build \
    && cd /opt/unitree_sdk2/build \
    && cmake .. \
        -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics \
        -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) \
    && make install


# ============================================================
# Unitree RL MjLab
# ============================================================

RUN git clone \
    https://github.com/unitreerobotics/unitree_rl_mjlab.git \
    /opt/unitree_rl_mjlab


# ============================================================
# MuJoCo 3.3.6
# ============================================================

RUN mkdir -p /root/.mujoco \
    && wget \
        https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-x86_64.tar.gz \
        -O /tmp/mujoco.tar.gz \
    && tar -xzf /tmp/mujoco.tar.gz \
        -C /root/.mujoco \
    && rm /tmp/mujoco.tar.gz


# unitree_rl_mjlab espera MuJoCo en simulate/mujoco

RUN ln -s \
    /root/.mujoco/mujoco-3.3.6 \
    /opt/unitree_rl_mjlab/simulate/mujoco


# ============================================================
# Backend SIM directo: MuJoCo -> UDP -> g1_core
# ============================================================

COPY docker/sim_udp_state.h \
    /opt/unitree_rl_mjlab/simulate/src/sim_udp_state.h

COPY docker/sim_udp_command.h \
    /opt/unitree_rl_mjlab/simulate/src/sim_udp_command.h

COPY docker/patch_sim_bridge.py \
    /tmp/patch_sim_bridge.py

RUN python3 /tmp/patch_sim_bridge.py \
    && rm /tmp/patch_sim_bridge.py


# ============================================================
# Compilar simulador MuJoCo
# ============================================================

RUN mkdir -p /opt/unitree_rl_mjlab/simulate/build \
    && cd /opt/unitree_rl_mjlab/simulate/build \
    && cmake .. \
    && make -j$(nproc)


# ============================================================
# Configurar G1 23 DOF
# ============================================================

RUN sed -i \
    's|robot: "g1"|#robot: "g1"|' \
    /opt/unitree_rl_mjlab/simulate/config.yaml \
    && sed -i \
    's|robot_scene: "src/assets/robots/unitree_g1/xmls/scene_g1.xml"|#robot_scene: "src/assets/robots/unitree_g1/xmls/scene_g1.xml"|' \
    /opt/unitree_rl_mjlab/simulate/config.yaml \
    && sed -i \
    's|#robot: "g1_23dof"|robot: "g1_23dof"|' \
    /opt/unitree_rl_mjlab/simulate/config.yaml \
    && sed -i \
    's|#robot_scene: "src/assets/robots/unitree_g1/xmls/scene_g1_23dof.xml"|robot_scene: "src/assets/robots/unitree_g1/xmls/scene_g1_23dof.xml"|' \
    /opt/unitree_rl_mjlab/simulate/config.yaml \
    && sed -i \
    's|use_joystick: 1|use_joystick: 0|' \
    /opt/unitree_rl_mjlab/simulate/config.yaml \
    && sed -i \
    's/^enable_elastic_band:.*/enable_elastic_band: 1/' \
    /opt/unitree_rl_mjlab/simulate/config.yaml

# ============================================================
# Unitree ROS 2
# ============================================================

RUN git clone \
    https://github.com/unitreerobotics/unitree_ros2.git \
    /opt/unitree_ros2


# ============================================================
# CycloneDDS compatible con Unitree
# ============================================================

RUN cd /opt/unitree_ros2/cyclonedds_ws/src \
    && git clone \
        https://github.com/ros2/rmw_cyclonedds.git \
        -b humble \
    && git clone \
        https://github.com/eclipse-cyclonedds/cyclonedds.git \
        -b releases/0.10.x


# IMPORTANTE:
# Se compila CycloneDDS sin cargar el entorno ROS.
# Además desactivamos DDSPerf e Iceoryx SHM porque no los necesitamos.

RUN unset AMENT_PREFIX_PATH \
    && unset CMAKE_PREFIX_PATH \
    && export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu \
    && cd /opt/unitree_ros2/cyclonedds_ws \
    && colcon build \
        --packages-select cyclonedds \
        --cmake-args \
        -DBUILD_DDSPERF=OFF \
        -DENABLE_SHM=OFF


# ============================================================
# Compilar mensajes ROS 2 de Unitree
# ============================================================

RUN source /opt/ros/humble/setup.bash \
    && source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash \
    && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    && cd /opt/unitree_ros2/cyclonedds_ws \
    && colcon build \
        --packages-select \
        unitree_go \
        unitree_hg \
        unitree_api

# ============================================================
# Unitree SDK2 Python - HighLevel REAL
#
# Entorno Python aislado incluido en la imagen.
# El alumno NO instala nada manualmente.
# ============================================================

RUN git clone \
    https://github.com/unitreerobotics/unitree_sdk2_python.git \
    /opt/unitree_sdk2_python \
    && python3 -m venv \
        --system-site-packages \
        /opt/g1-unitree-venv \
    && source /opt/g1-unitree-venv/bin/activate \
    && python -m pip install --upgrade \
        pip \
        setuptools \
        wheel \
    && python -m pip install \
        pyyaml \
    && export CYCLONEDDS_HOME=/opt/unitree_ros2/cyclonedds_ws/install/cyclonedds \
    && python -m pip install \
        /opt/unitree_sdk2_python \
    && SDK2_PKG_DIR="$(python -c 'import os, unitree_sdk2py; print(os.path.dirname(unitree_sdk2py.__file__))')" \
    && mkdir -p "$SDK2_PKG_DIR/utils/lib" \
    && cp \
        /opt/unitree_sdk2_python/unitree_sdk2py/utils/lib/crc_amd64.so \
        "$SDK2_PKG_DIR/utils/lib/crc_amd64.so" \
    && python -c "\
import unitree_sdk2py; \
from unitree_sdk2py.core.channel import ChannelFactoryInitialize; \
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient; \
from unitree_sdk2py.utils.crc import CRC; \
CRC(); \
print('Unitree SDK2 Python + CRC OK')"



# ============================================================
# Fix SDK2 Python CRC native library
# ============================================================

RUN source /opt/g1-unitree-venv/bin/activate \
    && SDK2_PKG_DIR="$(python -c 'import os, unitree_sdk2py; print(os.path.dirname(unitree_sdk2py.__file__))')" \
    && mkdir -p "$SDK2_PKG_DIR/utils/lib" \
    && cp \
        /opt/unitree_sdk2_python/unitree_sdk2py/utils/lib/crc_amd64.so \
        "$SDK2_PKG_DIR/utils/lib/crc_amd64.so" \
    && python -c "from unitree_sdk2py.utils.crc import CRC; CRC(); print('Unitree SDK2 Python + CRC OK')"


# ============================================================
# GUI: Xvfb + Openbox + noVNC
# ============================================================

COPY start-gui.sh /usr/local/bin/start-gui.sh

# Windows puede guardar el archivo como CRLF.
# Lo convertimos automáticamente a LF.

RUN sed -i 's/\r$//' /usr/local/bin/start-gui.sh \
    && chmod +x /usr/local/bin/start-gui.sh \
    && ln -sf \
        /usr/share/novnc/vnc.html \
        /usr/share/novnc/index.html

# ============================================================
# Runtime user
#
# Se crea con el mismo UID/GID que el usuario de WSL.
# ./g1 ejecutará el contenedor con estos IDs.
# ============================================================

ARG G1_UID=1000
ARG G1_GID=1000

RUN set -eux; \
    if ! getent group "${G1_GID}" >/dev/null; then \
        groupadd --gid "${G1_GID}" g1; \
    fi; \
    if ! getent passwd "${G1_UID}" >/dev/null; then \
        useradd \
            --uid "${G1_UID}" \
            --gid "${G1_GID}" \
            --create-home \
            --shell /bin/bash \
            g1; \
    fi; \
    mkdir -p /home/g1; \
    chown "${G1_UID}:${G1_GID}" /home/g1

# ============================================================
# Launcher MuJoCo
#
# MuJoCo / Unitree SDK2 necesita SU CycloneDDS.
# No ponemos estas librerías globalmente para no contaminar ROS.
# ============================================================

RUN printf '%s\n' \
    '#!/bin/bash' \
    'export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/ros/humble/lib/x86_64-linux-gnu' \
    'cd /opt/unitree_rl_mjlab' \
    'exec ./simulate/build/unitree_mujoco "$@"' \
    > /usr/local/bin/run-g1-sim \
    && sed -i 's/\r$//' /usr/local/bin/run-g1-sim \
    && chmod +x /usr/local/bin/run-g1-sim


# ============================================================
# Entorno ROS automático para cada terminal
# ============================================================

RUN printf '%s\n' \
    '' \
    '# ========================================================' \
    '# G1 Course ROS Environment' \
    '# ========================================================' \
    'source /opt/ros/humble/setup.bash' \
    '' \
    'if [ -f /opt/unitree_ros2/cyclonedds_ws/install/setup.bash ]; then' \
    '    source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash' \
    'fi' \
    '' \
    'if [ -f /workspace/install/setup.bash ]; then' \
    '    source /workspace/install/setup.bash' \
    'fi' \
    '' \
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    'export ROS_DOMAIN_ID=0' \
    "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>'" \
    >> /root/.bashrc


# ============================================================
# Workspace
# ============================================================

WORKDIR /workspace


# ============================================================
# Inicio
# ============================================================

CMD ["/usr/local/bin/start-gui.sh"]
