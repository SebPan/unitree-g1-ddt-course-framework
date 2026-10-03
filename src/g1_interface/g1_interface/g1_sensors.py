import math
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu


class G1Sensors:
    """
    API educativa de sensores del Unitree G1.

    Esta clase NO conoce:
      - LowState
      - DDS
      - motor_id
      - índices internos de Unitree
      - diferencias SIM / REAL

    Solo consume la interfaz normalizada publicada por g1_core:

        /g1/joint_states
        /g1/imu
    """

    def __init__(self):
        self._owns_rclpy = False

        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True

        self._node = Node("g1_sensors")

        self._lock = threading.Lock()

        self._joint_event = threading.Event()
        self._imu_event = threading.Event()

        self._joint_names = []

        self._position = {}
        self._velocity = {}
        self._effort = {}

        self._imu = None

        self._joint_subscription = self._node.create_subscription(
            JointState,
            "/g1/joint_states",
            self._joint_callback,
            10,
        )

        self._imu_subscription = self._node.create_subscription(
            Imu,
            "/g1/imu",
            self._imu_callback,
            10,
        )

        self._running = True

        self._spin_thread = threading.Thread(
            target=self._spin,
            daemon=True,
        )

        self._spin_thread.start()


    # ============================================================
    # ROS interno
    # ============================================================

    def _spin(self):
        while self._running and rclpy.ok():
            rclpy.spin_once(
                self._node,
                timeout_sec=0.1,
            )


    def _joint_callback(self, msg: JointState):
        count = len(msg.name)

        if len(msg.position) < count:
            return

        with self._lock:
            self._joint_names = list(msg.name)

            self._position = {
                name: msg.position[i]
                for i, name in enumerate(msg.name)
            }

            self._velocity = {
                name: (
                    msg.velocity[i]
                    if i < len(msg.velocity)
                    else 0.0
                )
                for i, name in enumerate(msg.name)
            }

            self._effort = {
                name: (
                    msg.effort[i]
                    if i < len(msg.effort)
                    else 0.0
                )
                for i, name in enumerate(msg.name)
            }

        self._joint_event.set()


    def _imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll, pitch, yaw = self._quaternion_to_rpy(
            qx,
            qy,
            qz,
            qw,
        )

        imu = {
            "orientation": {
                "x": qx,
                "y": qy,
                "z": qz,
                "w": qw,
            },

            "gyroscope": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },

            "accelerometer": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },

            "rpy": {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            },
        }

        with self._lock:
            self._imu = imu

        self._imu_event.set()


    @staticmethod
    def _quaternion_to_rpy(x, y, z, w):
        sinr_cosp = 2.0 * (
            w * x +
            y * z
        )

        cosr_cosp = 1.0 - 2.0 * (
            x * x +
            y * y
        )

        roll = math.atan2(
            sinr_cosp,
            cosr_cosp,
        )


        sinp = 2.0 * (
            w * y -
            z * x
        )

        if abs(sinp) >= 1.0:
            pitch = math.copysign(
                math.pi / 2.0,
                sinp,
            )
        else:
            pitch = math.asin(sinp)


        siny_cosp = 2.0 * (
            w * z +
            x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y +
            z * z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        return roll, pitch, yaw


    # ============================================================
    # Estado
    # ============================================================

    def wait_for_data(self, timeout=2.0):
        """
        Espera hasta recibir joints e IMU.

        Retorna:
            True  -> ambos disponibles
            False -> timeout
        """

        start = time.monotonic()

        if not self._joint_event.wait(timeout):
            return False

        elapsed = time.monotonic() - start
        remaining = max(0.0, timeout - elapsed)

        return self._imu_event.wait(remaining)


    def has_joint_data(self):
        return self._joint_event.is_set()


    def has_imu_data(self):
        return self._imu_event.is_set()


    # ============================================================
    # Articulaciones
    # ============================================================

    def get_joint_names(self):
        with self._lock:
            return list(self._joint_names)


    def _check_joint(self, name):
        if not self._joint_event.is_set():
            raise RuntimeError(
                "Todavia no se recibieron datos "
                "de articulaciones."
            )

        if name not in self._position:
            raise ValueError(
                f"Articulacion desconocida: {name}"
            )


    def get_joint_position(self, name):
        self._check_joint(name)

        with self._lock:
            return self._position[name]


    def get_joint_velocity(self, name):
        self._check_joint(name)

        with self._lock:
            return self._velocity[name]


    def get_joint_effort(self, name):
        self._check_joint(name)

        with self._lock:
            return self._effort[name]


    def get_joint_state(self, name):
        """
        Retorna q, dq y tau_est de una articulacion.
        """

        self._check_joint(name)

        with self._lock:
            return {
                "position": self._position[name],
                "velocity": self._velocity[name],
                "effort": self._effort[name],
            }


    def get_all_joint_states(self):
        if not self._joint_event.is_set():
            raise RuntimeError(
                "Todavia no se recibieron datos "
                "de articulaciones."
            )

        with self._lock:
            return {
                name: {
                    "position": self._position[name],
                    "velocity": self._velocity[name],
                    "effort": self._effort[name],
                }
                for name in self._joint_names
            }


    # ============================================================
    # IMU
    # ============================================================

    def get_imu(self):
        if not self._imu_event.is_set():
            raise RuntimeError(
                "Todavia no se recibieron datos de IMU."
            )

        with self._lock:
            return {
                section: dict(values)
                for section, values in self._imu.items()
            }


    # ============================================================
    # Cierre
    # ============================================================

    def close(self):
        self._running = False

        if (
            self._spin_thread.is_alive()
            and threading.current_thread()
            is not self._spin_thread
        ):
            self._spin_thread.join(
                timeout=1.0
            )

        self._node.destroy_node()

        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()


    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
