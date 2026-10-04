import threading
import time

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_,
    LowState_,
)
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


JOINTS = {
    "left_shoulder_pitch": 15,
    "left_shoulder_roll": 16,
    "left_shoulder_yaw": 17,
    "left_elbow": 18,
    "left_wrist_roll": 19,

    "right_shoulder_pitch": 22,
    "right_shoulder_roll": 23,
    "right_shoulder_yaw": 24,
    "right_elbow": 25,
    "right_wrist_roll": 26,
}

ARM_IDS = list(JOINTS.values())

# Unitree usa motor_cmd[29].q como weight:
# 1 = arm_sdk activo
# 0 = arm_sdk liberado
ARM_WEIGHT_ID = 29


class G1Arms:

    def __init__(
        self,
        control_dt=0.02,
        kp=60.0,
        kd=1.5,
    ):
        self.control_dt = control_dt
        self.kp = kp
        self.kd = kd

        self.low_state = None
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

        self.crc = CRC()
        self.lock = threading.Lock()

        self.targets = {}
        self.weight = 0.0

        self.thread = None
        self.started = False

        self.publisher = ChannelPublisher(
            "rt/arm_sdk",
            LowCmd_
        )
        self.publisher.Init()

        self.subscriber = ChannelSubscriber(
            "rt/lowstate",
            LowState_
        )
        self.subscriber.Init(
            self._lowstate_callback,
            10
        )

    # ---------------------------------------------------------
    # LOWSTATE
    # ---------------------------------------------------------

    def _lowstate_callback(self, msg):
        self.low_state = msg

    def wait_for_state(self, timeout=5.0):
        t0 = time.time()

        while self.low_state is None:
            if time.time() - t0 > timeout:
                return False

            time.sleep(0.05)

        return True

    def get_joint(self, name):
        if name not in JOINTS:
            raise ValueError(
                f"Joint desconocido: {name}"
            )

        if self.low_state is None:
            raise RuntimeError(
                "Todavia no hay LowState"
            )

        idx = JOINTS[name]

        return float(
            self.low_state.motor_state[idx].q
        )

    # ---------------------------------------------------------
    # LOOP ARM SDK
    # ---------------------------------------------------------

    def start(self):
        if self.started:
            return True

        print("[ARMS] Esperando rt/lowstate...")

        if not self.wait_for_state():
            print("[ARMS] No llega LowState")
            return False

        # Inicialmente mantenemos exactamente
        # las posiciones actuales.
        with self.lock:
            for name, idx in JOINTS.items():
                self.targets[name] = float(
                    self.low_state.motor_state[idx].q
                )

        self.thread = RecurrentThread(
            interval=self.control_dt,
            target=self._write,
            name="g1_arm_sdk"
        )

        self.thread.Start()
        self.started = True

        print("[ARMS] Loop arm_sdk iniciado")

        return True

    def _write(self):
        if self.low_state is None:
            return

        with self.lock:
            weight = self.weight
            targets = dict(self.targets)

        self.low_cmd.motor_cmd[
            ARM_WEIGHT_ID
        ].q = float(weight)

        for name, idx in JOINTS.items():

            q = targets.get(
                name,
                float(
                    self.low_state.motor_state[idx].q
                )
            )

            cmd = self.low_cmd.motor_cmd[idx]

            cmd.tau = 0.0
            cmd.q = float(q)
            cmd.dq = 0.0
            cmd.kp = float(self.kp)
            cmd.kd = float(self.kd)

        self.low_cmd.crc = self.crc.Crc(
            self.low_cmd
        )

        self.publisher.Write(
            self.low_cmd
        )

    # ---------------------------------------------------------
    # ENABLE / RELEASE
    # ---------------------------------------------------------

    def enable(self, duration=0.5):
        if not self.started:
            if not self.start():
                return False

        # Capturamos nuevamente las posiciones actuales
        # antes de dar autoridad al arm_sdk.
        with self.lock:
            for name, idx in JOINTS.items():
                self.targets[name] = float(
                    self.low_state.motor_state[idx].q
                )

        print("[ARMS] Enable arm_sdk")

        steps = max(
            1,
            int(duration / self.control_dt)
        )

        for i in range(steps + 1):
            ratio = i / steps

            with self.lock:
                self.weight = ratio

            time.sleep(self.control_dt)

        print("[ARMS] weight = 1.0")
        return True

    def release(self, duration=0.5):
        print("[ARMS] Release arm_sdk")

        steps = max(
            1,
            int(duration / self.control_dt)
        )

        with self.lock:
            initial = self.weight

        for i in range(steps + 1):
            ratio = i / steps

            with self.lock:
                self.weight = (
                    initial * (1.0 - ratio)
                )

            time.sleep(self.control_dt)

        with self.lock:
            self.weight = 0.0

        print("[ARMS] weight = 0.0")
        return True

    # ---------------------------------------------------------
    # MOVIMIENTO
    # ---------------------------------------------------------

    def move_joint(
        self,
        name,
        position,
        duration=1.0
    ):
        return self.move_joints(
            {name: position},
            duration
        )

    def move_joints(
        self,
        positions,
        duration=1.0
    ):
        for name in positions:
            if name not in JOINTS:
                raise ValueError(
                    f"Joint desconocido: {name}"
                )

        if not self.started:
            if not self.start():
                return False

        with self.lock:
            weight = self.weight

        if weight < 0.99:
            if not self.enable():
                return False

        starts = {
            name: self.get_joint(name)
            for name in positions
        }

        steps = max(
            1,
            int(duration / self.control_dt)
        )

        print(
            "[ARMS] Moviendo:",
            positions
        )

        for i in range(steps + 1):

            t = i / steps

            # smoothstep
            s = t * t * (3.0 - 2.0 * t)

            with self.lock:
                for name, target in positions.items():

                    q0 = starts[name]

                    self.targets[name] = (
                        q0
                        + (float(target) - q0) * s
                    )

            time.sleep(self.control_dt)

        with self.lock:
            for name, target in positions.items():
                self.targets[name] = float(target)

        return True
