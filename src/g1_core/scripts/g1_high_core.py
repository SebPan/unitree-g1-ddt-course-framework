#!/usr/bin/env python3

import os
import threading
import time

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)

from unitree_sdk2py.g1.loco.g1_loco_client import (
    LocoClient,
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


# ============================================================
# CONFIGURACION
# ============================================================

INTERFACE = os.getenv(
    "G1_INTERFACE",
    "eth2"
)


# ============================================================
# FSM
# ============================================================

FSM_ZERO = 0
FSM_DAMP = 1
FSM_STAND = 4
FSM_WALK = 500


# ============================================================
# G1 EDU 23 DOF - BRAZOS
# ============================================================

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

ARM_WEIGHT_ID = 29


# ============================================================
# LOCOMOTION
# ============================================================

class G1Locomotion:

    def __init__(self):

        self.lock = threading.RLock()

        self.motion_switcher = MotionSwitcherClient()
        self.motion_switcher.SetTimeout(5.0)
        self.motion_switcher.Init()

        self.loco = LocoClient()
        self.loco.SetTimeout(10.0)
        self.loco.Init()

    # --------------------------------------------------------

    def get_fsm(self):

        with self.lock:

            try:
                return self.loco.GetFsmId()

            except Exception as e:
                print(
                    "[LOCO] GetFsmId error:",
                    e
                )

                return -1, None

    # --------------------------------------------------------

    def get_motion_mode(self):

        with self.lock:

            try:

                code, mode = (
                    self.motion_switcher.CheckMode()
                )

                if code == 0 and mode:

                    return mode.get(
                        "name"
                    )

            except Exception as e:

                print(
                    "[LOCO] CheckMode error:",
                    e
                )

        return None

    # --------------------------------------------------------

    def wait_fsm(
        self,
        expected,
        timeout=5.0
    ):

        start = time.time()

        while time.time() - start < timeout:

            code, fsm = self.get_fsm()

            if (
                code == 0
                and fsm == expected
            ):
                print(
                    f"[LOCO] FSM confirmado: "
                    f"{fsm}"
                )

                return True

            time.sleep(0.1)

        code, fsm = self.get_fsm()

        print(
            f"[LOCO] Timeout esperando "
            f"FSM {expected}. Actual={fsm}"
        )

        return False

    # --------------------------------------------------------

    def set_fsm(
        self,
        target
    ):

        print(
            f"[LOCO] SetFsmId({target})"
        )

        with self.lock:

            ret = self.loco.SetFsmId(
                int(target)
            )

        if ret != 0:

            print(
                "[LOCO] SetFsmId retorno:",
                ret
            )

            return False

        return self.wait_fsm(
            target
        )

    # --------------------------------------------------------

    def stop(self):

        print("[LOCO] STOP")

        with self.lock:

            ret = self.loco.SetVelocity(
                0.0,
                0.0,
                0.0,
                1.0,
            )

        return ret == 0

    # --------------------------------------------------------

    def damp(self):

        print("[LOCO] DAMP")

        self.stop()

        time.sleep(0.2)

        return self.set_fsm(
            FSM_DAMP
        )

    # --------------------------------------------------------

    def stand(self):

        code, fsm = self.get_fsm()

        if code != 0:
            return False

        print(
            f"[LOCO] STAND desde FSM {fsm}"
        )

        # Ya estamos en Stand
        if fsm == FSM_STAND:

            return True

        # WALK -> STOP -> DAMP -> STAND
        if fsm == FSM_WALK:

            print(
                "[LOCO] "
                "WALK -> STOP -> DAMP -> STAND"
            )

            self.stop()

            time.sleep(0.5)

            if not self.set_fsm(
                FSM_DAMP
            ):
                return False

            time.sleep(1.0)

        # ZERO -> DAMP -> STAND
        elif fsm == FSM_ZERO:

            print(
                "[LOCO] "
                "ZERO -> DAMP -> STAND"
            )

            if not self.set_fsm(
                FSM_DAMP
            ):
                return False

            time.sleep(1.0)

        # Cualquier otro estado -> DAMP
        elif fsm != FSM_DAMP:

            print(
                f"[LOCO] FSM {fsm} "
                "-> DAMP -> STAND"
            )

            if not self.set_fsm(
                FSM_DAMP
            ):
                return False

            time.sleep(1.0)

        return self.set_fsm(
            FSM_STAND
        )

    # --------------------------------------------------------

    def prepare_walk(self):

        code, fsm = self.get_fsm()

        if code != 0:
            return False

        if fsm == FSM_WALK:

            return True

        if fsm != FSM_STAND:

            if not self.stand():
                return False

            time.sleep(0.5)

        return self.set_fsm(
            FSM_WALK
        )

    # --------------------------------------------------------

    def velocity(
        self,
        vx,
        vy,
        wz
    ):

        if not self.prepare_walk():

            return False

        print(
            f"[LOCO] VELOCITY "
            f"vx={vx:.3f} "
            f"vy={vy:.3f} "
            f"wz={wz:.3f}"
        )

        with self.lock:

            ret = self.loco.SetVelocity(
                float(vx),
                float(vy),
                float(wz),
                864000.0,
            )

        return ret == 0

    # --------------------------------------------------------

    def zero_torque(self):

        self.stop()

        time.sleep(0.2)

        return self.set_fsm(
            FSM_ZERO
        )


# ============================================================
# BRAZOS
# ============================================================

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

        self.low_cmd = (
            unitree_hg_msg_dds__LowCmd_()
        )

        self.crc = CRC()

        self.lock = threading.Lock()

        self.targets = {}

        self.weight = 0.0

        self.started = False
        self.thread = None

        # ----------------------------------------------------
        # ARM SDK publisher
        # ----------------------------------------------------

        self.publisher = ChannelPublisher(
            "rt/arm_sdk",
            LowCmd_
        )

        self.publisher.Init()

        # ----------------------------------------------------
        # Robot state
        # ----------------------------------------------------

        self.subscriber = ChannelSubscriber(
            "rt/lowstate",
            LowState_
        )

        self.subscriber.Init(
            self._lowstate_callback,
            10
        )

    # --------------------------------------------------------

    def _lowstate_callback(
        self,
        msg
    ):

        self.low_state = msg

    # --------------------------------------------------------

    def wait_for_state(
        self,
        timeout=5.0
    ):

        start = time.time()

        while self.low_state is None:

            if (
                time.time() - start
                > timeout
            ):

                return False

            time.sleep(0.05)

        return True

    # --------------------------------------------------------

    def get_joint(
        self,
        name
    ):

        if name not in JOINTS:

            raise ValueError(
                f"Joint desconocido: {name}"
            )

        if self.low_state is None:

            raise RuntimeError(
                "No hay LowState"
            )

        idx = JOINTS[name]

        return float(
            self.low_state
            .motor_state[idx]
            .q
        )

    # --------------------------------------------------------
    # 50 Hz ARM LOOP
    # --------------------------------------------------------

    def start(self):

        if self.started:

            return True

        print(
            "[ARMS] Esperando rt/lowstate..."
        )

        if not self.wait_for_state():

            print(
                "[ARMS] No llega LowState"
            )

            return False

        # Partimos exactamente de la
        # posición actual del robot.
        with self.lock:

            for name, idx in JOINTS.items():

                self.targets[name] = float(
                    self.low_state
                    .motor_state[idx]
                    .q
                )

        self.thread = RecurrentThread(
            interval=self.control_dt,
            target=self._write,
            name="g1_arm_sdk",
        )

        self.thread.Start()

        self.started = True

        print(
            "[ARMS] arm_sdk loop iniciado"
        )

        return True

    # --------------------------------------------------------

    def _write(self):

        if self.low_state is None:

            return

        with self.lock:

            weight = self.weight
            targets = dict(
                self.targets
            )

        # Unitree arm_sdk weight
        self.low_cmd.motor_cmd[
            ARM_WEIGHT_ID
        ].q = float(weight)

        for name, idx in JOINTS.items():

            q = targets.get(
                name,
                float(
                    self.low_state
                    .motor_state[idx]
                    .q
                )
            )

            cmd = self.low_cmd.motor_cmd[
                idx
            ]

            cmd.tau = 0.0
            cmd.q = float(q)
            cmd.dq = 0.0
            cmd.kp = float(self.kp)
            cmd.kd = float(self.kd)

        self.low_cmd.crc = (
            self.crc.Crc(
                self.low_cmd
            )
        )

        self.publisher.Write(
            self.low_cmd
        )

    # --------------------------------------------------------

    def enable(
        self,
        duration=0.5
    ):

        if not self.started:

            if not self.start():

                return False

        # Capturar posición actual antes
        # de asumir autoridad.
        with self.lock:

            for name, idx in JOINTS.items():

                self.targets[name] = float(
                    self.low_state
                    .motor_state[idx]
                    .q
                )

        print(
            "[ARMS] ENABLE"
        )

        steps = max(
            1,
            int(
                duration /
                self.control_dt
            )
        )

        for i in range(
            steps + 1
        ):

            ratio = i / steps

            with self.lock:

                self.weight = ratio

            time.sleep(
                self.control_dt
            )

        with self.lock:
            self.weight = 1.0

        print(
            "[ARMS] weight = 1.0"
        )

        return True

    # --------------------------------------------------------

    def release(
        self,
        duration=0.5
    ):

        if not self.started:

            return True

        print(
            "[ARMS] RELEASE"
        )

        steps = max(
            1,
            int(
                duration /
                self.control_dt
            )
        )

        with self.lock:
            initial = self.weight

        for i in range(
            steps + 1
        ):

            ratio = i / steps

            with self.lock:

                self.weight = (
                    initial *
                    (1.0 - ratio)
                )

            time.sleep(
                self.control_dt
            )

        with self.lock:
            self.weight = 0.0

        print(
            "[ARMS] weight = 0.0"
        )

        return True

    # --------------------------------------------------------

    def move_joint(
        self,
        name,
        position,
        duration=1.0
    ):

        return self.move_joints(
            {
                name: position
            },
            duration
        )

    # --------------------------------------------------------

    def move_joints(
        self,
        positions,
        duration=1.0
    ):

        for name in positions:

            if name not in JOINTS:

                raise ValueError(
                    f"Joint desconocido: "
                    f"{name}"
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
            int(
                duration /
                self.control_dt
            )
        )

        print(
            "[ARMS] Movimiento:",
            positions
        )

        for i in range(
            steps + 1
        ):

            t = i / steps

            # Smoothstep
            s = (
                t * t *
                (3.0 - 2.0 * t)
            )

            with self.lock:

                for (
                    name,
                    target
                ) in positions.items():

                    q0 = starts[name]

                    self.targets[name] = (
                        q0
                        +
                        (
                            float(target)
                            - q0
                        )
                        * s
                    )

            time.sleep(
                self.control_dt
            )

        with self.lock:

            for (
                name,
                target
            ) in positions.items():

                self.targets[name] = (
                    float(target)
                )

        return True


# ============================================================
# ROS HIGH CORE
# ============================================================

class G1HighCore(Node):

    def __init__(self):

        super().__init__(
            "g1_high_core"
        )

        self.callback_group = (
            ReentrantCallbackGroup()
        )

        self.get_logger().info(
            f"Unitree interface: "
            f"{INTERFACE}"
        )

        # ----------------------------------------------------
        # Unitree DDS
        # ----------------------------------------------------

        ChannelFactoryInitialize(
            0,
            INTERFACE
        )

        self.locomotion = (
            G1Locomotion()
        )

        self.arms = (
            G1Arms()
        )

        # ====================================================
        # ROS INPUT
        # ====================================================

        self.mode_sub = (
            self.create_subscription(
                String,
                "/g1/high/mode",
                self.mode_callback,
                10,
                callback_group=
                    self.callback_group,
            )
        )

        self.velocity_sub = (
            self.create_subscription(
                Twist,
                "/g1/high/cmd_vel",
                self.velocity_callback,
                10,
                callback_group=
                    self.callback_group,
            )
        )

        self.arm_trajectory_sub = (
            self.create_subscription(
                JointTrajectory,
                "/g1/high/arm_trajectory",
                self.arm_trajectory_callback,
                10,
                callback_group=
                    self.callback_group,
            )
        )

        self.arm_mode_sub = (
            self.create_subscription(
                String,
                "/g1/high/arm_mode",
                self.arm_mode_callback,
                10,
                callback_group=
                    self.callback_group,
            )
        )

        # ====================================================
        # ROS OUTPUT
        # ====================================================

        self.fsm_pub = (
            self.create_publisher(
                Int32,
                "/g1/high/fsm",
                10
            )
        )

        self.motion_mode_pub = (
            self.create_publisher(
                String,
                "/g1/high/motion_mode",
                10
            )
        )

        self.result_pub = (
            self.create_publisher(
                String,
                "/g1/high/result",
                10
            )
        )

        # ----------------------------------------------------

        self.timer = self.create_timer(
            1.0,
            self.publish_state,
            callback_group=
                self.callback_group,
        )

        self.get_logger().info(
            "G1 High Core ready"
        )

    # ========================================================
    # RESULT
    # ========================================================

    def publish_result(
        self,
        text
    ):

        msg = String()
        msg.data = str(text)

        self.result_pub.publish(
            msg
        )

    # ========================================================
    # MODE
    # ========================================================

    def mode_callback(
        self,
        msg
    ):

        command = (
            msg.data
            .strip()
            .lower()
        )

        self.get_logger().info(
            f"MODE: {command}"
        )

        ok = False

        if command == "stand":

            ok = (
                self.locomotion.stand()
            )

        elif command == "damp":

            ok = (
                self.locomotion.damp()
            )

        elif command == "stop":

            ok = (
                self.locomotion.stop()
            )

        elif command == "zero_torque":

            ok = (
                self.locomotion
                .zero_torque()
            )

        else:

            self.get_logger().warning(
                f"Modo desconocido: "
                f"{command}"
            )

            return

        self.publish_result(
            f"{command}: "
            f"{'OK' if ok else 'ERROR'}"
        )

    # ========================================================
    # VELOCITY
    # ========================================================

    def velocity_callback(
        self,
        msg
    ):

        vx = float(
            msg.linear.x
        )

        vy = float(
            msg.linear.y
        )

        wz = float(
            msg.angular.z
        )

        # Twist cero = STOP
        if (
            abs(vx) < 1e-6
            and abs(vy) < 1e-6
            and abs(wz) < 1e-6
        ):

            ok = (
                self.locomotion.stop()
            )

        else:

            ok = (
                self.locomotion.velocity(
                    vx,
                    vy,
                    wz
                )
            )

        self.publish_result(
            "velocity: "
            + (
                "OK"
                if ok
                else "ERROR"
            )
        )

    # ========================================================
    # ARMS
    # ========================================================

    def arm_mode_callback(
        self,
        msg
    ):

        command = (
            msg.data
            .strip()
            .lower()
        )

        self.get_logger().info(
            f"ARM MODE: {command}"
        )

        if command == "enable":

            ok = self.arms.enable()

        elif command == "release":

            ok = self.arms.release()

        else:

            self.get_logger().warning(
                f"Arm mode desconocido: "
                f"{command}"
            )

            return

        self.publish_result(
            f"arm_{command}: "
            f"{'OK' if ok else 'ERROR'}"
        )

    # --------------------------------------------------------

    def arm_trajectory_callback(
        self,
        msg
    ):

        if not msg.points:

            self.get_logger().warning(
                "JointTrajectory sin puntos"
            )

            return

        point = msg.points[-1]

        if (
            len(msg.joint_names)
            != len(point.positions)
        ):

            self.get_logger().error(
                "joint_names y positions "
                "tienen distinta longitud"
            )

            return

        positions = dict(
            zip(
                msg.joint_names,
                point.positions
            )
        )

        duration = (
            float(
                point.time_from_start.sec
            )
            +
            float(
                point.time_from_start.nanosec
            )
            * 1e-9
        )

        if duration <= 0.0:
            duration = 1.0

        try:

            ok = self.arms.move_joints(
                positions,
                duration
            )

        except Exception as e:

            self.get_logger().error(
                f"Arm trajectory: {e}"
            )

            ok = False

        self.publish_result(
            "arm_trajectory: "
            + (
                "OK"
                if ok
                else "ERROR"
            )
        )

    # ========================================================
    # STATE
    # ========================================================

    def publish_state(
        self
    ):

        code, fsm = (
            self.locomotion.get_fsm()
        )

        if code == 0:

            msg = Int32()
            msg.data = int(fsm)

            self.fsm_pub.publish(
                msg
            )

        mode = (
            self.locomotion
            .get_motion_mode()
        )

        if mode is not None:

            msg = String()
            msg.data = str(mode)

            self.motion_mode_pub.publish(
                msg
            )


# ============================================================
# MAIN
# ============================================================

def main():

    # SDK2 puede modificar sys.argv.
    rclpy.init(args=[])

    node = G1HighCore()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(
        node
    )

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
