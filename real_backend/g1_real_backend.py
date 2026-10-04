#!/usr/bin/env python3

import argparse
import json
import socket

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.g1.loco.g1_loco_client import (
    LocoClient,
)

from locomotion import G1Locomotion
from arms import G1Arms


class G1RealBackend:

    def __init__(self, interface):
        print(f"[REAL] Interface: {interface}")

        # IMPORTANTE:
        # Solo una inicializacion DDS por proceso.
        ChannelFactoryInitialize(
            0,
            interface
        )

        # --------------------------------------------------
        # LOCOMOTION
        # --------------------------------------------------

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()

        self.loco = G1Locomotion(
            self.msc,
            self.loco_client
        )

        # --------------------------------------------------
        # BRAZOS
        # --------------------------------------------------

        self.arms = G1Arms()

        print(
            "[REAL] Estado inicial:",
            self.loco.status()
        )

    # ======================================================
    # STATUS
    # ======================================================

    def status(self):
        return {
            "ok": True,
            "locomotion": self.loco.status(),
        }

    # ======================================================
    # LOCOMOTION
    # ======================================================

    def damp(self):
        ok = self.loco.damp()

        return {
            "ok": ok,
            "fsm": self.loco.status()["fsm"],
        }

    def stand(self):
        ok = self.loco.stand()

        return {
            "ok": ok,
            "fsm": self.loco.status()["fsm"],
        }

    def velocity(
        self,
        vx,
        vy=0.0,
        wz=0.0,
        duration=1.0,
    ):
        ok = self.loco.velocity(
            vx,
            vy,
            wz,
            duration
        )

        return {
            "ok": ok,
            "fsm": self.loco.status()["fsm"],
        }

    def stop(self):
        ok = self.loco.stop()

        return {
            "ok": ok,
            "fsm": self.loco.status()["fsm"],
        }

    def zero_torque(self):
        ok = self.loco.zero_torque()

        return {
            "ok": ok,
            "fsm": self.loco.status()["fsm"],
        }

    # ======================================================
    # BRAZOS
    # ======================================================

    def arm_enable(self):
        ok = self.arms.enable()

        return {
            "ok": ok,
        }

    def arm_release(self):
        ok = self.arms.release()

        return {
            "ok": ok,
        }

    def arm_joint(
        self,
        name,
        position,
        duration=1.0
    ):
        ok = self.arms.move_joint(
            name,
            position,
            duration
        )

        return {
            "ok": ok,
        }

    def arm_move(
        self,
        joints,
        duration=1.0
    ):
        ok = self.arms.move_joints(
            joints,
            duration
        )

        return {
            "ok": ok,
        }

    # ======================================================
    # DISPATCH
    # ======================================================

    def execute(self, request):

        cmd = request.get("cmd")

        if cmd == "status":
            return self.status()

        if cmd == "damp":
            return self.damp()

        if cmd == "stand":
            return self.stand()

        if cmd == "prepare":
            return self.stand()

        if cmd == "velocity":
            return self.velocity(
                request.get("vx", 0.0),
                request.get("vy", 0.0),
                request.get("wz", 0.0),
                request.get("duration", 1.0),
            )

        if cmd == "stop":
            return self.stop()

        if cmd == "zero_torque":
            return self.zero_torque()

        if cmd == "arm_enable":
            return self.arm_enable()

        if cmd == "arm_release":
            return self.arm_release()

        if cmd == "arm_joint":
            return self.arm_joint(
                request["name"],
                request["position"],
                request.get("duration", 1.0),
            )

        if cmd == "arm_move":
            return self.arm_move(
                request["joints"],
                request.get("duration", 1.0),
            )

        return {
            "ok": False,
            "error": f"Unknown command: {cmd}",
        }


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--interface",
        required=True
    )

    parser.add_argument(
        "--port",
        type=int,
        default=15030
    )

    args = parser.parse_args()

    backend = G1RealBackend(
        args.interface
    )

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    sock.bind(
        ("127.0.0.1", args.port)
    )

    print(
        f"[REAL] Backend READY "
        f"127.0.0.1:{args.port}"
    )

    while True:

        data, addr = sock.recvfrom(65535)

        try:
            request = json.loads(
                data.decode()
            )

            print()
            print("[REAL] RX:", request)

            response = backend.execute(
                request
            )

        except Exception as e:

            response = {
                "ok": False,
                "error": repr(e),
            }

        response["id"] = request.get("id")

        print("[REAL] TX:", response)

        sock.sendto(
            json.dumps(response).encode(),
            addr
        )


if __name__ == "__main__":
    main()
