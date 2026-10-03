import json
import time

from rclpy.node import Node

from unitree_api.msg import Request


class G1HighLevel(Node):

    API_SET_FSM_ID = 7101
    API_SET_VELOCITY = 7105

    FSM_DAMP = 1
    FSM_STAND = 4
    FSM_START = 500

    def __init__(self):
        super().__init__("g1_high_level")

        self.sport_pub = self.create_publisher(
            Request,
            "/api/sport/request",
            10
        )

        self.get_logger().info(
            "G1 High Level Node created"
        )

    def wait_for_core(self, timeout=5.0):
        start = time.time()

        while time.time() - start < timeout:
            if self.sport_pub.get_subscription_count() > 0:
                self.get_logger().info(
                    "G1 High Level connected"
                )
                return True

            time.sleep(0.1)

        self.get_logger().warning(
            "High Level backend not connected"
        )

        return False

    def _send(self, api_id, parameter=None):
        msg = Request()

        msg.header.identity.api_id = int(api_id)

        if parameter is None:
            parameter = {}

        msg.parameter = json.dumps(parameter)

        self.sport_pub.publish(msg)

        return True

    # --------------------------------------------------
    # Modos
    # --------------------------------------------------

    def damp(self):
        return self._send(
            self.API_SET_FSM_ID,
            {"data": self.FSM_DAMP}
        )

    def prepare(self):
        # API oficial G1: Start() -> FSM 500
        return self._send(
            self.API_SET_FSM_ID,
            {"data": self.FSM_START}
        )

    def stand(self):
        # API oficial G1: StandUp() -> FSM 4
        return self._send(
            self.API_SET_FSM_ID,
            {"data": self.FSM_STAND}
        )

    # --------------------------------------------------
    # Locomoción
    # --------------------------------------------------

    def velocity(
        self,
        vx,
        vy=0.0,
        wz=0.0,
        duration=864000.0
    ):
        return self._send(
            self.API_SET_VELOCITY,
            {
                "velocity": [
                    float(vx),
                    float(vy),
                    float(wz)
                ],
                "duration": float(duration)
            }
        )

    def walk(
        self,
        forward,
        lateral=0.0,
        turn=0.0,
        duration=864000.0
    ):
        return self.velocity(
            forward,
            lateral,
            turn,
            duration
        )

    def stop(self):
        return self.velocity(
            0.0,
            0.0,
            0.0,
            duration=1.0
        )
