import json
import os
import time

from rclpy.node import Node

from unitree_api.msg import Request
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class G1HighLevel(Node):

    # ========================================================
    # SIM SPORT API
    # ========================================================

    API_SET_FSM_ID = 7101
    API_SET_VELOCITY = 7105

    FSM_DAMP = 1
    FSM_STAND = 4
    FSM_START = 500

    # ========================================================

    def __init__(self):
        super().__init__("g1_high_level")

        self.backend = os.getenv(
            "G1_MODE",
            "sim"
        ).strip().lower()

        if self.backend not in ("sim", "real"):
            self.backend = "sim"

        # ----------------------------------------------------
        # SIM
        # ----------------------------------------------------

        self.sport_pub = self.create_publisher(
            Request,
            "/api/sport/request",
            10
        )

        # ----------------------------------------------------
        # REAL
        # ----------------------------------------------------

        self.mode_pub = self.create_publisher(
            String,
            "/g1/high/mode",
            10
        )

        self.velocity_pub = self.create_publisher(
            Twist,
            "/g1/high/cmd_vel",
            10
        )

        self.arm_mode_pub = self.create_publisher(
            String,
            "/g1/high/arm_mode",
            10
        )

        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/g1/high/arm_trajectory",
            10
        )

        self.get_logger().info(
            f"G1 High Level created - backend={self.backend}"
        )

    # ========================================================
    # CONNECTION
    # ========================================================

    def wait_for_core(self, timeout=5.0):

        start = time.time()

        while time.time() - start < timeout:

            if self.backend == "real":

                if (
                    self.mode_pub.get_subscription_count() > 0
                    and
                    self.velocity_pub.get_subscription_count() > 0
                ):

                    self.get_logger().info(
                        "G1 High Level REAL connected"
                    )

                    return True

            else:

                if self.sport_pub.get_subscription_count() > 0:

                    self.get_logger().info(
                        "G1 High Level SIM connected"
                    )

                    return True

            time.sleep(0.1)

        self.get_logger().warning(
            f"High Level backend not connected: {self.backend}"
        )

        return False

    # ========================================================
    # SIM
    # ========================================================

    def _send_sim(
        self,
        api_id,
        parameter=None
    ):

        msg = Request()

        msg.header.identity.api_id = int(api_id)

        if parameter is None:
            parameter = {}

        msg.parameter = json.dumps(parameter)

        self.sport_pub.publish(msg)

        return True

    # ========================================================
    # REAL
    # ========================================================

    def _send_mode_real(
        self,
        command
    ):

        msg = String()
        msg.data = str(command)

        self.mode_pub.publish(msg)

        return True

    def _send_velocity_real(
        self,
        vx,
        vy,
        wz
    ):

        msg = Twist()

        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)

        self.velocity_pub.publish(msg)

        return True

    # ========================================================
    # MODES
    # ========================================================

    def damp(self):

        if self.backend == "real":
            return self._send_mode_real(
                "damp"
            )

        return self._send_sim(
            self.API_SET_FSM_ID,
            {
                "data": self.FSM_DAMP
            }
        )

    def prepare(self):

        if self.backend == "real":
            return self._send_mode_real(
                "prepare"
            )

        return self._send_sim(
            self.API_SET_FSM_ID,
            {
                "data": self.FSM_START
            }
        )

    def stand(self):

        if self.backend == "real":
            return self._send_mode_real(
                "stand"
            )

        return self._send_sim(
            self.API_SET_FSM_ID,
            {
                "data": self.FSM_STAND
            }
        )

    # ========================================================
    # LOCOMOTION
    # ========================================================

    def velocity(
        self,
        vx,
        vy=0.0,
        wz=0.0,
        duration=864000.0
    ):

        if self.backend == "real":

            return self._send_velocity_real(
                vx,
                vy,
                wz
            )

        return self._send_sim(
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

        if self.backend == "real":
            return self._send_mode_real(
                "stop"
            )

        return self.velocity(
            0.0,
            0.0,
            0.0,
            duration=1.0
        )


    # ========================================================
    # BRAZOS HIGH LEVEL
    # ========================================================

    def arm_enable(self):

        if self.backend != "real":
            self.get_logger().warning(
                "arm_enable(): backend SIM aun no implementado"
            )
            return False

        msg = String()
        msg.data = "enable"

        self.arm_mode_pub.publish(msg)

        return True

    def arm_release(self):

        if self.backend != "real":
            self.get_logger().warning(
                "arm_release(): backend SIM aun no implementado"
            )
            return False

        msg = String()
        msg.data = "release"

        self.arm_mode_pub.publish(msg)

        return True

    def arm_joint(
        self,
        name,
        position,
        duration=1.0
    ):

        return self.move_arms(
            {
                name: position
            },
            duration=duration
        )

    def move_arms(
        self,
        positions,
        duration=1.0
    ):

        if self.backend != "real":
            self.get_logger().warning(
                "move_arms(): backend SIM aun no implementado"
            )
            return False

        if not positions:
            return False

        duration = max(
            0.02,
            float(duration)
        )

        msg = JointTrajectory()

        msg.joint_names = list(
            positions.keys()
        )

        point = JointTrajectoryPoint()

        point.positions = [
            float(positions[name])
            for name in msg.joint_names
        ]

        sec = int(duration)

        nanosec = int(
            (duration - sec)
            * 1_000_000_000
        )

        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        msg.points.append(point)

        self.arm_trajectory_pub.publish(msg)

        return True
