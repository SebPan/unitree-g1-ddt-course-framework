import rclpy
from rclpy.node import Node

import json

from unitree_api.msg import Request
from unitree_hg.msg import LowCmd, LowState


class G1HighLevel(Node):
    JOINTS = [
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_shoulder_yaw",
        "left_elbow",
        "left_wrist_roll",

        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_shoulder_yaw",
        "right_elbow",
        "right_wrist_roll"
    ]
    MOTOR_ID = {
        "left_shoulder_pitch": 15,
        "left_shoulder_roll": 16,
        "left_shoulder_yaw": 17,
        "left_elbow": 18,
        "left_wrist_roll": 19,

        "right_shoulder_pitch": 22,
        "right_shoulder_roll": 23,
        "right_shoulder_yaw": 24,
        "right_elbow": 25,
        "right_wrist_roll": 26
    }

    ARM_WEIGHT_MOTOR = 29

    ARM_KP = 60.0
    ARM_KD = 1.5

    def __init__(self):
        super().__init__('g1_high_level')

        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.arm_pub = self.create_publisher(LowCmd, '/arm_sdk', 10)
        self.low_state_sub = self.create_subscription(LowState, '/lowstate', self._low_state_callback, 10)

        self.arm_position = {name: 0.0 for name in self.JOINTS}
        self.state_received = False
        self.arm_active =  False
        self.arm_weight = 0.0
        self.arm_releasing = False
        self.arm_release_step = 0.01

        self.arm_timer = self.create_timer(0.02, self._publish_arm_command)

    def wait_until_ready(self, timeout=10.0):
        import time
        start = time.time()
        self.get_logger().info('Waiting for G1 High Level interface...')

        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            sport_ready = (self.sport_pub.get_subscription_count() > 0)
            arm_ready = (self.arm_pub.get_subscription_count() > 0)
            state_ready = self.state_received

            if sport_ready and state_ready and arm_ready:
                self.get_logger().info('G1 High Level ready.')
                return True
        self.get_logger().warning('G1 High level not ready')
        self.get_logger().warning(f'Sport: {self.sport_pub.get_subscription_count()} | '
                                  f'Arm: {self.arm_pub.get_subscription_count()} | '
                                  f'LowState: {self.state_received}')
        return False

    def wait_for_robot(self,timeout_sec=10.0):
        return self.wait_until_ready(timeout_sec)

    def _low_state_callback(self, msg):
        if self.state_received:
            return

        for name in self.JOINTS:
            motor = self.MOTOR_ID[name]
            self.arm_position[name] = float(msg.motor_state[motor].q)

        if not self.state_received:
            self.state_received = True
            self.get_logger().info("Arm LowState received")

    def _publish_arm_command(self):
        if not self.arm_active and not self.arm_releasing:
            return

        if self.arm_releasing:
            self.arm_weight -= self.arm_release_step
            if self.arm_weight < 0.0:
                self.arm_weight = 0.0

        msg = LowCmd()
        for name in self.JOINTS:
            motor = self.MOTOR_ID[name]
            msg.motor_cmd[motor].q = float(self.arm_position[name])
            msg.motor_cmd[motor].dq = 0.0
            msg.motor_cmd[motor].tau = 0.0
            msg.motor_cmd[motor].kp = self.ARM_KP
            msg.motor_cmd[motor].kd = self.ARM_KD
        msg.motor_cmd[self.ARM_WEIGHT_MOTOR].q = float(self.arm_weight)
        self.arm_pub.publish(msg)
        if self.arm_releasing and self.arm_weight <= 0:
            self.arm_active = False
            self.arm_releasing = False
            self.get_logger().info('Arm control released')

    def _send_sport_request(self, api_id, parameter=None):
        msg = Request()
        msg.header.identity.api_id = api_id
        if parameter is not None:
            msg.parameter = json.dumps(parameter)
        self.sport_pub.publish(msg)
        return True

    def stand(self):
        return self._send_sport_request(7101,{"data": 4})

    def walk(self, forward, lateral=0.0, turn=0.0, duration=864000.0):
        return self._send_sport_request(7105,
                                        {"velocity": [float(forward), float(lateral), float(turn)], "duration": float(duration)})

    def set_joint(self, joint_name, position):
        if joint_name not in self.JOINTS:
            print(f'Joint not available in high level: {joint_name}')
            return False

        if not self.state_received:
            self.get_logger().info('Waiting for LowState')
            return False

        self.arm_position[joint_name] = float(position)
        self.arm_weight = 1.0
        self.arm_releasing = False
        self.arm_active = True
        return True

    def set_joints(self, joints):
        if not self.state_received:
            self.get_logger().info('Waiting for LowState')
            return False

        valid = False

        for name, position in joints.items():
            if name not in self.JOINTS:
                print(f'Joint not available in high level: {name}')
                continue

            self.arm_position[name] = float(position)
            valid = True

        if valid:
            self.arm_weight = 1.0
            self.arm_active = True
            self.arm_releasing = False

        return valid

    def stop(self):
        return self._send_sport_request(7105,{"velocity": [0.0, 0.0, 0.0], 'duration': 1.0})

    def release_joints(self):
        if not self.arm_active:
            self.get_logger().info('Arm control already released')
            return True

        self.arm_releasing = True
        self.get_logger().info('Releasing arm control')
        return True

    def get_joint_names(self):
        return self.JOINTS.copy()

def main(args=None):
    rclpy.init(args=args)
    robot = G1HighLevel()

    try:
        if not robot.wait_for_core():
            return

        robot.get_logger().info('High-level interface ready.')
        rclpy.spin(robot)

    except KeyboardInterrupt:
        pass

    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()