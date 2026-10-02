import time

import rclpy
from rclpy.node import Node

from g1_core.msg import JointCommand


class G1LowLevel(Node):

    JOINTS = [
        "left_hip_pitch",
        "left_hip_roll",
        "left_hip_yaw",
        "left_knee",
        "left_ankle_pitch",
        "left_ankle_roll",

        "right_hip_pitch",
        "right_hip_roll",
        "right_hip_yaw",
        "right_knee",
        "right_ankle_pitch",
        "right_ankle_roll",

        "waist_yaw",

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
    def __init__(self):
        super().__init__('g1_low_level')

        self.command_pub = self.create_publisher(JointCommand, '/g1/joint_command', 10)
        self.get_logger().info('G1 Low Level Node created')

    def wait_for_core(self, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.command_pub.get_subscription_count() > 0:
                self.get_logger().info('G1 Core connected')
                return True

        self.get_logger().warning('G1 Core not connected')
        return False

    def set_joint(self, joint_name, position, velocity=None, torque=None, kp=None, kd=None):
        if joint_name not in self.JOINTS:
            print(f'Joint not found: {joint_name}')
            return False
        msg = JointCommand()
        msg.name = [joint_name]
        msg.position = [float(position)]
        msg.velocity = [float('nan') if velocity is None else float(velocity)]
        msg.torque = [float('nan') if torque is None else float(torque)]
        msg.kp = [float('nan') if kp is None else float(kp)]
        msg.kd = [float('nan') if kd is None else float(kd)]
        self.command_pub.publish(msg)
        return True

    def set_joints(self, joints, velocity=None, torque=None, kp=None, kd=None):
        names = []
        positions = []
        velocities = []
        torques = []
        kps = []
        kds = []

        velocity = velocity or {}
        torque = torque or {}
        kp = kp or {}
        kd = kd or {}

        for name, position in joints.items():
            if name not in self.JOINTS:
                print(f'Joint not found: {name}')
                continue

            names.append(name)
            positions.append(float(position))
            velocities.append(float(velocity[name]) if name in velocity else float('nan'))
            torques.append(float(torque[name]) if name in torque else float('nan'))
            kps.append(float(kp[name]) if name in kp else float('nan'))
            kds.append(float(kd[name]) if name in kd else float('nan'))

        if len(names) == 0:
            return False

        msg = JointCommand()
        msg.name = names
        msg.position = positions
        msg.velocity = velocities
        msg.torque = torques
        msg.kp = kps
        msg.kd = kds
        
        self.command_pub.publish(msg)
        return True

    def get_joint_names(self):
        return self.JOINTS.copy()


def main(args=None):
    rclpy.init(args=args)
    node = G1LowLevel()

    try:
        if not node.wait_for_core():
            return
        print()
        print('Moving right arm')
        print()

        node.set_joints({'right_shoulder_roll': -0.4, 'right_elbow': 0.7})
        time.sleep(3)

        print('Returning home...')
        node.set_joints({'right_shoulder_roll': 0.0, 'right_elbow': 0.0})
        time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
