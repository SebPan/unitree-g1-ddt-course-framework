import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from unitree_hg.msg import LowState
from sensor_msgs.msg import Image

import numpy as np

class G1Sensors(Node):
    JOINT_INDEX = {
        # Left leg
        "left_hip_pitch": 0,
        "left_hip_roll": 1,
        "left_hip_yaw": 2,
        "left_knee": 3,
        "left_ankle_pitch": 4,
        "left_ankle_roll": 5,

        # Right leg
        "right_hip_pitch": 6,
        "right_hip_roll": 7,
        "right_hip_yaw": 8,
        "right_knee": 9,
        "right_ankle_pitch": 10,
        "right_ankle_roll": 11,

        # Waist
        "waist_yaw": 12,

        # Left arm
        "left_shoulder_pitch": 15,
        "left_shoulder_roll": 16,
        "left_shoulder_yaw": 17,
        "left_elbow": 18,
        "left_wrist_roll": 19,

        # Right arm
        "right_shoulder_pitch": 22,
        "right_shoulder_roll": 23,
        "right_shoulder_yaw": 24,
        "right_elbow": 25,
        "right_wrist_roll": 26,
    }
    def __init__(self):
        super().__init__('g1_sensors')

        self.joints = {}
        self.imu = None
        self.camera_image = None

        self.camera_received = False
        self.joint_state_received = False
        self.imu_received = False

        self.low_state_sub = self.create_subscription(LowState, '/lowstate', self.low_state_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info('G1 Sensors Node Initialized')

    def low_state_callback(self, msg):
        self.joints = {}

        for name, index in self.JOINT_INDEX.items():
            motor = msg.motor_state[index]
            self.joints[name] = {
                'position': motor.q,
                'velocity': motor.dq,
                'effort': motor.tau_est,
            }

        imu = msg.imu_state
        self.imu = {
            'orientation': {
                'w': imu.quaternion[0],
                'x': imu.quaternion[1],
                'y': imu.quaternion[2],
                'z': imu.quaternion[3]
            },
            'angular_velocity': {
                'x': imu.gyroscope[0],
                'y': imu.gyroscope[1],
                'z': imu.gyroscope[2]
            },
            'linear_acceleration': {
                'x': imu.accelerometer[0],
                'y': imu.accelerometer[1],
                'z': imu.accelerometer[2]
            },
            'rpy' : {
                'roll': imu.rpy[0],
                'pitch': imu.rpy[1],
                'yaw': imu.rpy[2]
            }
        }

        if not self.joint_state_received:
            self.get_logger().info('Low State Received')

        self.joint_state_received = True
        self.imu_received = True

    def camera_callback(self, msg):
        if msg.encoding != 'rgb8':
            self.get_logger().info(f'Unsupported encoding: {msg.encoding}')
            return

        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape(msg.height, msg.width, 3)
        self.camera_image = image.copy()

        if not self.camera_received:
            self.get_logger().info(f'Camera Received: {msg.width} x {msg.height} {msg.encoding}')
        self.camera_received = True

    def get_camera_image(self):
        return self.camera_image

    def get_joint_position(self, joint_name):
        if joint_name not in self.joints:
            return None
        return self.joints[joint_name]['position']

    def get_joint_velocity(self, joint_name):
        if joint_name not in self.joints:
            return None
        return self.joints[joint_name]['velocity']

    def get_joint_effort(self, joint_name):
        if joint_name not in self.joints:
            return None
        return self.joints[joint_name]['effort']

    def get_imu(self):
        return self.imu

    def get_joint_names(self):
        return list(self.joints.keys())

    def print_status(self):
        print()
        print('==== G1 ====')
        if self.joint_state_received:
            elbow = self.get_joint_position('right_elbow')
            if elbow is not None:
                print('Right elbow', round(elbow, 2))
            else:
                print('Waiting joint states')

        if self.imu_received:
            print('Orientation:', {key: round(value, 3) for key, value in self.imu['orientation'].items()})
            print('Gyroscope:', {key: round(value, 3) for key, value in self.imu['angular_velocity'].items()})
            print('Acceleration:', {key: round(value, 3) for key, value in self.imu['linear_acceleration'].items()})
        else:
            print('Waiting IMU')

        print('=======================')

        if self.camera_received:
            print(f'Camera Received: {self.camera_image.shape} RGB')
        else:
            print('Waiting for camera')

def main(args=None):
    rclpy.init(args=args)
    node = G1Sensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
