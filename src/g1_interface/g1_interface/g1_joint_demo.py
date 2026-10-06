import time
import rclpy

from g1_interface.g1_sensors import G1Sensors
from g1_interface.g1_low_level import G1LowLevel

def update_sensors(sensors, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(sensors, timeout_sec=0.02)
        shoulder = sensors.get_joint_position('right_shoulder_roll')
        elbow = sensors.get_joint_position('right_elbow')
        if shoulder is not None and elbow is not None:
            print(f'Shoulder: {shoulder}rad | elbow: {elbow} rad', end='',flush=True)
        time.sleep(0.03)
    print()

def main(args=None):
    rclpy.init(args=args)
    sensors = G1Sensors()
    robot = G1LowLevel()

    try:
        if not robot.wait_for_core():
            print('G1 Core not available')
            return

        print('Waiting for robot state...')
        start_time = time.time()
        while sensors.get_joint_position('right_shoulder_roll') is None and time.time() - start_time < 5.0:
            rclpy.spin_once(sensors, timeout_sec=0.1)
        print()
        print('Moving right arm...')

        robot.set_joints({'right_shoulder_roll': -0.4,
                          'right_elbow': 0.7})
        update_sensors(sensors, 3.0)

        print()
        print('Returning home...')

        robot.set_joints({'right_shoulder_roll': 0.0,
                          'right_elbow': 0.0})
        update_sensors(sensors, 3.0)

        print()
        print('Demo finished.')

    except KeyboardInterrupt:
        pass
    finally:
        sensors.destroy_node()
        robot.destroy_node()

if __name__ == '__main__':
    main()
