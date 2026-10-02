import time
import rclpy

from g1_interface.g1_low_level import G1LowLevel


def main():
    rclpy.init()

    robot = G1LowLevel()

    print("Esperando core...")
    print(robot.wait_for_core())

    time.sleep(1.0)

    print("Codo -> 1.5 rad")
    robot.set_joint("right_elbow", 1.5)
    time.sleep(3.0)

    print("Codo -> 0.7 rad")
    robot.set_joint("right_elbow", 0.7)
    time.sleep(3.0)

    print("Codo -> 1.5 rad")
    robot.set_joint("right_elbow", 1.5)
    time.sleep(3.0)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
