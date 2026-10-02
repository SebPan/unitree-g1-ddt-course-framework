import time
import rclpy
from g1_interface.g1_high_level import G1HighLevel

def spin_for(robot, seconds):
    start = time.time()
    while time.time() - start < seconds:
        rclpy.spin_once(robot, timeout_sec=0.02)

def main():
    rclpy.init()
    node = G1HighLevel()
    if not node.wait_until_ready():
        print("Robot not ready yet.")
        return

    print("Waiting for LowState")
    while not node.state_received:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("LowState received")

    print("Standing")
    node.stand()
    spin_for(node, seconds=3.0)

    print("Moving elbow to 0.8 rad")
    node.set_joint('right_elbow', 0.8)
    spin_for(node, seconds=3.0)

    print("Moving elbow to 1.0 rad")
    node.set_joint('right_elbow', 1.0)
    spin_for(node, seconds=3.0)

    print('Walking...')
    node.walk(0.2, 0.0, 0.0)
    spin_for(node, seconds=4.0)

    print('Moving elbow while walking')
    node.set_joint('right_elbow', 0.6)
    spin_for(node, seconds=4.0)

    print('Releasing arms')
    node.release_joints()
    spin_for(node, seconds=2.5)

    print('Stop')
    node.stop()
    spin_for(node, seconds=1.0)

    print("Moving elbow to 1.0 rad")
    node.set_joint('right_elbow', 1.0)
    spin_for(node, seconds=3.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
