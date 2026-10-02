import rclpy
from g1_interface.g1_sensors import G1Sensors


def main():
    rclpy.init()
    robot = G1Sensors()
    print('Esperando datos...')
    try:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            position = robot.get_joint_position('right_elbow')

            if position is not None:
                print(f'Right Elbow Position: {position} rad')
    except KeyboardInterrupt:
        pass

    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
