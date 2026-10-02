import time
import rclpy

from g1_interface.g1_sensors import G1Sensors
from g1_interface.g1_low_level import G1LowLevel

JOINT = 'right_elbow'
DELTA = 0.05
DURATION = 3.0

def observe(sensors, seconds):
    start = time.time()
    last_print = 0.0

    while time.time() - start < seconds:
        rclpy.spin_once(sensors, timeout_sec=0.02)
        elapsed = time.time() - start
        if elapsed -last_print >= 0.25:
            q = sensors.get_joint_position(JOINT)

            if q is not None:
                print(f'{elapsed} s -> {q} rad')
            last_print = elapsed

def main():
    rclpy.init()
    sensors = G1Sensors()
    robot = G1LowLevel()

    if not robot.wait_for_core(timeout=5.0):
        print('Error: g1_core no encontrado')
        return

    position = None
    print('Esperando sensors')

    while rclpy.ok() and position is None:
        rclpy.spin_once(sensors, timeout_sec=0.1)
        position = sensors.get_joint_position(JOINT)

    target = position + DELTA

    print()
    print(f"Inicial : {position:.4f} rad")
    print(f"Objetivo: {target:.4f} rad")
    print(f"Duracion: {DURATION:.1f} s")
    print()

    print('Moviendo...')
    robot.move_joint(JOINT, position=target, duration=DURATION)
    observe(sensors, 2.3)

    print()
    print('Regresando...')
    robot.move_joint(JOINT, position=position, duration=DURATION)
    observe(sensors, 2.3)

    print()
    print('Prueba terminada.')

    sensors.destroy_node()
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
