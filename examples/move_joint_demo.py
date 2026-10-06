import time
import rclpy

from g1_interface.g1_sensors import G1Sensors
from g1_interface.g1_low_level import G1LowLevel


JOINT = "right_elbow"

TARGET_A = 0.60
TARGET_B = 1.80

DURATION = 9.5


def wait_position(sensors):
    q = None

    while rclpy.ok() and q is None:
        rclpy.spin_once(sensors, timeout_sec=0.1)
        q = sensors.get_joint_position(JOINT)

    return q


def observe(sensors, seconds):
    start = time.time()
    last_print = 0.0

    while rclpy.ok() and time.time() - start < seconds:
        rclpy.spin_once(sensors, timeout_sec=0.02)

        elapsed = time.time() - start

        if elapsed - last_print >= 0.5:
            q = sensors.get_joint_position(JOINT)

            if q is not None:
                print(
                    f"{elapsed:5.2f} s -> "
                    f"{q:7.4f} rad"
                )

            last_print = elapsed


def move(robot, sensors, target):
    current = wait_position(sensors)

    print()
    print(f"Actual   : {current:.4f} rad")
    print(f"Objetivo : {target:.4f} rad")
    print(f"Delta    : {target-current:+.4f} rad")
    print(f"Duracion : {DURATION:.1f} s")
    print()

    robot.move_joint(
        JOINT,
        position=target,
        duration=DURATION
    )

    observe(sensors, DURATION + 0.5)


def main():
    rclpy.init()

    sensors = G1Sensors()
    robot = G1LowLevel()

    if not robot.wait_for_core(timeout=5.0):
        print("ERROR: g1_core no encontrado")
        return

    initial = wait_position(sensors)

    print()
    print("================================")
    print("   G1 LARGE MOTION TEST")
    print("================================")
    print(f"Inicial : {initial:.4f} rad")
    print(f"Rango   : {TARGET_A:.2f} -> {TARGET_B:.2f} rad")
    print("================================")

    print("\n1) Moviendo a TARGET_A")
    move(robot, sensors, TARGET_A)

    time.sleep(1.0)

    print("\n2) Moviendo a TARGET_B")
    move(robot, sensors, TARGET_B)

    time.sleep(1.0)

    print("\n3) Regresando a posicion inicial")
    move(robot, sensors, initial)

    print("\nPrueba terminada.")

    sensors.destroy_node()
    robot.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()