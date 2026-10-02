import time
import rclpy

from g1_interface.g1_sensors import G1Sensors
from g1_interface.g1_low_level import G1LowLevel


JOINT = "right_elbow"
DELTA = 0.15


def main():
    rclpy.init()

    sensors = G1Sensors()
    robot = G1LowLevel()

    print("Esperando g1_core...")

    if not robot.wait_for_core(timeout=5.0):
        print("ERROR: g1_core no encontrado")
        return

    print("Core conectado.")

    # Esperar hasta recibir realmente LowState
    print("Esperando sensores...")

    position = None

    while rclpy.ok() and position is None:
        rclpy.spin_once(sensors, timeout_sec=0.1)
        position = sensors.get_joint_position(JOINT)

    print(f"Posicion inicial: {position:.4f} rad")

    target = position + DELTA
    print(f"Objetivo:         {target:.4f} rad")

    print("\nMoviendo...")
    robot.set_joint(JOINT, target)

    start_time = time.time()

    while time.time() - start_time < 3.0:
        rclpy.spin_once(sensors, timeout_sec=0.05)

        q = sensors.get_joint_position(JOINT)

        if q is not None:
            print(f"{time.time() - start_time:.2f} s -> {q:.4f} rad")

        time.sleep(0.15)

    print("\nRegresando...")
    robot.set_joint(JOINT, position)

    start_time = time.time()

    while time.time() - start_time < 3.0:
        rclpy.spin_once(sensors, timeout_sec=0.05)

        q = sensors.get_joint_position(JOINT)

        if q is not None:
            print(f"{time.time() - start_time:.2f} s -> {q:.4f} rad")

        time.sleep(0.15)

    print("\nPrueba terminada.")

    sensors.destroy_node()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
