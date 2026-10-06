import time
import rclpy

from g1_interface.g1_sensors import G1Sensors
from g1_interface.g1_low_level import G1LowLevel


JOINT = "left_knee"

# Primera prueba:
DELTA = 0.60
DURATION = 6.0


def wait_for_position(sensors):
    q = None

    while rclpy.ok() and q is None:
        rclpy.spin_once(sensors, timeout_sec=0.1)
        q = sensors.get_joint_position(JOINT)

    return q


def observe(sensors, duration):
    start = time.time()
    last_print = 0.0

    while rclpy.ok() and (time.time() - start) < duration:
        rclpy.spin_once(sensors, timeout_sec=0.02)

        elapsed = time.time() - start

        if elapsed - last_print >= 0.2:
            q = sensors.get_joint_position(JOINT)

            if q is not None:
                print(
                    f"{elapsed:5.2f} s | "
                    f"q_real = {q:7.4f} rad"
                )

            last_print = elapsed

        time.sleep(0.01)


def main():
    rclpy.init()

    sensors = G1Sensors()
    robot = G1LowLevel()

    if not robot.wait_for_core(timeout=5.0):
        print("ERROR: g1_core no encontrado")
        return

    q_initial = wait_for_position(sensors)

    target = q_initial + DELTA

    print()
    print("==============================")
    print(" G1 REAL - MOVEMENT TEST")
    print("==============================")
    print(f"Joint     : {JOINT}")
    print(f"Inicial   : {q_initial:.4f} rad")
    print(f"Objetivo  : {target:.4f} rad")
    print(f"Delta     : {DELTA:.4f} rad")
    print(f"Duracion  : {DURATION:.1f} s")
    print("==============================")
    print()

    # ---------------------------------
    # Movimiento hacia adelante
    # ---------------------------------

    print("MOVIENDO HACIA OBJETIVO...")

    robot.move_joint(
        JOINT,
        position=target,
        duration=DURATION
    )

    observe(
        sensors,
        DURATION + 0.5
    )

    time.sleep(1.0)

    # ---------------------------------
    # Regreso
    # ---------------------------------

    print()
    print("REGRESANDO A POSICION INICIAL...")

    robot.move_joint(
        JOINT,
        position=q_initial,
        duration=DURATION
    )

    observe(
        sensors,
        DURATION + 0.5
    )

    print()
    print("Prueba terminada.")

    robot.destroy_node()
    sensors.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()