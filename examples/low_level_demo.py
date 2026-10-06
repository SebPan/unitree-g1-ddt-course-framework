import time
import rclpy

from g1_interface.g1_low_level import G1LowLevel
from g1_interface.g1_sensors import G1Sensors


JOINT = "right_elbow"
DELTA = 0.15


def main():

    rclpy.init()

    robot = G1LowLevel()
    sensors = G1Sensors()

    print("Esperando g1_core...")

    if not robot.wait_for_core():
        print("ERROR: g1_core no disponible")
        return

    print("Core conectado.")

    print("Esperando sensores...")

    q0 = None

    for _ in range(50):

        q0 = sensors.get_joint_position(JOINT)

        if q0 is not None:
            break

        time.sleep(0.1)

    if q0 is None:
        print("ERROR: no llegan sensores")
        return

    target = q0 + DELTA

    print(f"Posicion inicial: {q0:.4f} rad")
    print(f"Objetivo:         {target:.4f} rad")

    # ========================================================
    # MOVER
    # ========================================================

    print("\nMoviendo...")

    ok = robot.move_joint(
        JOINT,
        target,
        duration=2.0
    )

    print("move_joint =", ok)

    time.sleep(0.5)

    q1 = sensors.get_joint_position(JOINT)

    print(
        f"Posicion alcanzada: "
        f"{q1:.4f} rad"
    )

    # ========================================================
    # REGRESAR
    # ========================================================

    print("\nRegresando...")

    ok = robot.move_joint(
        JOINT,
        q0,
        duration=2.0
    )

    print("move_joint =", ok)

    time.sleep(0.5)

    q2 = sensors.get_joint_position(JOINT)

    print(
        f"Posicion final: "
        f"{q2:.4f} rad"
    )

    # Mantener posicion final
    robot.hold()

    print("\nPrueba terminada.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
