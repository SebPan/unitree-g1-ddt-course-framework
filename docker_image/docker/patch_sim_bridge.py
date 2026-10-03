from pathlib import Path

p = Path(
    "/opt/unitree_rl_mjlab/"
    "simulate/src/unitree_sdk2_bridge.h"
)

s = p.read_text()

# ============================================================
# Includes
# ============================================================

anchor = '#include "physics_joystick.h"'

if '#include "sim_udp_state.h"' not in s:
    s = s.replace(
        anchor,
        anchor + '\n#include "sim_udp_state.h"',
        1
    )

if '#include "sim_udp_command.h"' not in s:
    s = s.replace(
        '#include "sim_udp_state.h"',
        '#include "sim_udp_state.h"\n'
        '#include "sim_udp_command.h"',
        1
    )


# ============================================================
# Miembros
# ============================================================

anchor = (
    'std::shared_ptr<unitree::common::UnitreeJoystick> '
    'joystick = nullptr;'
)

if 'SimUdpStatePublisher sim_udp_state_;' not in s:
    s = s.replace(
        anchor,
        anchor +
        '\n\n    SimUdpStatePublisher sim_udp_state_;',
        1
    )

if 'SimUdpCommandReceiver sim_udp_command_;' not in s:
    s = s.replace(
        '    SimUdpStatePublisher sim_udp_state_;',
        '    SimUdpStatePublisher sim_udp_state_;'
        '\n    SimUdpCommandReceiver sim_udp_command_;',
        1
    )


# ============================================================
# Estado SIM -> UDP 15010
# ============================================================

anchor = '        if(!mj_data_) return;'

if 'sim_udp_state_.publish(' not in s:
    replacement = '''        if(!mj_data_) return;

        // Backend SIM educativo:
        // MuJoCo -> UDP 15010 -> g1_core
        sim_udp_state_.publish(
            mj_data_,
            num_motor_,
            imu_quat_adr_,
            imu_gyro_adr_,
            imu_acc_adr_
        );'''

    if s.count(anchor) != 1:
        raise RuntimeError(
            "No se encontro run() de RobotBridge"
        )

    s = s.replace(
        anchor,
        replacement,
        1
    )


# ============================================================
# Comando UDP 15020 -> MuJoCo
#
# Se aplica DESPUES de LowCmd Unitree, por lo que mientras
# LowLevel SIM este activo nuestro backend tiene prioridad.
# ============================================================

if 'sim_udp_command_.apply(' not in s:
    anchor = '        // lowstate'

    if anchor not in s:
        raise RuntimeError(
            "No se encontro bloque lowstate"
        )

    replacement = '''        // Backend LowLevel SIM:
        // g1_core -> UDP 15020 -> MuJoCo
        sim_udp_command_.apply(
            mj_data_,
            num_motor_
        );

        // lowstate'''

    s = s.replace(
        anchor,
        replacement,
        1
    )


p.write_text(s)

print("SIM UDP state + command bridge aplicado.")
