# Unitree G1 EDU 23DOF Course Framework

Framework educativo para Unitree G1 EDU 23DOF.

## Características

- MuJoCo
- Robot real
- LowLevel
- HighLevel
- Locomoción
- Control de brazos
- Misma API Python para SIM y REAL

## Requisitos

- Windows 11
- WSL2 con Ubuntu
- Docker Desktop
- Integración Docker Desktop + WSL
- Arquitectura x86_64

Para el robot real, la PC debe tener una interfaz de red con una IP:

    192.168.123.x/24

El launcher intenta detectar automáticamente esa interfaz.

## Instalación desde cero

Clonar el repositorio:

    git clone <URL_DEL_REPOSITORIO>
    cd <NOMBRE_DEL_REPOSITORIO>

Instalar:

    chmod +x install_g1.sh
    ./install_g1.sh

## Verificar instalación

    ./verify_install.sh

Debe finalizar con:

    G1 INSTALLATION OK

## Simulación

LowLevel:

    ./g1 sim --mode low

HighLevel:

    ./g1 sim --mode high

## Robot real

LowLevel:

    ./g1 real --mode low

HighLevel:

    ./g1 real --mode high

La interfaz puede especificarse manualmente si es necesario:

    ./g1 real --mode high --interface ethX

## API LowLevel

    from g1_interface.g1_low_level import G1LowLevel

Métodos principales:

- set_joint()
- move_joint()
- set_joints()
- move_joints()

## API HighLevel

    from g1_interface.g1_high_level import G1HighLevel

Métodos principales:

- damp()
- stand()
- prepare()
- velocity()
- walk()
- stop()
- arm_joint()
- move_arms()
- arm_enable()
- arm_release()

## Reglas de brazos

Los brazos solamente pueden moverse con el robot detenido o en equilibrio.

Durante el control de brazos:

- arm_sdk toma autoridad sobre los brazos.
- WaistYaw permanece bloqueado en su posición actual.
- No se permite iniciar locomoción mientras una trayectoria de brazos está ejecutándose.
- No se permiten movimientos de brazos durante la marcha.

Cuando se solicita locomoción después de usar los brazos:

- arm_sdk se libera automáticamente.
- WaistYaw vuelve al controlador normal.
- Se inicia locomoción.

## Controlador de locomoción

Basado en:

    https://github.com/PIControlLab/unitree_rl_lab.git

Commit utilizado:

    8b14a640d85a2629633ae4e6358d9765d9d73bb5

Ver también:

    docs/THIRD_PARTY_VERSIONS.md
