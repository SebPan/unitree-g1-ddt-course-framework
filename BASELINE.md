# G1 Course - Functional Baseline

This snapshot preserves the first validated SIM/REAL architecture before cleanup and refactoring.

## External controller

Repository:
https://github.com/PIControlLab/unitree_rl_lab.git

Commit:
324c4eac09a3af635066b89e950d776f3da9e87e

Path:
controller/unitree_rl_lab_23

The controller repository is intentionally not tracked inside this repository.

## Validated functionality

### Simulation
- Unitree G1 EDU 23DOF in MuJoCo
- /lowstate
- Python sensor API
- Low-level joint commands
- High-level locomotion through g1_ctrl
- Head camera bridge

### Real G1
- Ethernet / DDS communication
- /lowstate at approximately 1 kHz
- Python G1Sensors API
- MotionSwitcher ReleaseMode
- g1_core low-level command path
- Real right elbow movement
- Joint limit / max-step protection

## Important

The current directory structure is a development baseline.
It is intentionally not yet clean and will be refactored later.
