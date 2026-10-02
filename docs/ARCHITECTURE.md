# G1 Course Architecture

## Main goal

Student Python programs must run unchanged in simulation and on the real Unitree G1.

## Student API

The public educational API consists of:

- G1Sensors
- G1LowLevel
- G1HighLevel

Students should not need to configure or understand:

- CycloneDDS networking
- MotionSwitcher
- CRC
- Unitree motor IDs
- C++ infrastructure
- simulator UDP internals

## Runtime platform

Windows 11
└── WSL2 Ubuntu 22.04 in mirrored networking mode
    └── Native Docker Engine
        └── G1 course container using host networking

## Simulation

Container
└── MuJoCo
    └── simulated G1

## Real robot

Container
└── CycloneDDS
    └── Ethernet interface 192.168.123.99
        └── Unitree G1

Real DDS communication has been validated from inside the container.

Observed /lowstate rate:
approximately 1100 Hz.

## Core principle

SIM and REAL must expose equivalent behavior and, where practical,
the same ROS 2 topics and message types.

A feature is not considered complete until the same student Python
program works in both SIM and REAL.
