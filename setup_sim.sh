#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'

echo "================================"
echo " G1 NETWORK MODE: SIMULATION"
echo " DDS interface: lo"
echo "================================"
