#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'

echo "================================"
echo " G1 NETWORK MODE: REAL"
echo " DDS interface: eth0"
echo "================================"
