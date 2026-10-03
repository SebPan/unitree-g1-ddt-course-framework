ld
id
echo $HOME
touch /workspace/prueba_docker.txt
ls -l /workspace/prueba_docker.txt
ls
rm prueba_docker.txt 
exit
id
rm -rf build install log
colcon build --packages-select g1_core --symlink-install
ls -l /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
source /opt/ros/humble/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
ros2 pkg prefix unitree_hg
ros2 pkg prefix unitree_api
/opt/unitree_ros2/cyclonedds_ws/install/unitree_hg
/opt/unitree_ros2/cyclonedds_ws/install/unitree_api
cd /workspace
colcon build     --packages-select g1_core     --symlink-install
exit
source /opt/ros/humble/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
cd /workspace
rm -rf build install log
colcon build     --packages-select g1_core     --symlink-install
source /opt/ros/humble/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
if [ -f /workspace/install/setup.bash ]; then     source /workspace/install/setup.bash; fi
exit
cd /workspace
source /opt/ros/humble/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build   --packages-select g1_core   --symlink-install
cd ~/Unitree_G1/g1_sim/workspace
git add -A
git commit -m "Remove control mode from g1_core"
exit
cd /workspace
colcon build   --packages-select g1_core   --symlink-install
exit
cd /workspace
colcon build   --packages-select g1_core   --symlink-install
exit
colcon build   --packages-select g1_core   --symlink-install
exit
colcon build   --packages-select g1_core   --symlink-install
exit
colcon build   --packages-select g1_core   --symlink-install
exit
