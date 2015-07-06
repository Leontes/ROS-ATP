chmod 777 src/ROS-ATP/atp_utils/nodes/*.py
chmod 777 src/ROS-ATP/tasks_exec/nodes/*.py
chmod 777 src/ROS-ATP/atp_utils/cfg/BatterySimulator.cfg
cd src/
git clone https://github.com/vanadiumlabs/arbotix_ros.git
cd ..
apt-get install ros-indigo-rviz
catkin_make
source devel/setup.bash
