chmod 777 src/ROS-ATP/atp_utils/nodes/*.py
chmod 777 src/ROS-ATP/tasks_exec/nodes/*.py
chmod 777 src/ROS-ATP/atp_utils/cfg/BatterySimulator.cfg

apt-get install -y ros-indigo-arbotix ros-indigo-rviz

catkin_make
source devel/setup.bash