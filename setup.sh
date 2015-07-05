chmod 777 src/PFG/atp_utils/nodes/*.py
chmod 777 src/PFG/atp_tasks/nodes/*.py
chmod 777 src/PFG/atp_utils/cfg/BatterySimulator.cfg
cd src/
git clone https://github.com/vanadiumlabs/arbotix_ros.git
cd ..
catkin_make
source devel/setup.bash
