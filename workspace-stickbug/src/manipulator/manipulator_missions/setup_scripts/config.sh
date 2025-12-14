echo "creating arm_main.launch"
python3 /home/ubuntu/manipulation_ws/src/manipulator/manipulator_missions/setup_scripts/create_launch_file.py

echo "building workspace"
cd ~/manipulation_ws
catkin_make -k
catkin_make -k
source devel/setup.bash

echo "removing setup script and adding ros_master, sourcing, and launch file to .bashrc"
sed -i '$d' ~/.bashrc

echo "export ROS_MASTER_URI=http://192.168.47.10:11311/" >> ~/.bashrc
echo "cd ~/manipulation_ws" >> ~/.bashrc
echo "source devel/setup.bash" >> ~/.bashrc
echo "roslaunch manipulator_missions arm_main.launch" >> ~/.bashrc

echo "configuration is complete rebooting"
sudo reboot now
