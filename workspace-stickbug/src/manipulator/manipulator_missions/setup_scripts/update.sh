echo "removing old files"
sudo rm -r /home/ubuntu/manipulation_ws/src/manipulator
sudo rm -r /home/ubuntu/manipulation_ws/src/CMakeLists.txt
sudo rm -r /home/ubuntu/manipulation_ws/devel
sudo rm -r /home/ubuntu/manipulation_ws/build

echo "trying to connect to base computer at stickbug@192.168.47.10 and copy workspace"
until sshpass -p 'stickbug' scp -r stickbug@192.168.47.10:~/workspace-stickbug/src/manipulator ~/manipulation_ws/src; do
         sleep 1
done

echo "creating arm_main.launch"
python3 /home/ubuntu/manipulation_ws/src/manipulator/manipulator_missions/setup_scripts/create_launch_file.py

echo "building workspace"
cd ~/manipulation_ws
catkin_make -k
catkin_make -k
source devel/setup.bash

echo "configuration is complete rebooting"
