import socket
ip = socket.gethostbyname(socket.gethostname())
id = ip[-1]
print(ip)

id_num = int(id)
is_right_arm = str((id_num % 2) == 0).lower()

lines = ['<launch>\n',\
         '\t<arg name="robot_namespace" default="arm'+ id + '"/>\n'\
         '\t<arg name="is_right_arm" default="'+ is_right_arm + '"/>\n'\
         '\t<include file="$(find manipulator_missions)/launch/simple_manip_goal.launch">\n',\
         '\t\t<arg name="robot_namespace" value="$(arg robot_namespace)"/>\n',\
         '\t\t<arg name="is_right_arm" value="$(arg is_right_arm)"/>\n',\
         '\t</include>\n',\
         '</launch>']
         
with open('/home/ubuntu/manipulation_ws/src/manipulator/manipulator_missions/launch/arm_main.launch', 'w') as f:
    f.writelines(lines)