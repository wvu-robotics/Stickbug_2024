
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Float32MultiArray
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState
import copy


class servoIbvs:
    def __init__(self):
        rospy.init_node('servo_right_arm')
        

        
        #for transforming velocity to base frame
        self.listener = tf.TransformListener()

        # Wait for the transformation to become available
        self.listener.waitForTransform("stickbug/drivebase/r_lift", "stickbug/arm1/cam_link", rospy.Time(), rospy.Duration(4.0))
        
        self.no_marker = True

        #for jacobian
        self.move_group = MoveGroupCommander("left_arm_1")


        # set the desired points here
        self.d_pts = np.array([[200,290],[370,290],[370,440],[200,440]]) # this needs to be set here

        self.pts = np.zeros([4,3])  #points from aruco S

        self.err_pts = np.zeros([8,1])  #error 
        self.total_error_pts = np.zeros([8,1])

        self.Lx = np.zeros([8,6])  #interaction matrix 

        self.lam = -0.005/4  # proportional gain
        self.Ki = -7e-6 # integral gain

        self.u0 = 420; # x coordinate of principal point (image center)
        self.v0 = 241; # y coordinate of principal point (image center).
        self.px = 422; # Ratio between focal length and pixel size in x-direction.
        self.py = 422; # Ratio between focal length and pixel size in y-direction.
    
        self.v_c = np.array([6,1]) #velocity in camera frame 
        
        #self.joint_q = [1.60, -1.58, 1.0472, -2.6878, 0.1, 0.037]
        self.pub = rospy.Publisher('stickbug/arm1/manipulator_goal_joints', JointState, queue_size=10)
        #rospy.loginfo("all paramters initailized")
        #for getting points fom aruc0
        rospy.Subscriber('stickbug/arm1/aruco_corners',Float32MultiArray, self.aruco_corners_callback)
        #for joint values 
        rospy.Subscriber('stickbug/arm1/manipulator_actual_joints', JointState, self.joint_state_callback)
        
    def calc_err(self):
        #rospy.loginfo("i am inside cal error")
        self.err_pts[0,0] = self.pts[0,0] - self.d_pts[0,0]
        self.err_pts[1,0] = self.pts[0,1] - self.d_pts[0,1]
        self.err_pts[2,0] = self.pts[1,0] - self.d_pts[1,0]
        self.err_pts[3,0] = self.pts[1,1] - self.d_pts[1,1]
        self.err_pts[4,0] = self.pts[2,0] - self.d_pts[2,0]
        self.err_pts[5,0] = self.pts[2,1] - self.d_pts[2,1]
        self.err_pts[6,0] = self.pts[3,0] - self.d_pts[3,0]
        self.err_pts[7,0] = self.pts[3,1] - self.d_pts[3,1]

        self.total_error_pts += self.err_pts
        
        

    def calc_Lx(self):
        #rospy.loginfo("i am inside cal_lx")
        x1=(self.pts[0,0]-self.u0)/self.px
        y1=(self.pts[0,1]-self.v0)/self.py
        z1=self.pts[0,2]
        
        #2nd point
        x2=(self.pts[1,0]-self.u0)/self.px
        y2=(self.pts[1,1]-self.v0)/self.py
        z2=self.pts[1,2]
        
        #3rd point
        x3=(self.pts[2,0]-self.u0)/self.px
        y3=(self.pts[2,1]-self.v0)/self.py
        z3=self.pts[2,2]
        
        #4rd point
        x4=(self.pts[3,0]-self.u0)/self.px
        y4=(self.pts[3,1]-self.v0)/self.py
        z4=self.pts[3,2]
        
        
        #calculate lx

        lx1=[-1/z1, 0 ,x1/z1, x1*y1, -(1+x1*x1) ,y1]
        ly1= [0 , -1/z1,y1/z1, (1+y1*y1), -x1*y1, -x1]

        lx2=[-1/z2, 0 ,x2/z2, x2*y2, -(1+x2*x2) ,y2]
        ly2= [0 , -1/z2,y2/z2, (1+y2*y2), -x2*y2, -x2]

        lx3=[-1/z3, 0 ,x3/z3, x3*y3, -(1+x3*x3) ,y3]
        ly3= [0 , -1/z3,y3/z3, (1+y3*y3), -x3*y3, -x3]

        lx4=[-1/z4, 0 ,x4/z4, x4*y4, -(1+x4*x4) ,y4]

        ly4= [0 , -1/z4,y4/z4, (1+y4*y4), -x4*y4, -x4]

        Lx_total=np.array([lx1,ly1,lx2,ly2,lx3,ly3,lx4,ly4])
        self.Lx = copy.deepcopy(Lx_total)
        #rospy.loginfo("interaction matrix is : {}".format(self.Lx))
        ##print("the shape of lx is ",self.Lx.shape)
        
    def calc_control(self):
       #rospy.loginfo("i am inside cal_control")

       Lx_inv=np.linalg.pinv(self.Lx)
       #rospy.loginfo("invese of lx is : {}".format(Lx_inv))
       #rospy.loginfo("the errors inn points is : {}".format(self.err_pts))
       vi=np.dot(Lx_inv,self.err_pts)
       v_integral = np.dot(Lx_inv,self.total_error_pts)
       v_int_con = np.dot(self.Ki,v_integral)
       print("integral velocities are")
       print(v_int_con)
       v_c=np.dot(self.lam, vi)  + v_int_con
       self.v_c=np.array(v_c)
       #print("v_c value is ",self.v_c)
       #print("the shape of v_c is ",self.v_c.shape)

    
    def joint_state_callback(self,std_msgs):
        #rospy.loginfo("i am inside joint state call back")
        joint_names = std_msgs.name
        self.joint_values = std_msgs.position[:5]
        ##print("the current joint values are ", self.joint_values)
        ##rospy.loginfo("Received joint names: {}".format(joint_names))
        #rospy.loginfo("Received first five joint positions: {}".format(self.joint_values))
        
        
        self.servoFunc()
        
    def aruco_corners_callback(self, data):
        # Assuming the data is a 1D array representing the coordinates and depths of all corners
        corner_data = data.data

        if len(corner_data) == 0:
            self.no_marker = True
            return

        # Check if the data length is divisible by 3 (each corner has x, y coordinates, and depth)
        if len(corner_data) % 3 != 0:
            #rospy.logerr("Invalid corner data received")
            self.no_marker = True
            return

        self.no_marker = False
        # Update self.pts with the received corner coordinates and depths
        num_corners = len(corner_data) // 3
        for i in range(min(num_corners, 4)):  # Assuming only 4 corners per marker
            x = corner_data[i * 3]
            y = corner_data[i * 3 + 1]
            depth = corner_data[i * 3 + 2]
            self.pts[i] = [x, y, depth]

        # #print or use self.pts for further processing
        #rospy.loginfo("Received ArUco corners and depths: {}".format(self.pts))
 

    def calculate_jacobian(self):
        # Get current joint values
        #rospy.loginfo("i am inside jacobian")
        
        current_joint_values =self.joint_values
        joint_value_degree= np.asarray(current_joint_values).dot(57.2958)
        #print("current joint values in degree",joint_value_degree )
        # Calculate Jacobian matrix using MoveIt
        jacobian = self.move_group.get_jacobian_matrix(list(current_joint_values))
        #print("jacobian is",jacobian)
        #print("inside jacobian")
        return jacobian
    
    
    def convert_vel(self):
        #rospy.loginfo("i am inside convert_vel")
        (trans, rot_q) = self.listener.lookupTransform("stickbug/drivebase/r_lift", "stickbug/arm1/camera_frame", rospy.Time(0))

            # Create a homogeneous transformation matrix
        rot= tf.transformations.quaternion_matrix(rot_q)
        rotation=rot[:3, :3]
        

         
           # Extract camera linear and angular velocities from the message
        
        linear_velocity_tool0 = np.array([
            self.v_c[0,0],
            self.v_c[1,0],
            self.v_c[2,0]
            ])

        angular_velocity_tool0 = np.array([
            self.v_c[3,0],
            self.v_c[4,0],
            self.v_c[5,0]
           ])
       

            

            # Transform linear velocity
        linear_velocity_base = np.dot(rotation, linear_velocity_tool0)
           
            # Transform angular velocity
        #angular_velocity_base = np.dot(rotation, angular_velocity_tool0)
        angular_velocity_base = np.array([0,0,0])
       
            
        final_v=np.concatenate((linear_velocity_base, angular_velocity_base))
        ##print(final_v)
        return final_v
        


    




    def servoFunc(self):
       
        #rospy.loginfo("i am inside servo func")

        self.calc_Lx()
        self.calc_err()

        self.calc_control()
        #rospy.loginfo("control law calculated")

        
        
        vc_b=self.convert_vel()
        #rospy.loginfo("velocity wrt base done")

        #print("velocity wrt base is ",vc_b)

        #self.joint_state_callback()    
        jacobian = self.calculate_jacobian()
        #print("jacobian is ", jacobian)
        #rospy.loginfo("jacobian is calculated")
        j_inv=np.linalg.pinv(jacobian)
        
        joint_velocities = np.dot(j_inv,vc_b)
   
        gain=1
        #print("Chosen gains:", gain)
        joint_velocities=joint_velocities[:5]*gain
        joint_velocities=joint_velocities.tolist()

        # Extract the last four velocities and pad with zeros
        final_joint_velocities = [0.0, joint_velocities[-4], joint_velocities[-3], joint_velocities[-2], joint_velocities[-1], 0.0, 0.0 ,0.0]  #[0] + joint_velocities[-4:] + [0, 0, 0]
        print('the life velocities is',joint_velocities[0])
        ##print("joint velocities are",final_joint_velocities)
      
    
    
        
        #self.rtde_c.speedJ(joint_velocities,0.05, dt) 
        #print("Ready to send velocities to join congrats ")
        #need to publish velocity now

         # Create a publisher to publish to /arm0/manipulator_joint_goals
        
        joint_state_msg = JointState()

        # Fill in the velocities while leaving positions empty
        joint_state_msg.name = ["stickbug/arm2/slider", "stickbug/arm2/shoulder", "stickbug/arm2/elbow", "stickbug/arm2/w1", "stickbug/arm2/w2", "stickbug/arm2/e1", "stickbug/arm2/e2", "stickbug/arm2/e3"]
        joint_state_msg.velocity = final_joint_velocities  # Assuming joint_velocities is a numpy array
        
        if np.abs(self.err_pts).sum()<=100 or self.no_marker:
            joint_state_msg.velocity=[0,0,0,0,0,0,0,0]
        joint_state_msg.position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]  # Leave positions empty
        print("the errors are ",self.err_pts)
        print("joint state velocity are ",joint_state_msg.velocity)
        
        # Publish the joint state message
        self.pub.publish(joint_state_msg)
        #servo_node.draw_aruco(color_image)
       



def main():
    servo_node = servoIbvs()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #servo_node.servoFunc()
        rate.sleep()

if __name__ == '__main__':
    main()

