
#!/usr/bin/env python

import rospy
import tf
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
#from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Float32MultiArray
from moveit_commander import MoveGroupCommander
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive


class servoIbvs:
    def __init__(self):
        rospy.init_node('aruco_detection_node')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        # self.aruco_pub = rospy.Publisher('/aruco_markers', Float32MultiArray, queue_size=10)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        self.window_name = 'Aruco Detection'
        cv2.namedWindow(self.window_name)

        self.depth_image = None

        self.listener = tf.TransformListener()

        # Wait for the transformation to become available
        self.listener.waitForTransform("base", "cam_link", rospy.Time(), rospy.Duration(4.0))

        self.move_group = MoveGroupCommander("right_arm")

        # Initialize RTDE control interface
        #self.rtde_c = RTDEControl("192.168.10.111")
        #500, RTDEControl.FLAG_USE_EXT_UR_CAP,5001,RTDEControl.FLAG_NO_WAIT)
        #self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.10.111")
        #print("moving to work position")
       # self.joint_q = [1.60, -1.58, 1.0472, -2.6878, 0.1, 0.037]
        #self.rtde_c.moveJ(self.joint_q)

        # set the desired points here
        self.d_pts = np.array([[350,150],[500,150],[500,250],[350,250]]) # this needs to be set here

        self.pts = np.empty([4,3])

        self.err_pts = np.empty([8,1])

        self.Lx = np.empty([8,6])

        self.lam = -1

        self.u0 = 325.24761962890625; # x coordinate of principal point (image center)
        self.v0 = 233.45726013183594; # y coordinate of principal point (image center).
        self.px = 617.6026000976562; # Ratio between focal length and pixel size in x-direction.
        self.py = 617.7794189453125; # Ratio between focal length and pixel size in y-direction.
    
        self.v_c = np.array([6,1]) #velocity in camera frame 

        self.joint_q = [1.60, -1.58, 1.0472, -2.6878, 0.1, 0.037]
    
    def calc_err(self):
        self.err_pts[0,0] = self.pts[0,0] - self.d_pts[0,0]
        self.err_pts[1,0] = self.pts[0,1] - self.d_pts[0,1]
        self.err_pts[2,0] = self.pts[1,0] - self.d_pts[1,0]
        self.err_pts[3,0] = self.pts[1,1] - self.d_pts[1,1]
        self.err_pts[4,0] = self.pts[2,0] - self.d_pts[2,0]
        self.err_pts[5,0] = self.pts[2,1] - self.d_pts[2,1]
        self.err_pts[6,0] = self.pts[3,0] - self.d_pts[3,0]
        self.err_pts[7,0] = self.pts[3,1] - self.d_pts[3,1]


    def calc_Lx(self):
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
        self.Lx=Lx_total
        print("interaction matrix is ",self.Lx)
        print("the shape of lx is ",self.Lx.shape)
        
    def calc_control(self):

       Lx_inv=np.linalg.pinv(self.Lx)
       vi=np.dot(Lx_inv,self.err_pts)
       v_c=np.dot(self.lam, vi)
       self.v_c=np.array(v_c)
       print("v_c value is ",self.v_c)
       print("the shape of v_c is ",self.v_c.shape)
    def calculate_jacobian(self):
        # Get current joint values
        current_joint_values =  self.rtde_r.getActualQ()
        joint_value_degree= np.asarray(current_joint_values).dot(57.2958)
        #print("current joint values",joint_value_degree )
        # Calculate Jacobian matrix using MoveIt
        jacobian = self.move_group.get_jacobian_matrix(current_joint_values)
        #print("jacobian is",jacobian)
        return jacobian


    def depth_callback(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_image = np.array(cv_depth, dtype=np.float32) 
        except CvBridgeError as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # Draw ArUco marker on the image
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        if ids is not None and self.depth_image is not None:
            # Publish marker depth information
            aruco_info = Float32MultiArray()

            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]

                # Calculate the average depth value for the marker corners
                corner_depths = []
                for corner in marker_corners:
                    x, y = int(corner[0]), int(corner[1])
                    if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                        depth = self.depth_image[y, x]/1000
                        corner_depths.append(depth)

                if len(corner_depths) > 0:
                    # Calculate the average depth
                    average_depth = np.mean(corner_depths)

                    self.pts[0,0] = marker_corners[0][0] # X-coordinate of top-left corner
                    self.pts[0,1] = marker_corners[0][1]  # Y-coordinate of top-left corner
                    self.pts[0,2] = average_depth

                    self.pts[1,0] = marker_corners[1][0]   # X-coordinate of top-right corner
                    self.pts[1,1] = marker_corners[1][1]  # Y-coordinate of top-right corner
                    self.pts[1,2] = average_depth
                    
                    self.pts[2,0] = marker_corners[2][0] # X-coordinate of bottom-right corner
                    self.pts[2,1] = marker_corners[2][1]   # Y-coordinate of bottom-right corner
                    self.pts[2,2] = average_depth
                    
                    self.pts[3,0] = marker_corners[3][0]  # X-coordinate of bottom-left corner
                    self.pts[3,1] = marker_corners[3][1]  # Y-coordinate of bottom-left corner
                    self.pts[3,2] = average_depth

                    print( ' first corner x is',self.pts[0,0])
                    print( ' first corner y is',self.pts[0,1])
                    print( ' third  corner x is',self.pts[2,0])
                    print( ' third corne y  is',self.pts[2,1])

            self.servoFunc()
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)
        
    def convert_vel(self):
        (trans, rot_q) = self.listener.lookupTransform("base_link", "tool0", rospy.Time(0))

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
        #print(final_v)
        return final_v
        



    def servoFunc(self):
       
        

        self.calc_err()

        self.calc_Lx()

        self.calc_control()

        vc_b=self.convert_vel()
        print("velocity wrt base is ",vc_b)
            
        jacobian = self.calculate_jacobian()
        j_inv=np.linalg.pinv(jacobian)
        
        joint_velocities = np.dot(j_inv,vc_b)

        gain=1
        #print("Chosen gains:", gain)
        joint_velocities=joint_velocities*gain
        
    
     
        
    
        dt=0.01
        print("depth is ",self.pts[0,2])
        #print("camera velocity",self.v_c)
        self.rtde_c.speedJ(joint_velocities,0.05, dt) 
        #print("robot moving with join vel",joint_velocities)




def main():
    servo_node = servoIbvs()
    rospy.spin()

if __name__ == '__main__':
    main()

