#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
from ultralytics.utils.plotting import Annotator

# Initialize the ROS node
rospy.init_node('yolov8_realSense_tracker')

# Load the YOLOv8 model (Make sure 'yolov8n.pt' is in the correct path)
model = YOLO('best_bramble.pt')


# Initialize the RealSense camera pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the RealSense camera
pipe.start(cfg)

# Create a named window for displaying the camera feed
cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

try:
    while not rospy.is_shutdown():
        # Wait for frames from the RealSense camera
        frames = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if color_frame:
            # Convert the RealSense color frame to a NumPy array
            color_image = np.asanyarray(color_frame.get_data())
            
            # Perform YOLOv8 object tracking
            results = model.track(color_image,persist=True,show=True)
            annotated_frame = results[0].plot()
            
            for r in results:
                annotator = Annotator(color_image)
                boxes = r.boxes
                ids = r.boxes.id
                #print ("the id value is ",ids)
                #print("the box values are ",boxes)
                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format              
                    id=box[0].id
                    print("the id is ",id)
                    print("the box value is ", b)
                    #print ("flower id %d has box %d"%(id,b))
                    #c = box.cls
                    #print("class is ",c)
                    #id=
                    #annotator.box_label(b, model.names[int(c)])
		  
        
            
            # Display the camera feed
            #cv2.imshow("Camera Feed", color_image)
            
            # Display the annotated frame
            cv2.imshow("Tracking", annotated_frame)
            
            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
               break

finally:
    pipe.stop()
    cv2.destroyAllWindows()

