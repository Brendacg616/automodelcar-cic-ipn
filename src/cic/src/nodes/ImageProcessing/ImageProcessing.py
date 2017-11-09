#! /usr/bin/python
# it requires "chmod +x mypythonscript.py" to be called by ROS
##############################################################
# Image preprocesor v1
# Autor: Cesar Bravo
# 29/03/2017
##############################################################

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import math
from numpy import *

#Constants
bridge = CvBridge()

def image_callback(ros_data):
	
    try:
        # Convert your ROS Image message to OpenCV2

	# Uncomment the following line to work with raw images
        #cv2_img = bridge.imgmsg_to_cv2(ros_data, "bgr8")

	# Uncoment the following two lines to work with compressed images
	np_arr = np.fromstring(ros_data.data, np.uint8)
	cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    except CvBridgeError, e:
        print(e)
    else:

	# Image pre-processing function begins
	e1 = cv2.getTickCount()

        # 1. Convert data received to numpy array
        image = np.asarray(cv2_img) # 480x640x3	
	
	
	# 2. Get Region of interest and scalation ------------------------------
        crop = image[90:450,:] 
	scld = cv2.resize(crop,None,fx=0.4,fy=1.2,interpolation=cv2.INTER_AREA)

	# 3. Color transform ---------------------------------------------------
	color = cv2.cvtColor(scld, cv2.COLOR_BGR2GRAY)
	hght, wdth = color.shape[:2]
	print(hght, wdth)
	
	# 4. Inverse Prespective Mapping ---------------------------------------
	pts1 = np.float32([[wdth/2-55,0],[wdth/2+55,0],[0,hght],[wdth,hght]])
	pts2 = np.float32([[0,0],[wdth,0],[wdth/2-40,hght],[wdth/2+40,hght]])
	
	M = cv2.getPerspectiveTransform(pts1,pts2)
	dst = cv2.warpPerspective(color,M,(wdth,hght))
	
	# Compress image to pub ------------------------------------------------
        cropImage = CompressedImage()
        cropImage.header.stamp = rospy.Time.now()
        cropImage.format = "jpeg"
        cropImage.data = np.array(cv2.imencode('.jpg',dst)[1]).tostring()
        pub.publish(cropImage)

	# Print stats ----------------------------------------------------------
	e2 = cv2.getTickCount()	
    	t = (e2 - e1)/cv2.getTickFrequency()	
	
	print('frame time:'+str(t)+'-------------------------------block end')
        

def main():

    global pub

    rospy.init_node('im_prepros')
    image_topic = "/app/camera/rgb/image_raw/compressed"
    rospy.Subscriber(image_topic, CompressedImage, image_callback,queue_size=1)
    pub = rospy.Publisher('/img_prepros/compressed', CompressedImage, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
	main()
	
