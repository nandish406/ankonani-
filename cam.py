#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

import numpy as np

class Camera1:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback,queue_size=1)
        self.latest_image = np.zeros((360,640,3), np.uint8)
    # Ref: GeekforGeeks.com, https://www.geeksforgeeks.org/detect-the-rgb-color-from-a-webcam-using-python-opencv/
    def get_dominant_colour(self, arg_img):
        # setting values for base colors 
        b = arg_img[:, :, :1] 
        g = arg_img[:, :, 1:2] 
        r = arg_img[:, :, 2:] 
    
        # computing the mean 
        b_mean = np.mean(b) 
        g_mean = np.mean(g) 
        r_mean = np.mean(r) 

        # displaying the most prominent color 
        if (g_mean > r_mean and g_mean > b_mean): 
            return 'green' 
        else: 
            return 'something else'

    def get_qr_data(self, arg_image):
        qr_result = decode(arg_image)

        if ( len( qr_result ) > 0):
            return (qr_result)
        else :
            return ('NA')

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        (rows,cols,channels) = cv_image.shape
        image = cv_image
        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(image, (720//2, 1280//2)) 
        self.latest_image=resized_image
        cv2.imshow("/eyrc/vb/camera_1/image_raw", image)
        rospy.loginfo(self.get_dominant_colour(image))
        cv2.waitKey(3)



def main(args):
    rospy.init_node('node_eg2_colour_detection', anonymous=True)
    ic = Camera1()
    r=rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            print("loop")
            print(ic.get_qr_data(ic.latest_image))
            cv2.imshow("/eyrc/vb/camera_1/image_raw", ic.latest_image)
            cv2.waitKey(3)
        except:
            print("exception")

if __name__ == '__main__':
    main(sys.argv)
