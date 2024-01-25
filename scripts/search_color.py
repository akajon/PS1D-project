#!/usr/bin/python3

from os import stat_result
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main_state_callback(data):
    global state
    state = data.data

def image_sub_callback(data):
    global state
    if state == 'search':
        print("state=search")
        try:
            cv_image = bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            
        lower = np.array([0, 0, 100], dtype = "uint8")
        upper = np.array([0, 0, 255], dtype = "uint8")

        mask = cv2.inRange(cv_image, lower, upper)
        output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        cv2.imshow("Image window", output)
        if np.any((output != 0)):
            rospy.loginfo("found red")
            pub.publish("found")

        
        cv2.waitKey(3)

def main(args):
    global bridge 
    global state
    global pub

    bridge = CvBridge()
    state = "search"

    pub = rospy.Publisher('main_state', String, queue_size=10)
    rospy.init_node('search_color', anonymous=True)
    rospy.Subscriber('main_state', String, main_state_callback)

    image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,image_sub_callback, queue_size=1)
    # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
