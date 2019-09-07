#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def publish_video_file(path):
    # Get reference to the lanes_video topic
    publisher = rospy.Publisher('lanes_video', Image, queue_size=100)

    # Setup ROS
    rospy.init_node('lanes_publisher', anonymous=True)
    rospy.Rate(10)
    bridge = CvBridge()

    # Load the video from path
    capture = cv2.VideoCapture(path)

    if not capture.isOpened():
        print "Error opening resource: " + str(path)
        print "Maybe opencv VideoCapture can't open it"
        exit(1)

    print "Correctly opened resource, starting to show feed."

    # Read the first frame
    ret, frame = capture.read()

    # Read the video frame by frame:
    while ret:
        ret, frame = capture.read()

        if frame is None:
            break

        # Convert the cv2 image to cv-bridge:
        frame = np.uint8(frame)
        image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")

        # Publish the image to the topic
        publisher.publish(image_message)

        key = cv2.waitKey(1000)
        if key == 27 or key == 1048603:
            break


if __name__ == '__main__':
    try:
        # Or change it to your absolute path:
        path = "/home/hesham/my_ws2/src/lane_detection/src/lanes.mp4"
        #path = "/dev/video0"
        publish_video_file(path)
        exit(0)
    except rospy.ROSInterruptException as ex:
        print (ex)
        exit(1)
