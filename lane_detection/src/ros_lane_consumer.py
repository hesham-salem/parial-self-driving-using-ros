#!/usr/bin/env python

import cv2
import numpy as np
from detector import LaneDepartureDetector
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
global x ,y
x = 0
y=1
"""
Usi
ng this script is pretty simple, just set the input topic to be your camera input topic
and the lane departure from center will be published to `image_lane_departure`
and the processed image (which is the camera input + detected lanes) to `image_lane_detector`

So any other ROS node can subscribe to changes of the departure, for example:

def callback(data):
    
    # Get the departure direction:
    direction = ""
    if (data > 0):  # Right
        direction = "right"
    else:           # Left
        direction = "left"
    
    magnitude = abs(data)
    
    # Play a sound that notifies the driver 
    # if the departure is greater than 50 cm
    if (magnitude > 0.5)
        text_to_speech("Move to the " + direction)

rospy.Subscriber("image_lane_departure", Image, callback)
rospy.spin()
"""


def _create_config(height, width):
    """
    Create the initial config needed for the detector
    :param height: the height of the image
    :param width: the width of the image
    :return: src, dst transforms and roi vertices array
    """
    center_x = 680
    center_y = height / 2
    x_top_factor = 0.04
    x_lower_factor = 0.5
    lower_left = [center_x - x_lower_factor * width, height]
    lower_right = [center_x + x_lower_factor * width, height]
    top_left = [center_x - x_top_factor * width, center_y + height / 10]
    top_right = [center_x + x_top_factor * width, center_y + height / 10]

    roi_matrix = np.int32([
        lower_left, top_left, top_right, lower_right
    ])
    src = np.float32([(515, 415),
                      (705, 415),
                      (330, 715),
                      (1010, 715)])

    dst = np.float32([(500, 0),
                      (width - 500, 0),
                      (500, height),
                      (width - 500, height)])

    return src, dst, roi_matrix


class LaneDepartureConsumer:
    """
    A class to encapsulate the lane departure consumer, this class will subscribe
    to the `lanes_video` topic to get frames from the lanes camera input
    than it will apply the LaneDepartureDetector on that input to get the
    processed frame and the actual departure in meters
    """
    def __init__(self):
        rospy.init_node('ros_lane_consumer')
        self.bridge = CvBridge()
        self.config = None
        self.detector = None
        self.image_subscription = rospy.Subscriber("lanes_video", Image, self.callback)
        self.departure_publisher = rospy.Publisher("image_lane_departure", String, queue_size=10)
        self.image_publisher = rospy.Publisher("image_lane_detector", Image, queue_size=10)
        self.stream_subscription = rospy.Subscriber("video_stream", Bool, self.runs_tream) 
    def runs_tream(self, data2):
        global x
        x = data2.data
    def callback(self, data):
        try:
            # Read the image from the input topic:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e

        # If the config hasn't init yet, init it:
        if self.config is None:
            height = cv_image.shape[0]
            width = cv_image.shape[1]
            self.config = _create_config(height, width)
            self.detector = LaneDepartureDetector(self.config[0], self.config[1], self.config[2])

        # Run the lane detector on the input:
        (processed_image, departure) = self.detector.process_image(cv_image)

        # Publish the departure
        self.departure_publisher.publish(str(departure))
        print("The distance from the middle of the lane is: " + str(departure))

        # Publish the processed image:
        try:
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(processed_image))
	    print y
            if x == 1 and y==1:
                print "sreaming video is working"
                cv2.imshow('Departure Warning', processed_image)
        except CvBridgeError as e:
            print(e)

        cv2.waitKey(1) & 0xFF
       
        
   


def main(args):
    global y
    # Init the consumer:
    consumer = LaneDepartureConsumer()

    # Continue to spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
	y=0
        print "Shutting down vison node."


if __name__ == '__main__':
    main(sys.argv)
