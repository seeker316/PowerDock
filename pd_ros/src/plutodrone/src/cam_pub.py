#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def camera_feed_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_feed_publisher', anonymous=True)

    # Set up the publisher to publish to the topic "/camera/image_raw"
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    # Create a CvBridge object to convert OpenCV images to ROS Image messages
    bridge = CvBridge()

    # Open a connection to the camera (0 is usually the default webcam)
    cap = cv2.VideoCapture(6)

    # Check if the camera is opened properly
    if not cap.isOpened():
        rospy.logerr("Could not open camera.")
        return

    rate = rospy.Rate(10)  # Set the publish rate to 10 Hz

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()

        if ret:
            # Convert the OpenCV image (BGR format) to a ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the image
            image_pub.publish(ros_image)

            # Log the publishing status
            rospy.loginfo("Published a new frame to /camera/image_raw")
        
        # Sleep to maintain the desired publishing rate
        rate.sleep()

    # Release the camera when the node is shut down
    cap.release()

if __name__ == '__main__':
    try:
        camera_feed_publisher()
    except rospy.ROSInterruptException:
        pass
