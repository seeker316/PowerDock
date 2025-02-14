#!/usr/bin/env python3

import rospy
from plutodrone.srv import aruco, arucoResponse

def handle_add_two_ints(req):
    rospy.loginfo(f"Adding {req.a} + {req.b}")
    return arucoResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    service = rospy.Service('add_two_ints', aruco, handle_add_two_ints)
    rospy.loginfo("Ready to add two integers.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()


#!/usr/bin/env python

import rospy
import random
from beginner_tutorials.srv import GetArray, GetArrayResponse
from std_msgs.msg import Int16MultiArray

def handle_get_array(req):
    rospy.loginfo(f"Received request for an array of size {req.size}")
    
    # Generate random int16 values
    response = GetArrayResponse()
    response.data = [random.randint(-32768, 32767) for _ in range(req.size)]
    
    rospy.loginfo(f"Sending array: {response.data}")
    return response

def get_array_server():
    rospy.init_node('get_array_server')
    service = rospy.Service('get_array', GetArray, handle_get_array)
    rospy.loginfo("Ready to provide arrays.")
    rospy.spin()

if __name__ == "__main__":
    get_array_server()
