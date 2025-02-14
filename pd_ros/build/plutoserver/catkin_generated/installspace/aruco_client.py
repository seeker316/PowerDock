# #!/usr/bin/env python

# import rospy
# from plutodrone.srv import aruco, arucoRequest

# def add_two_ints_client(x, y):
#     rospy.wait_for_service('add_two_ints')
#     try:
#         add_two_ints = rospy.ServiceProxy('add_two_ints', aruco)
#         req = arucoRequest(x, y)
#         resp = add_two_ints(req.a, req.b)
#         return resp.sum
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s" % e)

# if __name__ == "__main__":
#     rospy.init_node('add_two_ints_client')
#     x, y = 5, 10  # Example inputs
#     rospy.loginfo(f"Requesting {x} + {y}")
#     result = add_two_ints_client(x, y)
#     rospy.loginfo(f"Result: {result}")

#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import GetArray, GetArrayRequest
from std_msgs.msg import Int16MultiArray

def get_array_client(size):
    rospy.wait_for_service('get_array')
    try:
        get_array = rospy.ServiceProxy('get_array', GetArray)
        req = GetArrayRequest(size)
        resp = get_array(req.size)
        return resp.data
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('get_array_client')
    size = 5  # Example size request
    rospy.loginfo(f"Requesting an array of size {size}")
    result = get_array_client(size)
    rospy.loginfo(f"Received array: {result}")
