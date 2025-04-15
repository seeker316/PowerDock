#!/usr/bin/env python3

import rospy
from plutodrone.srv import SetPos, SetPosResponse

def handle_set_position(req):
    rospy.loginfo("Received position request: x=%.2f, y=%.2f, z=%.2f", req.pos_x, req.pos_y, req.pos_z)

    # Echo back the values as confirmation (can be modified if needed)
    res = SetPosResponse()
    res.set_x = req.pos_x
    res.set_y = req.pos_y
    res.set_z = req.pos_z

    rospy.loginfo("Responding with: set_x=%.2f, set_y=%.2f, set_z=%.2f", res.set_x, res.set_y, res.set_z)
    return res

def pdserver():
    rospy.init_node('pdserver')
    service = rospy.Service('set_position', SetPos, handle_set_position)
    rospy.loginfo("SetPos service is ready to receive requests.")
    rospy.spin()

if __name__ == "__main__":
    pdserver()
