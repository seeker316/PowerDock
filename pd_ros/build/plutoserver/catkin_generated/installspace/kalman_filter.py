#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from plutodrone.srv import *
from plutodrone.msg import *
import rospy
from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import Imu
import numpy as np
from scipy.signal import firwin
import time

x1, y1, z1 = 0, 2, 0
x2, y2, z2 = 0, 0, 0
x3, y3, z3 = 2, 0, 0

def convert_raw_to_g(raw_value):
  sensitivity = 0.00976  # Sensitivity for ±2g range
  if raw_value >= 32768:
      raw_value = raw_value - 65536
  g_value = raw_value * sensitivity
  return g_value

def trilateration(s1,s2,s3):
    # Required Matrices
    A = np.array([
        [1, -2*x1, -2*y1, -2*z1],
        [1, -2*x2, -2*y2, -2*z2],
        [1, -2*x3, -2*y3, -2*z3]
    ])

    B = np.array([
        [s1**2 - x1**2 - y1**2 - z1**2],
        [s2**2 - x2**2 - y2**2 - z2**2],
        [s3**2 - x3**2 - y3**2 - z3**2]
    ])

    # Particular Solution (xp)
    xp = np.linalg.lstsq(A, B, rcond=None)[0]

    # Solution of the homogeneous system A.x = 0 (xh)
    xh = np.array([[0], [0], [0], [1]])

    # # Calculate C, D, and E
    C = xp[1]**2 + xp[2]**2 + xp[3]**2
    D = 2 * (xp[1]*xh[1] + xp[2]*xh[2] + xp[3]*xh[3])
    E = xh[1]**2 + xh[2]**2 + xh[3]**2


    # # Coefficients of the quadratic equation
    a = E
    b = D - xh[0]
    c = C - xp[0]

    # # Solve for t
    t = np.roots([a[0], b[0], c[0]])

    # # # General Solution (X)
    X1 = xp + t[1] * xh

    N1 = X1[1:4].flatten().real

    # print(f'X : {N1[0]:0.4f}  Y : {N1[1]:0.4f}  Z : {N1[2]:0.4f}')
    return N1[0], N1[1], N1[2]

def uwb_acc_offset_service(UWB_measured,ACC_measured):
  
    calib_U, calib_A = [], []
    offset_acc = [0, 0, 0]
    offset_uwb = [0, 0, 0]

    Fs = 400 / 14.586690
    Fc = 0.5
    filterOrder = 10
    bufferSize = filterOrder + 1
    FIR_coeff = firwin(filterOrder + 1, Fc / (Fs / 2), pass_zero=True)

    buffer_accel = np.zeros((3, bufferSize))
    buffer_uwb = np.zeros((3, bufferSize))

    fir_accel = np.zeros(3)
    fir_uwb = np.zeros(3)

    # for _ in range(200):
    raw_U, raw_A = (UWB_measured, ACC_measured)

    a_x = convert_raw_to_g(raw_A[0])
    a_y = convert_raw_to_g(raw_A[1])
    a_z = convert_raw_to_g(raw_A[2])
    
    

    if len(calib_A) < 100:
        calib_A.append(raw_A)
    else:
        a_x -= offset_acc[0]
        a_y -= offset_acc[1]
        a_z -= offset_acc[2]

        buffer_accel = np.hstack((np.array([[a_x], [a_y], [a_z]]), buffer_accel[:, :-1]))
        fir_accel = np.sum(buffer_accel * FIR_coeff, axis=1)

    if len(calib_U) < 50:
        calib_U.append(raw_U)
    else:
        s1, s2, s3 = raw_U
        s1 -= offset_uwb[0]
        s2 -= offset_uwb[1]
        s3 -= offset_uwb[2]

        buffer_uwb = np.hstack((np.array([[s1], [s2], [s3]]), buffer_uwb[:, :-1]))
        fir_uwb = np.sum(buffer_uwb * FIR_coeff, axis=1)
    
    if len(calib_A) == 100 and len(calib_U) == 50:
        offset_acc = np.mean(np.array(calib_A), axis=0) - np.array(ACC_measured)
        offset_uwb = np.mean(np.array(calib_U), axis=0) - np.array(UWB_measured)
        
        x, y, z = trilateration(s1, s2, s3)

    return x, y, z
    # Prepare the response with the offsets
    # offset_acc_msg = Float64MultiArray(data=offset_acc)
    # offset_uwb_msg = Float64MultiArray(data=offset_uwb)
    

    # rospy.loginfo("Offsets calculated:")
    # rospy.loginfo(f"Accelerometer Offset: {offset_acc}")
    # rospy.loginfo(f"UWB Offset: {offset_uwb}")
    # rospy.loginfo(f"UWB Values - s1: {s1}, s2: {s2}, s3: {s3}")
    
    # return offset_acc_msg, offset_uwb_msg


def access_data(req):
    UWB_measured = [req.u1, req.u2, req.u3]
    ACC_measured = [0,0,0]
    # ACC_measured = [req.a1, req.a2, req.a3]

    # Call the offset service and get XYZ coordinates
    x, y, z = uwb_acc_offset_service(UWB_measured, ACC_measured)
    print(x,y,z)
    # Prepare the response with XYZ coordinates
    # response = PlutoPilotResponse()
    # response.x = x
    # response.y = y
    # response.z = z
    # return response


def start_service():
    rospy.init_node('trilateration_node')
    service = rospy.Service('PlutoService', PlutoPilot, access_data)
    
    rospy.loginfo("Service started.")
    rospy.spin()

if __name__ == "__main__":
    try:
        start_service()
    except rospy.ROSInterruptException:
        pass