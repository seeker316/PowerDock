#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from plutodrone.srv import *
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
import numpy as np
from scipy.signal import firwin

UWB_measured = [1.28, 1.4, 1.55]
ACC_measured = [0, 0, -9.8066]
sim_error = [0.08, 0.01]

calib_U, calib_A = [], []
offset_acc = [0, 0, 0]
offset_uwb = [0, 0, 0]

x1, y1, z1 = 0, 1, 0
x2, y2, z2 = 0, 0, 0
x3, y3, z3 = 1, 0, 0

Fs = 400 / 14.586690
Fc = 0.5
filterOrder = 10
bufferSize = filterOrder + 1

FIR_coeff = firwin(filterOrder + 1, Fc / (Fs / 2), pass_zero=True)

buffer_accel = np.zeros((3, bufferSize))
buffer_uwb = np.zeros((3, bufferSize))


def convert_raw_to_g(raw_value):
    sensitivity = 0.00976  # Sensitivity for ±2g range
    if raw_value >= 32768:
        raw_value = raw_value - 65536
    g_value = raw_value * sensitivity
    return g_value


def trilateration(s1, s2, s3):
    A = np.array([
        [1, -2 * x1, -2 * y1, -2 * z1],
        [1, -2 * x2, -2 * y2, -2 * z2],
        [1, -2 * x3, -2 * y3, -2 * z3]
    ])

    B = np.array([
        [s1**2 - x1**2 - y1**2 - z1**2],
        [s2**2 - x2**2 - y2**2 - z2**2],
        [s3**2 - x3**2 - y3**2 - z3**2]
    ])
    xp = np.linalg.lstsq(A, B, rcond=-1)[0]
    # xp = np.linalg.lstsq(A, B, rcond=None)[0]
    xh = np.array([[0], [0], [0], [1]])

    C = xp[1]**2 + xp[2]**2 + xp[3]**2
    D = 2 * (xp[1] * xh[1] + xp[2] * xh[2] + xp[3] * xh[3])
    E = xh[1]**2 + xh[2]**2 + xh[3]**2

    a = E
    b = D - xh[0]
    c = C - xp[0]

    t = np.roots([a[0], b[0], c[0]])
    X1 = xp + t[1] * xh
    N1 = X1[1:4].flatten().real

    print('X : {:.4f}  Y : {:.4f}  Z : {:.4f}'.format(N1[0], N1[1], N1[2]))


def sampler(ACC, error):
    ind_a = []
    for a in ACC:
        ind_a.append(np.round(np.random.uniform(a - error[1], a + error[1]), 3))
    return ind_a


class request_data():
    """docstring for request_data"""

    def __init__(self):
        rospy.init_node('drone_board_data')
        data = rospy.Service('PlutoService', PlutoPilot, self.access_data)

    def access_data(self, req):
        req.a1, req.a2, req.a3 = req.a1 / 536870912, req.a2 / 536870912, req.a3 / 536870912
        fir_accel = np.zeros(3)
        fir_uwb = np.zeros(3)

        raw_A = sampler(ACC_measured, sim_error)
        raw_U = [req.a1, req.a2, req.a3]

        a_x = convert_raw_to_g(raw_A[0])
        a_y = convert_raw_to_g(raw_A[1])
        a_z = convert_raw_to_g(raw_A[2])
        print("hi....")

        if len(calib_A) < 100:
            calib_A.append(raw_A)
        else:
            a_x = a_x - offset_acc[0]
            a_y = a_y - offset_acc[1]
            a_z = a_z - offset_acc[2]

            buffer_accel[:, 1:] = buffer_accel[:, :-1]
            buffer_accel[:, 0] = [a_x, a_y, a_z]
            fir_accel = np.sum(buffer_accel * FIR_coeff, axis=1)

        if len(calib_U) < 50:
            calib_U.append(raw_U)
        else:
            [s1, s2, s3] = raw_U
            s1 = s1 - offset_uwb[0]
            s2 = s2 - offset_uwb[1]
            s3 = s3 - offset_uwb[2]
            print("yo")
            buffer_uwb[:, 1:] = buffer_uwb[:, :-1]
            buffer_uwb[:, 0] = [s1, s2, s3]
            fir_uwb = np.sum(buffer_uwb * FIR_coeff, axis=1)

        if (len(calib_A) == 100) and (len(calib_U) == 50):
            offset_acc[:] = np.mean(np.array(calib_A), axis=0) - np.array(ACC_measured)
            offset_uwb[:] = np.mean(np.array(calib_U), axis=0) - np.array(UWB_measured)
            print(offset_uwb)
            print("####### CALIBRATION FINISHED #######")

        s1, s2, s3 = fir_uwb
        print(fir_uwb)
        trilateration(s1, s2, s3)

        rospy.sleep(.1)
        return PlutoPilotResponse(rcAUX2=1500)


test = request_data()
rospy.spin()
