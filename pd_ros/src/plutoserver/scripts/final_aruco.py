#!/usr/bin/env python
import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import Int16MultiArray
from plutodrone.srv import aruco, arucoResponse

# Initialize ROS Node
rospy.init_node("aruco_pid_publisher", anonymous=True)
pid_pub = rospy.Publisher("/pid_values", Int16MultiArray, queue_size=10)
rate = rospy.Rate(100)  # 30 Hz publishing rate


# Camera Parameters
camera_matrix = np.array([[437.48477209,0,479.37404252],
                            [  0, 438.32360844,259.97078463],
                            [  0,0,1] ])

dist_coeffs = np.array([-0.06249348 , 0.14815148, -0.00491044, -0.00591934, -0.12213315])
marker_size = 0.05  # Marker size in meters

# Function to Get Yaw Angle from Rotation Vector
def get_yaw_angle(rvec):
    R, _ = cv2.Rodrigues(rvec)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees(yaw)


url = "http://192.168.85.39:5050/video"

cap = cv2.VideoCapture(url)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# cap.set(cv2.CAP_PROP_FPS, 10)

# Load ArUco Dictionary & Detector Parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

last_known_position, last_known_distance, last_yaw_error = None, None, None
prev_gray, prev_points = None, None
lost_frames, fps_counter = 0, 0
fps_start_time = time.time()

# Main Loop - Process Frames
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    fps_counter += 1

    # Detect ArUco Markers
    corners, ids, _ = detector.detectMarkers(gray)
    detected = False
    error_x, error_y, yaw_error, estimated_distance = 0, 0, 0, 0

    alt_hold = 50    

    if ids is not None:
        for i in range(len(ids)):
            if ids[i][0] == 69:  # Track marker with ID 69
                detected = True
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                lost_frames = 0  

                marker_corners = corners[i][0]
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)

                last_known_position = (center_x, center_y)
                frame_center_x, frame_center_y = frame.shape[1] // 2, frame.shape[0] // 2
                error_x, error_y = center_x - frame_center_x, center_y - frame_center_y

                # Estimate Pose
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size)
                last_known_distance = np.linalg.norm(tvec[0][0])
                yaw_error = get_yaw_angle(rvec[0])
                last_yaw_error = yaw_error  

    # Optical Flow for Marker Loss Handling
    if not detected and last_known_position:
        lost_frames += 1
        if prev_gray is not None and prev_points is not None:
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_points, None)
            if next_points is not None and status[0] == 1:
                last_known_position = (int(next_points[0][0][0]), int(next_points[0][0][1]))
                error_x, error_y = last_known_position[0] - frame_center_x, last_known_position[1] - frame_center_y
                estimated_distance, yaw_error = last_known_distance, last_yaw_error
                cv2.circle(frame, last_known_position, 10, (255, 0, 0), -1)
                cv2.putText(frame, "Optical Flow Tracking", (last_known_position[0] + 10, last_known_position[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    estimated_distance = estimated_distance if estimated_distance else last_known_distance or 1.5
    
    if estimated_distance < 1.8:

        alt_error = alt_hold - (estimated_distance*100)
        roll = roll_pid.compute(error_x)
        pitch = pitch_pid.compute(error_y)
        yaw = yaw_pid.compute(yaw_error)
        throttle = throttle_pid.compute(alt_error)

        # roll, pitch = pid_roll.compute(error_x), pid_pitch.compute(error_y)
        # yaw, throttle = pid_yaw.compute(yaw_error), pid_throttle.compute(alt_error)
        cv2.putText(frame, f"Distance{estimated_distance:.2f}  Error: {alt_error}", (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # ROS Publisher
    msg = Int16MultiArray()
    msg.data = [int(roll), int(pitch), int(yaw), int(throttle)]
    pid_pub.publish(msg)
    print(f"📡 Published: Roll={roll}, Pitch={pitch}, Yaw={yaw}, Throttle={throttle}")
    print(alt_error)

    # Update for Optical Flow
    prev_gray = gray.copy()
    if last_known_position:
        prev_points = np.array([[last_known_position]], dtype=np.float32)

    # Display on Screen
    cv2.putText(frame, f"Roll: {roll}  Pitch: {pitch}  Yaw: {yaw}  Throttle: {int(throttle)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"Error X: {error_x}  Error Y: {error_y}  Yaw Error: {yaw_error:.2f}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.putText(frame, f"FPS: {fps_counter/(time.time()-fps_start_time):.2f}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    

    cv2.imshow("Aruco Tracking with PID & Optical Flow", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    rate.sleep()

cap.release()
cv2.destroyAllWindows()
