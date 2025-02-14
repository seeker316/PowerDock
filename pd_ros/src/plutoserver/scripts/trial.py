# import cv2
# import numpy as np
# import time

# def map_range(x, new_min=25, new_max=80, old_min=15, old_max=45):
#     # Apply linear mapping formula
#     return ((x - new_min) / (new_max - new_min)) * (old_max - old_min) + old_min

# KP = 1.5  # Proportional gain
# KI = 0.01  # Integral gain
# KD = 0.5  # Derivative gain

# setpoint = 30  # Target marker distance in cm (similar to DownHold)
# prev_error = 0
# integral = 0
# dt = 0.1 

# def pid_controller(error):
#     """ PID controller to minimize marker distance error """
#     global prev_error, integral, dt
#     integral += error * dt
#     derivative = (error - prev_error) / dt
#     output = (KP * error) + (KI * integral) + (KD * derivative)
#     prev_error = error
#     return output

# # Define the ArUco dictionary and parameters
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
# aruco_params = cv2.aruco.DetectorParameters()

# camera_matrix = np.array([[437.48477209,0,479.37404252],
#                             [  0, 438.32360844,259.97078463],
#                             [  0,0,1] ])

# dist_coeffs = np.array([-0.06249348 , 0.14815148, -0.00491044, -0.00591934, -0.12213315])
# # Define the marker size in meters (e.g., 0.05 meters)
# marker_size = 0.05

# # Function to calculate yaw angle from rotation vector
# def get_yaw_angle(rvec):
#     R, _ = cv2.Rodrigues(rvec)  # Convert rotation vector to matrix
#     yaw = np.arctan2(R[1, 0], R[0, 0])  # Calculate yaw from the rotation matrix
#     return np.degrees(yaw)  # Convert radians to degrees

# # Function to dynamically resize the frame based on marker size
# # def resize_frame(frame, marker_distance, min_size=200, max_size=500):
# #     # Normalize marker distance to a range [0, 1]
# #     norm_distance = np.clip((marker_distance - 0.1) / 2.0, 0, 1)
# #     # Calculate new frame size based on normalized distance
# #     new_size = int(min_size + norm_distance * (max_size - min_size))
# #     return cv2.resize(frame, (new_size, new_size))

# # Initialize video capture
# cap = cv2.VideoCapture(2)  # Use camera index or video file path

# # Variables to calculate FPS
# fps = 0
# prev_time = time.time()

# while cap.isOpened():
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to capture frame")
#         break
#     # frame = cv2.resize(frame,(150,100))
#     frame = cv2.resize(frame, (1080,720))
#     # Convert frame to grayscale
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Create a blank image for displaying text
#     dashboard = np.zeros((300, 600, 3), dtype=np.uint8)

#     # Get frame dimensions and calculate center
#     frame_height, frame_width = frame.shape[:2]
#     # print(frame_width,frame_height)
#     frame_center = (frame_width / 2, frame_height / 2)

#     # Detect markers
#     corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

#     if ids is not None:  # If markers are detected
#         for i in range(len(ids)):
#             # Estimate pose of each marker
#             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
            
#             # Draw marker boundary and axis
#             cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#             cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size)

#             # Calculate marker center (image coordinates)
#             marker_center_x = (corners[i][0][0][0] + corners[i][0][2][0]) / 2
#             marker_center_y = (corners[i][0][0][1] + corners[i][0][2][1]) / 2
#             marker_center = (marker_center_x, marker_center_y)

#             # Calculate error from frame center
#             x_error_image = marker_center[0] - frame_center[0]
#             y_error_image = frame_center[1] - marker_center[1]

           
#             yaw_angle = get_yaw_angle(rvec[0])

#             # Display data on the dashboard
#             cv2.putText(dashboard, f"X Error: {x_error_image:.2f} px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
#             cv2.putText(dashboard, f"Y Error: {y_error_image:.2f} px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
#             cv2.putText(dashboard, f"Yaw: {yaw_angle:.2f} deg", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
#             print(f"X Error: {x_error_image:.2f} px")
#             print(f"Y Error: {y_error_image:.2f} px")
#             print(f"Yaw: {yaw_angle:.2f} deg")
#             marker_distance = tvec[0][0][2]*100
#             print("distance in cm",marker_distance)
            
#             error = setpoint - marker_distance
            
#             pid_output = pid_controller(error)
#             throttle_value = 1500 + pid_output 
#             print(throttle_value)
            
#             cv2.putText(frame, f"ghc: {throttle_value:.2f}", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


#             cv2.putText(dashboard, f"Marker Distance: {marker_distance:.2f} px", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
#             # frame = resize_frame(frame, marker_distance)

#     # Calculate FPS
#     current_time = time.time()
#     fps = 1 / (current_time - prev_time)
#     prev_time = current_time

#     # Display FPS on the frame
#     cv2.putText(frame, f"FPS: {fps:.2f}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
#     # Display the frame
#     cv2.imshow("ArUco Marker Detection", frame)

#     # Display the dashboard
#     cv2.imshow("Data Dashboard", dashboard)

#     # Break on 'q' key press
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Release resources
# cap.release()
# cv2.destroyAllWindows()

import rospy
from std_msgs.msg import Float32MultiArray

def pid_callback(msg):
    """Callback function to process received PID values."""
    roll, pitch, yaw, throttle = msg.data  # Extract values from the message

    print(f"📩 Received PID Values:")
    print(f"  🔹 Roll: {roll}")
    print(f"  🔹 Pitch: {pitch}")
    print(f"  🔹 Yaw: {yaw}")
    print(f"  🔹 Throttle: {throttle}")
    print("-" * 40)

def pid_listener():
    """Initialize the subscriber node and listen for PID values."""
    rospy.init_node("pid_subscriber", anonymous=True)
    rospy.Subscriber("/pid_values", Float32MultiArray, pid_callback)

    print("🟢 Listening for PID values on /pid_values...")
    rospy.spin()  # Keep the node running

if __name__ == "__main__":
    pid_listener()
