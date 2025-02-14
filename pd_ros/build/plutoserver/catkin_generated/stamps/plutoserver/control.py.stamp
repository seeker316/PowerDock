# #!/usr/bin/env python
# from plutodrone.srv import *
# from plutodrone.msg import *
# from std_msgs.msg import Int16
# import rospy

# control = False

# class send_data():
# 	"""docstring for request_data"""
# 	def __init__(self):
# 		rospy.init_node('drone_server')
# 		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

# 		self.key_value =0
# 		self.cmd = PlutoMsg()
# 		self.cmd.rcRoll =1500
# 		self.cmd.rcPitch = 1500
# 		self.cmd.rcYaw =1500
# 		self.cmd.rcThrottle =1500
# 		self.cmd.rcAUX1 =1500
# 		self.cmd.rcAUX2 =1500
# 		self.cmd.rcAUX3 =1500
# 		self.cmd.rcAUX4 =1000
# 		self.cmd.commandType = 0
# 		self.cmd.trim_roll = 0
# 		self.cmd.trim_pitch = 0
# 		self.cmd.isAutoPilotOn = 0

# 		rospy.Subscriber('/input_key', Int16, self.indentify_key )

# 	def arm(self):
# 		self.cmd.rcRoll=1500
# 		self.cmd.rcYaw=1500
# 		self.cmd.rcPitch =1500
# 		self.cmd.rcThrottle =1000
# 		self.cmd.rcAUX4 =1500
# 		self.cmd.isAutoPilotOn = 0
# 		self.command_pub.publish(self.cmd)
# 		rospy.sleep(1)

# 	def box_arm(self):
# 		self.cmd.rcRoll=1500
# 		self.cmd.rcYaw=1500
# 		self.cmd.rcPitch =1500
# 		self.cmd.rcThrottle =1500
# 		self.cmd.rcAUX4 =1500
# 		self.cmd.isAutoPilotOn = 0
# 		self.command_pub.publish(self.cmd)
# 		rospy.sleep(0.5)

# 	def disarm(self):
# 		self.cmd.rcThrottle =1300
# 		self.cmd.rcAUX4 = 1200
# 		self.command_pub.publish(self.cmd)
# 		rospy.sleep(0.5)

# 	def indentify_key(self, msg):
# 		self.key_value = msg.data

# 		if self.key_value == 70:
# 			if(self.cmd.rcAUX4 == 1500):
# 				self.disarm()
# 			else:
# 				self.arm()
# 		elif self.key_value == 10:
# 			self.forward()
# 		elif self.key_value == 30:
# 			self.left()
# 		elif self.key_value == 40:
# 			self.right()
# 		elif self.key_value == 80:
# 			self.reset()
# 		elif self.key_value == 90:
# 			if(self.cmd.isAutoPilotOn == 1):
# 				self.cmd.isAutoPilotOn = 0
# 			else:
# 				self.cmd.isAutoPilotOn = 1
# 		elif self.key_value == 50:
# 			self.increase_height()
# 		elif self.key_value == 60:
# 			self.decrease_height()
# 		elif self.key_value == 110:
# 			self.backward()
# 		elif self.key_value == 130:
# 			self.take_off()
# 		elif self.key_value == 140:
# 			self.land()
# 		elif self.key_value == 150:
# 			self.left_yaw()
# 		elif self.key_value == 160:
# 			self.right_yaw()
# 	    elif self.keyvalue == 85:
#             control = True
#         elif self.keyvalue == 85:
#             control = False
#         self.command_pub.publish(self.cmd)
	
# 	def forward(self):
# 		self.cmd.rcPitch =1600
# 		self.command_pub.publish(self.cmd)
# 	def backward(self):
# 		self.cmd.rcPitch =1400
# 		self.command_pub.publish(self.cmd)
# 	def left(self):
# 		self.cmd.rcRoll =1400
# 		self.command_pub.publish(self.cmd)
# 	def right(self):
# 		self.cmd.rcRoll =1600
# 		self.command_pub.publish(self.cmd)
# 	def left_yaw(self):
# 		self.cmd.rcYaw = 1200
# 		self.command_pub.publish(self.cmd)
# 	def right_yaw(self):
# 		self.cmd.rcYaw = 1800
# 		self.command_pub.publish(self.cmd)
# 	def reset(self):
# 		self.cmd.rcRoll =1500
# 		self.cmd.rcThrottle =1500
# 		self.cmd.rcPitch =1500
# 		self.cmd.rcYaw = 1500
# 		self.cmd.commandType = 0
# 		self.command_pub.publish(self.cmd)
# 	def increase_height(self):
# 		self.cmd.rcThrottle = 1800
# 		self.command_pub.publish(self.cmd)
# 	def decrease_height(self):
# 		self.cmd.rcThrottle =1300
# 		self.command_pub.publish(self.cmd)
# 	def take_off(self):
# 		self.disarm()
# 		self.box_arm()
# 		self.cmd.commandType = 1
# 		self.command_pub.publish(self.cmd)
# 	def land(self):
# 		self.cmd.commandType = 2
# 		self.command_pub.publish(self.cmd)
			
#     def control(self):
#         self.cmd.rcRoll =1500
#         self.cmd.rcThrottle =1500
#         self.cmd.rcPitch =1500
#         self.cmd.rcYaw = 1500
#         self.command_pub.publish(self.cmd)

# if __name__ == '__main__':
# 	test = send_data()
# 	while not rospy.is_shutdown():
# 		rospy.spin()
# 		sys.exit(1)

#!/usr/bin/env python
import rospy
import sys
from plutodrone.msg import PlutoMsg
from std_msgs.msg import Float32MultiArray, Int16

control = False  # Variable to enable/disable autonomous control

class DroneControl:
    """Drone Control Node that listens for PID values and sends commands to the drone."""

    def __init__(self):
        rospy.init_node('drone_server')

        # Publishers
        self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

        # Subscribers
        rospy.Subscriber('/pid_values', Float32MultiArray, self.pid_callback)  # Subscribe to PID values
        rospy.Subscriber('/input_key', Int16, self.identify_key)  # Subscribe to keyboard input

        # Initialize command message
        self.cmd = PlutoMsg()
        self.reset_command()

        # Store PID values
        self.pid_roll = 1500
        self.pid_pitch = 1500
        self.pid_yaw = 1500
        self.pid_throttle = 1500

    def reset_command(self):
        """Reset all command values to neutral positions."""
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        self.cmd.commandType = 0
        self.cmd.trim_roll = 0
        self.cmd.trim_pitch = 0
        self.cmd.isAutoPilotOn = 0

    def pid_callback(self, msg):
        """Receive PID values and store them."""
        global control
        self.pid_roll, self.pid_pitch, self.pid_yaw, self.pid_throttle = msg.data

        # If autonomous control is enabled, apply PID values
        if control:
            self.control()

    def identify_key(self, msg):
        """Handle keyboard input commands."""
        global control
        key_value = msg.data

        if key_value == 70:  # Arm/Disarm Toggle
            if self.cmd.rcAUX4 == 1500:
                self.disarm()
            else:
                self.arm()
        elif key_value == 10:
            self.forward()
        elif key_value == 30:
            self.left()
        elif key_value == 40:
            self.right()
        elif key_value == 80:
            self.reset_command()
        elif key_value == 90:
            self.cmd.isAutoPilotOn = 1 - self.cmd.isAutoPilotOn  # Toggle autopilot
        elif key_value == 50:
            self.increase_height()
        elif key_value == 60:
            self.decrease_height()
        elif key_value == 110:
            self.backward()
        elif key_value == 130:
            self.take_off()
        elif key_value == 140:
            self.land()
        elif key_value == 150:
            self.left_yaw()
        elif key_value == 160:
            self.right_yaw()
        elif key_value == 85:  # Enable control mode
            control = True
        elif key_value == 86:  # Disable control mode
            control = False

        self.command_pub.publish(self.cmd)

    def control(self):
        """Apply PID control values to the drone."""
        if control:
			self.cmd.rcRoll = int(self.pid_roll)
			self.cmd.rcPitch = int(self.pid_pitch)
			self.cmd.rcYaw = int(self.pid_yaw)
			self.cmd.rcThrottle = int(self.pid_throttle)
			self.command_pub.publish(self.cmd)
			print(f"🔹 Applying PID Control: Roll={self.pid_roll}, Pitch={self.pid_pitch}, Yaw={self.pid_yaw}, Throttle={self.pid_throttle}")
		else:
            print("ACTIVATE AUTO PILOT")
    def arm(self):
        """Arm the drone."""
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.cmd.isAutoPilotOn = 0
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def disarm(self):
        """Disarm the drone."""
        self.cmd.rcThrottle = 1300
        self.cmd.rcAUX4 = 1200
        self.command_pub.publish(self.cmd)
        rospy.sleep(0.5)

    def forward(self):
        self.cmd.rcPitch = 1600
        self.command_pub.publish(self.cmd)

    def backward(self):
        self.cmd.rcPitch = 1400
        self.command_pub.publish(self.cmd)

    def left(self):
        self.cmd.rcRoll = 1400
        self.command_pub.publish(self.cmd)

    def right(self):
        self.cmd.rcRoll = 1600
        self.command_pub.publish(self.cmd)

    def left_yaw(self):
        self.cmd.rcYaw = 1200
        self.command_pub.publish(self.cmd)

    def right_yaw(self):
        self.cmd.rcYaw = 1800
        self.command_pub.publish(self.cmd)

    def increase_height(self):
        self.cmd.rcThrottle = 1800
        self.command_pub.publish(self.cmd)

    def decrease_height(self):
        self.cmd.rcThrottle = 1300
        self.command_pub.publish(self.cmd)

    def take_off(self):
        self.disarm()
        self.arm()
        self.cmd.commandType = 1
        self.command_pub.publish(self.cmd)

    def land(self):
        self.cmd.commandType = 2
        self.command_pub.publish(self.cmd)


if __name__ == '__main__':
    try:
        drone = DroneControl()
        while not rospy.is_shutdown():
            drone.control
            rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit(1)
