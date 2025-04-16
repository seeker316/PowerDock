#!/usr/bin/env python3

from kivy.lang import Builder
from kivymd.app import MDApp
from kivy.uix.widget import Widget
from kivy.graphics import Ellipse, Line, Color, InstructionGroup
from kivy.core.window import Window
from kivy.clock import Clock
from kivy.uix.image import Image
from kivymd.uix.button import MDRaisedButton
from kivymd.uix.label import MDLabel
from kivy.uix.boxlayout import BoxLayout
import rospy
import threading
from plutodrone.msg import Drone_stats
import math
from kivy.garden import joystick
import numpy as np
import cv2
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from kivy.graphics.texture import Texture

from kivy.core.window import Window
from std_msgs.msg import Int16

import time
from plutodrone.srv import SetPos,SetPosResponse


KV = '''
BoxLayout:
    orientation: 'vertical'
    padding: 20
    spacing: 20
    canvas.before:
        Color:
            rgba: 0.1, 0.1, 0.1, 1  # Dark background for the whole app
        Rectangle:
            size: self.size
            pos: self.pos

    # Top Section: Path Planning + Buttons + Telemetry
    BoxLayout:
        orientation: 'horizontal'
        size_hint_y: 0.5
        spacing: 10

        # Path Planning (2/5 width)
        MDBoxLayout:
            orientation: 'vertical'
            size_hint_x: 2/5
            md_bg_color: 0.9, 0.9, 0.9, 1  # Dark Gray

            MDTopAppBar:
                title: "Path Planning"
                elevation: 10
                md_bg_color: 0.13, 0.13, 0.13, 1  # Darker for consistency
                specific_text_color: 1, 1, 1, 1  # White text

            MDBoxLayout:
                orientation: 'horizontal'

                # Path Planning Widget (Takes Most of the Space)
                PathPlannerWidget:
                    id: path_planner
                    size_hint_x: 0.9  # 90% of the available width

                # Vertical Slider on the Right
                MDSlider:
                    id: path_slider
                    orientation: 'vertical'
                    min: 0
                    max: 100
                    value: 50  # Default Middle Position
                    size_hint_x: 0.1  # 10% of the width
                    size_hint_y: 1  # Full height
                    on_touch_up: app.snap_slider(self) 

        # Path Planning Buttons (1/5 width)
        MDBoxLayout:
            orientation: 'vertical'
            size_hint_x: 1/5
            spacing: 5  # Reduced spacing between buttons to make them fit
            padding: [10, 10, 10, 10]  # Optional: Add padding if needed

            MDRaisedButton:
                text: "Reset Path"
                on_release: app.reset_path()
                size_hint: 1, None  # Take up full width of allocated space
                height: "50dp"  # Reduced height to fit all buttons
                md_bg_color: 0.12, 0.6, 1, 1  # Light Blue
                pos_hint: {"center_x": 0.5}
                elevation: 10

            MDRaisedButton:
                text: "Save Path"
                on_release: app.save_path()
                size_hint: 1, None
                height: "50dp"  # Reduced height to fit all buttons
                md_bg_color: 0.12, 0.6, 1, 1
                pos_hint: {"center_x": 0.5}
                elevation: 10

            MDRaisedButton:
                text: "Execute Path"
                on_release: app.execute_path()
                size_hint: 1, None
                height: "50dp"  # Reduced height to fit all buttons
                md_bg_color: 0.12, 0.6, 1, 1
                pos_hint: {"center_x": 0.5}
                elevation: 10

            MDRaisedButton:
                text: "Pause/Resume"
                on_release: app.pause_resume()
                size_hint: 1, None
                height: "50dp"  # Adjusted height
                md_bg_color: 0.12, 0.6, 1, 1
                pos_hint: {"center_x": 0.5}
                elevation: 10

            MDRaisedButton:
                text: "Dock"
                on_release: app.dock()
                size_hint: 1, None
                height: "50dp"  # Adjusted height
                md_bg_color: 0.12, 0.6, 1, 1
                pos_hint: {"center_x": 0.5}
                elevation: 10

            MDRaisedButton:
                text: "Emergency Land"
                on_release: app.emergency_land()
                size_hint: 1, None
                height: "50dp"  # Adjusted height
                md_bg_color: 0.12, 0.6, 1, 1
                pos_hint: {"center_x": 0.5}
                elevation: 10


        # Telemetry (2/5 width)
        MDBoxLayout:
            orientation: 'vertical'
            size_hint_x: 2/5
            md_bg_color: 0.2, 0.2, 0.2, 1  # Dark Gray

            MDTopAppBar:
                title: "Telemetry"
                elevation: 10
                md_bg_color: 0.13, 0.13, 0.13, 1
                specific_text_color: 1, 1, 1, 1

            MDLabel:
                id: telemetry_data
                text: "Telemetry Data: Waiting..."
                halign: "center"
                theme_text_color: "Custom"
                text_color: 1, 1, 1, 1
                font_style: "H5"

    # Bottom Section: Camera Feed + Manual Controls + BMS
    BoxLayout:
        orientation: 'horizontal'
        size_hint_y: 0.5
        spacing: 10

        # Camera Feed (2/6 width)
        MDBoxLayout:
        
            orientation: 'vertical'
            size_hint_x: 2/6
            md_bg_color: 0.1, 0.1, 0.1, 1  # Dark background

            MDTopAppBar:
                title: "Camera Feed"
                elevation: 10
                md_bg_color: 0.13, 0.13, 0.13, 1

            Image:
                id: camera_feed
                size_hint: 1, 1
                fit_mode: 'contain'



        # Manual Control Buttons (2/6 width)
        MDBoxLayout:
            orientation: 'vertical'
            size_hint_x: 2/6
            spacing: 20
            padding: 20
            md_bg_color: 0.15, 0.15, 0.15, 1

            # Joysticks Section (Centered)
            MDBoxLayout:
                orientation: 'horizontal'
                spacing: 10
                size_hint_y: 0.3
                pos_hint: {"center_x": 0.5}

                # Left Joystick with WASD Labels
                FloatLayout:
                    size_hint: 0.4, 1
                    pos_hint: {"center_x": 0.5}

                    Joystick:
                        id: joystick_left
                        size_hint: 1, 1
                        pos_hint: {"center_x": 0.5, "center_y": 0.5}

                    # Adjusted Labels (Placed Further Apart)
                    MDLabel:
                        text: "W"
                        bold: True
                        font_size: "22sp"
                        halign: "center"
                        pos_hint: {"center_x": 0.5, "top": 1.35}  # Moved farther up

                    MDLabel:
                        text: "A"
                        bold: True
                        font_size: "22sp"
                        halign: "center"
                        pos_hint: {"x": -0.25, "center_y": 0.5}  # Moved farther left (x: -0.2)

                    MDLabel:
                        text: "D"
                        bold: True
                        font_size: "22sp"
                        halign: "center"
                        pos_hint: {"right": 1.25, "center_y": 0.5}  # Moved farther right (right: 1.2)

                    MDLabel:
                        text: "S"
                        bold: True
                        font_size: "22sp"
                        halign: "center"
                        pos_hint: {"center_x": 0.5, "y": -0.35}  # Moved farther down (y: -0.1)

                # Right Joystick (No Labels)
                Joystick:
                    id: joystick_right
                    size_hint: 0.4, 1
                    pos_hint: {"center_x": 0.5}

            # Manual / Auto Switch
            MDBoxLayout:
                orientation: 'vertical'
                spacing: 5
                size_hint_y: None
                height: "80dp"

                MDLabel:
                    text: "Manual / Auto"
                    text_color: 1, 1, 1, 1
                    halign: "center"
                    font_style: "H6"

                MDSwitch:
                    id: auto_toggle
                    size_hint_x: None
                    width: "100dp"
                    pos_hint: {"center_x": 0.5}
                    on_active: app.toggle_auto(self.active)

            # Arm / Disarm Switch
            MDBoxLayout:
                orientation: 'vertical'
                spacing: 5
                size_hint_y: None
                height: "80dp"

                MDLabel:
                    text: "Arm / Disarm"
                    text_color: 1, 1, 1, 1
                    halign: "center"
                    font_style: "H6"

                MDSwitch:
                    id: arm_toggle
                    size_hint_x: None
                    width: "100dp"
                    pos_hint: {"center_x": 0.5}
                    on_active: app.toggle_arm(self.active)

        # Status Bar (2/6 width)
        MDBoxLayout:
            orientation: 'vertical'
            size_hint_x: 2/6
            md_bg_color: 0.2, 0.2, 0.2, 1

            MDTopAppBar:
                title: "Status Bar"
                elevation: 10
                md_bg_color: 0.13, 0.13, 0.13, 1

            MDLabel:
                id: bms_status
                text: "System Status: Normal"
                halign: "center"
                theme_text_color: "Custom"
                text_color: 1, 1, 1, 1
                font_style: "H5"

            MDLabel:
                text: "Battery Level:"
                halign: "center"
                theme_text_color: "Custom"
                text_color: 1, 1, 1, 1
                font_style: "Subtitle1"

            MDProgressBar:
                id: battery_bar
                value: 80  # Battery level percentage (set dynamically in Python)
                max: 100
                color: (0, 1, 0, 1)  # Green color for full battery
                size_hint_x: 0.8
                pos_hint: {"center_x": 0.5}

'''


class PathPlannerWidget(Widget):
    ARENA_SIZE_METERS = 4  # 4m x 4m arena

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.points = []  # Stores path points
        self.path_group = InstructionGroup()  # Stores drawn path separately
        self.drone = Image(source="plutodronespinner.gif", size=(40, 40))  # Load drone image
        self.add_widget(self.drone)  # Add drone image to widget
        self.bind(size=self.on_size)  # Ensure it resizes correctly
        Window.bind(mouse_pos=self.on_mouse_move)  # Bind mouse movement for label update

        # Hover label for (X, Y, Z)
        self.hover_label = MDLabel(
            text="",
            size_hint=(None, None),
            size=(100, 30),
            color=(1, 1, 1, 1),  # White text
            bold=True
        )
        self.add_widget(self.hover_label)
        self.hover_label.opacity = 0  # Initially hidden

        Clock.schedule_interval(self.update_drone_position, 0.5)

    def update_drone_position(self, dt):
        """Update the drone's position using UWB data."""
        app = MDApp.get_running_app()

        uwb_x, uwb_y, uwb_z = app.drone_position # Simulated UWB values
        widget_x, widget_y = self.pos
        widget_width, widget_height = self.size

        # Convert UWB coordinates to screen space
        drone_x = widget_x + (uwb_x / self.ARENA_SIZE_METERS) * widget_width
        drone_y = widget_y + (1 - float(uwb_y) / self.ARENA_SIZE_METERS) * widget_height


        self.drone.pos = (drone_x - 20, drone_y - 20)
        print(f"Updated Drone Position (UWB): X={uwb_x}, Y={uwb_y}, Z={uwb_z}")


    def scale_to_arena(self, pos):
        """Convert screen pixels to real-world 4m x 4m coordinates"""
        widget_x, widget_y = self.pos  # Get widget position in the window
        widget_width, widget_height = self.size  # Get widget dimensions

        # Ensure scaling is relative to the Path Planner Widget, not the whole window
        x_meters = ((pos[0] - widget_x) / widget_width) * self.ARENA_SIZE_METERS
        y_meters = ((pos[1] - widget_y) / widget_height) * self.ARENA_SIZE_METERS

        return x_meters, y_meters
    
    def on_mouse_move(self, window, pos):
        """Update label position dynamically and show (X, Y, Z)."""
        if self.collide_point(*pos):  # Ensure mouse is within the widget
            x_meters, y_meters = self.scale_to_arena(pos)

            # Get slider (Z) value
            z_value = MDApp.get_running_app().root.ids.path_slider.value

            # Update label text
            self.hover_label.text = f"X: {x_meters:.2f} m, Y: {y_meters:.2f} m, Z: {z_value:.2f} m"
            self.hover_label.pos = (pos[0] + 10, pos[1] - 10)  # Position near mouse
            self.hover_label.opacity = 1  # Show label
        else:
            self.hover_label.opacity = 0  # Hide when outside

    def on_size(self, *args):
        """Reposition drone when window resizes."""
        self.drone.pos = (self.center_x - 20, self.center_y - 20)

    def on_touch_down(self, touch):
        """Handles left-click for marking points."""
        if self.collide_point(*touch.pos):  # Check if touch is within the widget
            if touch.button == 'left':
                scaled_pos = self.scale_to_arena(touch.pos)  # Scale position
                print(f"Clicked at: {scaled_pos} meters")  # Debugging output
                self.add_point(scaled_pos)

    def add_point(self, pos):
        """Adds a new point to the path and redraws."""
        x,y = pos
        z_value = MDApp.get_running_app().root.ids.path_slider.value
        pos = x,y,z_value
        print(pos)
        self.points.append(pos)
        self.update_path()

    def update_path(self):
        """Redraws the path separately without clearing the drone image."""
        self.canvas.remove(self.path_group)  # Remove old path only
        self.path_group.clear()
        app = MDApp.get_running_app()

        self.path_group.add(Color(0, 0, 1, 1))  # Blue path

        if self.points:
            screen_points = []
            widget_x, widget_y = self.pos  # Get path planner widget position
            widget_width, widget_height = self.size  # Get widget dimensions

            for point in self.points:
                # Convert arena coordinates to GUI coordinates
                x_pixels = widget_x + (point[0] / self.ARENA_SIZE_METERS) * widget_width
                y_pixels = widget_y + (point[1] / self.ARENA_SIZE_METERS) * widget_height
                screen_points.extend([x_pixels, y_pixels])

            self.path_group.add(Line(points=screen_points, width=2))

        self.canvas.add(self.path_group)  # Re-add updated path


    def move_drone(self, start_pos, end_pos, duration):
        """Moves the drone smoothly from start to end position."""
        # Interpolate between start_pos and end_pos over a period of time (duration)
        steps = 30  # Number of steps for smooth movement
        step_size = 1.0 / steps

        def move_step(step):
            if step <= 1:
                x = start_pos[0] + (end_pos[0] - start_pos[0]) * step
                y = start_pos[1] + (end_pos[1] - start_pos[1]) * step
                self.drone.pos = (x - 20, y - 20)  # Move drone
                Clock.schedule_once(lambda dt: move_step(step + step_size), duration / steps)
            else:
                # Ensure drone is exactly at the end position when the animation finishes
                self.drone.pos = (end_pos[0] - 20, end_pos[1] - 20)

        move_step(0)  # Start movement

        
      
class DroneApp(MDApp):
    def build(self):
        self.is_paused = False 
        self.root = Builder.load_string(KV)
        self.bridge = CvBridge()
        self.cv_image = None  # Store the latest frame
        self.image_path = "/tmp/ros_camera_feed.jpg"  # Temporary image file
        self.drone_position = (0, 0, 0)


        # Store points and index as instance variables
        self.interpolatedPoints = [
        
            # add as many as you want
        ]
        self.current_index = 0

        # Initialize ROS node only once
        rospy.init_node('pluto_gui_node', anonymous=True)

        # Start the ROS service
        self.service = rospy.Service('set_position', SetPos, self.handle_get_next_point)

        self.create_default_image()
        self.pub = rospy.Publisher('/input_key', Int16, queue_size=1)
        rospy.Subscriber("/camera/image_raw", ROSImage, self.image_callback)
        Window.bind(on_key_down=self.on_key_down)
        Window.bind(on_key_up=self.on_key_up)

        self.subscriber_thread = threading.Thread(target=self.start_ros_subscriber, daemon=True)
        self.subscriber_thread.start()
        self.last_telemetry_time = time.time()
        Clock.schedule_interval(self.check_telemetry_status, 1)
        Clock.schedule_interval(self.update_kivy_image, 1.0 / 30.0)  
        

        return self.root
    
    def handle_get_next_point(self, req):
        # Always update the latest drone-reported position
        self.drone_position = (req.pos_x, req.pos_y, req.pos_z)

        # If all points have been sent, echo back current position
        if self.current_index >= len(self.interpolatedPoints):
          
            res = SetPosResponse()
            res.set_x, res.set_y, res.set_z = req.pos_x, req.pos_y, req.pos_z
            return res

        # Otherwise, send the next mission point
        x, y, z = self.interpolatedPoints[self.current_index]
        self.current_index += 1
        print(self.current_index )

        res = SetPosResponse()
        res.set_x = x
        res.set_y = y
        res.set_z = z
        return res

    def start_ros_subscriber(self):
        """ROS Subscriber to get drone position from UWB data."""
        rospy.Subscriber("drone_data", Drone_stats, self.telemetry)
        rospy.Subscriber("drone_data", Drone_stats, self.update_bms_status)
        
        rospy.spin()

    def on_key_down(self, instance, keyboard, keycode, text, modifiers):
        print(f"Received keycode: {keycode}")  # Debugging

        # Extract keycode if it's a tuple
        keycode = keycode[1] if isinstance(keycode, tuple) else keycode

        keyboard_control = {
            82: 10,  # up
            80: 30,  # left
            79: 40,  # right
            26: 50,  # w
            22: 60,  # s
            44: 70,  # space
            21: 80,  # r
            23: 90,  # t
            19: 100, # p
            12: 85,  # i
            18: 95,  # o
            81: 110, # down
            17: 120, # n
            20: 130, # q
            8:  140, # e
            4: 150,  # a
            7: 160,  # d
            46: 15,  # +
            30: 25,  # 1
            31: 30,  # 2
            32: 35,  # 3
            33: 45,  # 4
            13: 63,  # k
            14: 64,  # k
            16: 65,  # m
            17: 66,  # n
            15: 67,  # l
            19: 68   # p

        }

        msg_pub = keyboard_control.get(keycode, 80)  # Default to 80 if key is not in dictionary

        # Ensure self.pub is initialized
        if not hasattr(self, 'pub') or self.pub is None:
            return

        # Update GUI label safely
        if hasattr(self, 'label'):
            self.label.text = f"Key Pressed: {keycode} Command: {msg_pub}"

        # Publish message
        self.pub.publish(msg_pub)

    def on_key_up(self, window, key, scancode):
        print(f"Key Released: {key}")
        # Stop movement when the key is released


    def create_default_image(self):
        """Create a default black image to avoid a white screen."""
        default_img = cv2.imread("default.jpg")  # Try loading an existing image
        if default_img is None:
            default_img = 255 * np.ones((480, 640, 3), dtype=np.uint8)  # Create a white image
            cv2.imwrite("default.jpg", default_img)  # Save it


    def image_callback(self, msg):
        """Convert incoming ROS image message to OpenCV format."""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def update_kivy_image(self, dt):
        """Convert OpenCV image to Kivy texture and update UI."""
        if self.cv_image is not None:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

            # Flip the image vertically to correct orientation
            flipped_image = cv2.flip(rgb_image, 0)

            # Convert OpenCV image to Kivy texture
            texture = Texture.create(size=(flipped_image.shape[1], flipped_image.shape[0]), colorfmt='rgb')
            texture.blit_buffer(flipped_image.tobytes(), colorfmt='rgb', bufferfmt='ubyte')

            # Update the Kivy Image widget
            self.root.ids.camera_feed.texture = texture
            
    def snap_slider(self, slider):
        """Snap the slider to predefined values when released."""
        fixed_values = [30, 50, 70, 100, 130]
        closest_value = min(fixed_values, key=lambda x: abs(x - slider.value))
        Clock.schedule_once(lambda dt: setattr(slider, 'value', closest_value), 0.1)  # Slight delay to ensure UI update



    def reset_path(self):
        """Resets the path on the planner."""
        self.root.ids.path_planner.points = []
        self.root.ids.path_planner.update_path()

    def save_path(self):
        """Saves the current path to a file."""
        # Add your path saving logic here (e.g., save to a file)
        print("Path saved!")

    
    def mission_planner(self, mission):
        """Generates interpolated path points for smooth drone movement."""
        # Ensure mission is a NumPy array
        mission = np.array(mission)

        stepFactor = 0.01
        threshold_radius = 0.1
        self.interpolatedPoints = []

        for i in range(len(mission) - 1):
            startPoint = mission[i]
            endPoint = mission[i + 1]
            if(np.array_equal(startPoint, endPoint) == 0):
                distance = np.linalg.norm(endPoint - startPoint)
                stepSize = stepFactor * distance
                numSteps = int(np.ceil(distance / stepSize))

                for j in range(1, numSteps + 1):
                    t = (j - 1) / numSteps
                    point = (1 - t) * startPoint + t * endPoint
                    self.interpolatedPoints.append(point)

        self.interpolatedPoints.append(mission[-1])
        self.interpolatedPoints = np.array(self.interpolatedPoints)
        

    def execute_path(self):
        # self.current_index  = 0
        """Executes the drawn path for the drone to follow sequentially."""
        path_points = self.root.ids.path_planner.points
        path_points.insert(0,self.drone_position)
        # print(path_points)
        # print(self.drone_position)
        self.mission_planner(path_points)
        path_planner = self.root.ids.path_planner
        widget_x, widget_y = path_planner.pos
        widget_width, widget_height = path_planner.size

        if len(path_points) < 2:
            return  # Need at least two points to move

        # def move_next(index):
        #     if index < len(path_points) - 1:
        #         x1, y1, z1 = path_points[index]
        #         x2, y2, z2 = path_points[index + 1]

        #         new_x1 = widget_x + (x1 / 4) * widget_width
        #         new_y1 = widget_y + (y1 / 4) * widget_height
        #         new_x2 = widget_x + (x2 / 4) * widget_width
        #         new_y2 = widget_y + (y2 / 4) * widget_height

        #         path_planner.move_drone((new_x1, new_y1), (new_x2, new_y2), 2)

        #         # Schedule the next move after the current one finishes
        #         Clock.schedule_once(lambda dt: move_next(index + 1), 2)  # 2 seconds duration

        # move_next(0)
        self.current_index  = 0
        

    def pause_resume(self):

        self.is_paused = not self.is_paused
        if self.is_paused:
            print("Drone is paused")
        else:
            print("Drone is resumed")

    def dock(self):

        print("Guiding drone to docking station...")

    def emergency_land(self):
        print("Initiating emergency landing...")


    
    def telemetry(self, msg):
        self.last_telemetry_time = time.time()  # Update last telemetry time

        (x, y, z)  = self.drone_position 

        telemetry_text = (
            f"Roll: {msg.roll} | Pitch: {msg.pitch} | Yaw: {msg.yaw}\n"
            f"ACC_x: {msg.accX} | ACC_y: {msg.accY} | ACC_z: {msg.accZ}\n"
            f"GYRO_x: {msg.gyroX} | GYRO_y: {msg.gyroY} | GYRO_z: {msg.gyroZ}\n"
            f"Anchor_1: {msg.a1/536870912} | Anchor_2: {msg.a2/536870912} | Anchor_3: {msg.a3/536870912}\n"
            f"Drone Position: X={x}, Y={y}, Z={z}\n"
            f"RSSI : {msg.rssi}"
        )
        self.root.ids.telemetry_data.text = telemetry_text
        self.update_bms_status(msg)  # Update BMS status with telemetry data

    def check_telemetry_status(self, dt):
        """Check if telemetry data is delayed and update BMS status."""
        if time.time() - self.last_telemetry_time > 2:  # If no telemetry in last 2 seconds
            self.root.ids.bms_status.text = "System Status: Offline"
            self.root.ids.battery_bar.color = (1, 0, 0, 1)  # Red color for offline status

    def update_bms_status(self, msg):
        """Update Battery Monitoring System (BMS) status."""
        battery_level = msg.battery
        battery_bar = self.root.ids.battery_bar

        # Convert voltage to percentage if necessary
        if battery_level <= 4.2:  # Assuming voltage input (3.0V - 4.2V)
            battery_level = max(0, min(100, ((battery_level - 3.0) / (4.2 - 3.0)) * 100))

        # Set color based on battery percentage
        battery_bar.color = (0, 1, 0, 1) if battery_level > 60 else \
                            (1, 1, 0, 1) if battery_level > 30 else \
                            (1, 0, 0, 1)
        
        if battery_level <= 30:
            
            """Executes the drawn path for the drone to follow sequentially."""
            path_points = [(2,2,50)]
            path_points.insert(0,self.drone_position)
            print(path_points)
        # print(self.drone_position)
            self.mission_planner(path_points)
            path_planner = self.root.ids.path_planner


        battery_bar.value = battery_level
        battery_level = round(msg.battery, 2) 
        self.root.ids.bms_status.text = f"System Status: Battery {battery_level}"

    


if __name__ == "__main__":
    DroneApp().run()
