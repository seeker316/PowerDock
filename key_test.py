# from kivy.app import App
# from kivy.uix.label import Label
# from kivy.core.window import Window

# class KeypressApp(App):
#     def build(self):
#         self.label = Label(text="Press any key...")
#         Window.bind(on_key_down=self.on_key_down)
#         return self.label

#     def on_key_down(self, instance, keyboard, keycode, text, modifiers):
#         if text:
#             self.label.text = f"Key Pressed: {text}"
#         else:
#             self.label.text = f"Key Pressed (Special Key): {keycode}"

# if __name__ == "__main__":
#     KeypressApp().run()

###########################################################3

from kivy.app import App
from kivy.uix.label import Label
from kivy.core.window import Window
import rospy
from std_msgs.msg import Int16

class DroneTeleopApp(App):
    def build(self):
        self.label = Label(text="Press a key to control the drone")
        Window.bind(on_key_down=self.on_key_down)
        rospy.init_node('simple_drone_teleop_key', anonymous=True)
        self.pub = rospy.Publisher('/input_key', Int16, queue_size=1)
        return self.label

    def on_key_down(self, instance, keyboard, keycode, text, modifiers):
        keyboard_control = {
            82: 10, #up
            80: 30, #left
            79: 40, #right
            26: 50, #w
            22: 60, #s
            44: 70, #space
            21: 80, #r
            23: 90, #t
            19: 100,#p
            12: 85, #i
            18: 95, #o
            81: 110,#down
            17: 120,#n
            20: 130,#q
            8:  140,#e
            4: 150, #a
            7: 160, #d
            46: 15, #+
            30: 25, #1
            31: 30, #2
            32: 35, #3
            33: 45  #4
        }
        
        if keycode in keyboard_control:
            msg_pub = keyboard_control[keycode]
            self.label.text = f"Key Pressed: {keycode} Command: {msg_pub}"
            self.pub.publish(msg_pub)
        else:
            msg_pub = 80  # Default hover command
            self.label.text = f"Key Pressed: {keycode} Command: {msg_pub}"
            self.pub.publish(msg_pub)

if __name__ == "__main__":
    DroneTeleopApp().run()
