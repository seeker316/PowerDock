#include <ros/ros.h>
#include <plutodrone/PlutoMsg.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>


class DroneControl
{
    private:        
        bool control;
        ros::NodeHandle nh;
        ros::Publisher command_pub;
        ros::Subscriber key_sub;
        ros::Subscriber pid_sub;
        plutodrone::PlutoMsg cmd;

    public:
        DroneControl()
        {
            control= false;
            command_pub = nh.advertise<plutodrone::PlutoMsg>("/drone_command", 1);
            key_sub = nh.subscribe("/input_key", 1, &DroneControl::identifyKey, this);
            pid_sub = nh.subscribe("/pid_values", 1, &DroneControl::arucoCallback, this);
            
            cmd.rcRoll = 1500;
            cmd.rcPitch = 1500;
            cmd.rcYaw = 1500;
            cmd.rcThrottle = 1500;
            cmd.rcAUX1 = 1500;
            cmd.rcAUX2 = 1500;
            cmd.rcAUX3 = 1500;
            cmd.rcAUX4 = 1000;
            cmd.commandType = 0;
            cmd.trim_roll = 0;
            cmd.trim_pitch = 0;
            cmd.isAutoPilotOn = 0;
        }

        void arm()
        {
            cmd.rcRoll = 1500;
            cmd.rcYaw = 1500;
            cmd.rcPitch = 1500;
            cmd.rcThrottle = 1000;
            cmd.rcAUX4 = 1500;
            cmd.isAutoPilotOn = 0;
            command_pub.publish(cmd);
            ros::Duration(1).sleep();
        }

        void box_arm()
        {
            cmd.rcRoll = 1500;
            cmd.rcYaw = 1500;
            cmd.rcPitch = 1500;
            cmd.rcThrottle = 1000;
            cmd.rcAUX4 = 1500;
            cmd.isAutoPilotOn = 0;
            command_pub.publish(cmd);
            ros::Duration(0.5).sleep();
        }

        void disarm()
        {
            cmd.rcThrottle = 1300;
            cmd.rcAUX4 = 1200;
            command_pub.publish(cmd);
            ros::Duration(0.5).sleep();
        }

        void arucoCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
            if (control) {
                cmd.rcRoll = msg->data[0];
                cmd.rcPitch = msg->data[1];
                // cmd.rcYaw = msg->data[2];
                cmd.rcThrottle = msg->data[3];
                command_pub.publish(cmd);
            }
        }

        void moveForward() { cmd.rcPitch = 1600; command_pub.publish(cmd); }
        void moveBackward() { cmd.rcPitch = 1400; command_pub.publish(cmd); }
        void moveLeft() { cmd.rcRoll = 1400; command_pub.publish(cmd); }
        void moveRight() { cmd.rcRoll = 1600; command_pub.publish(cmd); }
        void yawLeft() { cmd.rcYaw = 1200; command_pub.publish(cmd); }
        void yawRight() { cmd.rcYaw = 1800; command_pub.publish(cmd); }
        void increaseHeight() { cmd.rcThrottle = 1800; command_pub.publish(cmd); }
        void decreaseHeight() { cmd.rcThrottle = 1300; command_pub.publish(cmd); }
        void takeOff() { disarm(); box_arm(); cmd.commandType = 1; command_pub.publish(cmd); }
        void land() { cmd.commandType = 2; command_pub.publish(cmd); }

        void identifyKey(const std_msgs::Int16::ConstPtr& msg) {
            int key = msg->data;
            switch (key) {
                case 70: (cmd.rcAUX4 == 1500) ? disarm() : arm(); break;
                case 10: moveForward(); break;
                case 30: moveLeft(); break;
                case 40: moveRight(); break;
                // case 80: resetCommand(); command_pub.publish(cmd); break;
                case 90: cmd.isAutoPilotOn = !cmd.isAutoPilotOn; break;
                case 50: increaseHeight(); break;
                case 60: decreaseHeight(); break;
                case 110: moveBackward(); break;
                case 130: takeOff(); break;
                case 140: land(); break;
                case 150: yawLeft(); break;
                case 160: yawRight(); break;
                case 85: control = true; ROS_INFO("Control on"); break;
                case 95: control = false; ROS_INFO("Control off");disarm() ; break;
            }
            command_pub.publish(cmd);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_controller");
    DroneControl controller;
    ros::spin();
    return 0;
}