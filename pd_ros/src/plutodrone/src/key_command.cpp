#include <iostream>
#include <map>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>

std::map<std::string, int> keyboard_control =
{
    {"[A", 10}, {"[D", 30}, {"[C", 40}, {"w", 50}, {"s", 60},
    {" ", 70}, {"r", 80}, {"t", 90}, {"p", 100}, {"i", 85},
    {"o", 95}, {"[B", 110}, {"n", 120}, {"q", 130}, {"e", 140},
    {"a", 150}, {"d", 160}, {"+", 15}, {"1", 25}, {"2", 30},
    {"3", 35}, {"4", 45} 
}; 

int getKey()
{
    struct termios oldt,newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}


int main (int argc, char** argv)
{   
    ros::init(argc, argv, "simple_drone_teleop_key");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int16>("/input_key", 1);
    ros::Rate rate(100);
    
    int msg_pub = 0;
    
    std::string key;
    std_msgs::Int16 msg ;

    std::cout << "Drone Keyboard control initiated!\n";


    while(ros::ok())
    {
        char c = getKey();
        // printf("Pressed key is %c",c);
        if (c == 27) {
            char next1 = getKey();
            char next2 = getKey();
            key = "[";
            key += next2;
        } else {
            key = c;
        }

        if (key == "\x03") {
            break;
        }
    
        if (keyboard_control.find(key) != keyboard_control.end()) {
            msg_pub = keyboard_control[key];
        } else {
            msg_pub = 80;
        }
        
        msg.data = msg_pub;
        pub.publish(msg);
        
        rate.sleep();
    }

    return 0;
}






















///////////////////////////BACKUP//////////////////////////////////
// #include <iostream>
// #include <map>
// #include <termios.h>
// #include <unistd.h>

// std::map<std::string, int> keyboard_control =
// {
//     {"[A", 10}, {"[D", 30}, {"[C", 40}, {"w", 50}, {"s", 60},
//     {" ", 70}, {"r", 80}, {"t", 90}, {"p", 100}, {"i", 85},
//     {"o", 95}, {"[B", 110}, {"n", 120}, {"q", 130}, {"e", 140},
//     {"a", 150}, {"d", 160}, {"+", 15}, {"1", 25}, {"2", 30},
//     {"3", 35}, {"4", 45} 
// }; 

// int getKey()
// {
//     struct termios oldt,newt;
//     tcgetattr(STDIN_FILENO, &oldt);
//     newt = oldt;
//     newt.c_lflag &= ~(ICANON | ECHO);
//     tcsetattr(STDIN_FILENO, TCSANOW, &newt);

//     int ch = getchar();
//     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

//     return ch;
// }


// int main (int argc, char** argv)
// {   
//     std::string key;

//     while(true)
//     {
//         char c = getKey();
//         // printf("Pressed key is %c",c);
//         if (c == 27) {  // Escape sequence handling
//             char next1 = getKey();
//             char next2 = getKey();
//             key = "[";
//             key += next2;
//         } else {
//             key = c;
//         }

//         if (key == "\x03") {  // Handle CTRL+C
//             break;
//         }
    
//         if (keyboard_control.find(key) != keyboard_control.end()) {
//             printf("%d\n",keyboard_control[key]);
//         } else {
//             printf("%d\n",80);
//         }
        
//     }

//     return 0;

// }