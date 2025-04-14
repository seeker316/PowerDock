#include "ros/ros.h"
#include "plutodrone/SetPos.h"

bool handle_set_position(plutodrone::SetPos::Request &req,
                         plutodrone::SetPos::Response &res)
{
    ROS_INFO("Received position request: x=%.2f, y=%.2f, z=%.2f", req.pos_x, req.pos_y, req.pos_z);

    // Echo back the values as confirmation (you could modify them if needed)
    res.set_x = req.pos_x;
    res.set_y = req.pos_y;
    res.set_z = req.pos_z;

    ROS_INFO("Responding with: set_x=%.2f, set_y=%.2f, set_z=%.2f", res.set_x, res.set_y, res.set_z);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pdserver");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("set_position", handle_set_position);
    ROS_INFO("SetPos service is ready to receive requests.");

    ros::spin();
    return 0;
}
