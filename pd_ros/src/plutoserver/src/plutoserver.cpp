// #include "ros/ros.h"
// #include "plutodrone/PlutoPilot.h"

// bool myFunction(plutodrone::PlutoPilot::Request  &req,
//          plutodrone::PlutoPilot::Response &res)
// {
//   ROS_INFO("Ax=%f, Ay=%f, Az=%f", req.accX, req.accY, req.accZ);
//   // ROS_INFO("Gx=%f, Gy=%f, Gz=%f", req.gyroX, req.gyroY, req.gyroZ);
//   // ROS_INFO("Mx=%f, My=%f, Mz=%f", req.magX, req.magY, req.magZ);


//   // ROS_INFO("roll=%i, pitch=%i, yaw=%i", req.roll, req.pitch, req.yaw);
//   // ROS_INFO("altitiude=%f", req.alt);
//   // ROS_INFO("battery=%f rssi=%i", req.battery,req.rssi);

//   ROS_INFO("Anchor_1=%f Anchor_2=%f Anchor_3=%f", (req.a1/536870912),(req.a2/536870912),(req.a3/536870912));
//   // ROS_INFO("Anchor_1=%f Anchor_3=%f Anchor_2=%f", (req.a1),(req.a2),(req.a3));

//   res.rcAUX1=1800;

//   // ROS_INFO("sending back response: [%ld]", (long int)res.rcAUX1);
//   return true;
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "PlutoData");
//   ros::NodeHandle n;

//   ros::ServiceServer service = n.advertiseService("PlutoService", myFunction);
//   printf("Ready to Provide Pluto Service\n");
//   ros::spin();

//   return 0;
// }


#include "ros/ros.h"
#include "plutodrone/Drone_stats.h"  // Assuming you have this message type

void dataCallback(const plutodrone::Drone_stats::ConstPtr& msg) {
  // Print the received data
  ROS_INFO("Received data: ");
  ROS_INFO("AccX: %f, AccY: %f, AccZ: %f", msg->accX, msg->accY, msg->accZ);
  ROS_INFO("GyroX: %f, GyroY: %f, GyroZ: %f", msg->gyroX, msg->gyroY, msg->gyroZ);
  ROS_INFO("MagX: %f, MagY: %f, MagZ: %f", msg->magX, msg->magY, msg->magZ);
  ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", msg->roll, msg->pitch, msg->yaw);
  ROS_INFO("Altitude: %f, Battery: %f, RSSI: %f", msg->alt, msg->battery, msg->rssi);
  ROS_INFO("Anchor 1: %f, Anchor 2: %f, Anchor 3: %f", msg->a1, msg->a2, msg->a3);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pluto_server");
  ros::NodeHandle n;

  // Create a subscriber that listens to the "drone_data" topic
  ros::Subscriber dataSubscriber = n.subscribe("drone_data", 1000, dataCallback);

  // Spin to keep the node running and processing callbacks
  ros::spin();

  return 0;
}
