// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "plutodrone/PlutoPilot.h"
// #include <geometry_msgs/PoseArray.h>
// #include <sys/time.h>
// #include <boost/thread.hpp>
// #include <boost/chrono.hpp>
// #include <pthread.h>
// #include <unistd.h>
// #include <plutodrone/Common.h>
// #include <plutodrone/Protocol.h>
// #include <plutodrone/PlutoMsg.h>
// #include <plutodrone/PlutoMsgAP.h>


// #define TRIM_MAX 1000
// #define TRIM_MIN -1000

// using namespace std;


// bool isSocketCreated;

// int isAutoPilotOn = 0;

// Communication com;
// Protocol pro;

// ros::ServiceClient serviceClient;
// plutodrone::PlutoPilot service;

// int userRC[8]={1500,1500,1500,1500,1000,1000,1000,1000};

// int userRCAP[4]={1500,1500,1500,1500};

// int droneRC[8] = {1500,1500,1500,1500,1000,1000,1000,1000};

// int commandType = 0, commandTypeAP = 0;

// void *createSocket(void *threadid){
//   isSocketCreated=com.connectSock();
//   pthread_exit(NULL);
// }

// void *writeFunction(void *threadid){
  
//   std::vector<int> requests;
//   requests.push_back(MSP_RC);
//   requests.push_back(MSP_ATTITUDE);
//   requests.push_back(MSP_RAW_IMU);
//   requests.push_back(MSP_ALTITUDE);
//   requests.push_back(MSP_ANALOG);
//   requests.push_back(MSP_UWB);

//   // pro.sendRequestMSP_ACC_TRIM();

//   while(1)
//   {
//     memcpy(droneRC, userRC, sizeof(userRC));

//     if(isAutoPilotOn && droneRC[7] == 1500) {
//       droneRC[0] += userRCAP[0] - 1500;
//       droneRC[1] += userRCAP[1] - 1500;
//       droneRC[2] += userRCAP[2] - 1500;
//       droneRC[3] += userRCAP[3] - 1500;
//     }

//     pro.sendRequestMSP_SET_RAW_RC(droneRC);
//     pro.sendRequestMSP_GET_DEBUG(requests);

//     if(commandType != NONE_COMMNAD) {
//         pro.sendRequestMSP_SET_COMMAND(commandType);
//         commandType = NONE_COMMNAD;
//     }
//     else if(commandTypeAP != NONE_COMMNAD && isAutoPilotOn && droneRC[7] == 1500) {
//        pro.sendRequestMSP_SET_COMMAND(commandTypeAP);
//        commandTypeAP = NONE_COMMNAD;
//     }

//     usleep(22000);
//   }
//   pthread_exit(NULL);
// }

// void *readFunction(void *threadid){
//   do
//   {
//     com.readFrame();
//   }
//   while(1);
//   pthread_exit(NULL);
// }

// void *serviceFunction(void *threadid){
//   while (1)
//   {
//       if (serviceClient.call(service))
//       {
//        service.request.accX=accX;
//        service.request.accY=accY;
//        service.request.accZ=accZ;
//        service.request.gyroX=gyroX;
//        service.request.gyroY=gyroY;
//        service.request.gyroZ=gyroZ;
//        service.request.magX=magX;
//        service.request.magY=magY;
//        service.request.magZ=magZ;
//        service.request.roll=roll;
//        service.request.pitch=pitch;
//        service.request.yaw=yaw;
//        service.request.alt=alt;
//        service.request.battery=battery;
//        service.request.rssi=rssi;
//        service.request.a1=a1;
//        service.request.a2=a2;
//        service.request.a3=a3;

//        ROS_INFO("Publishing data...");
//        ROS_INFO("AccX: %f, AccY: %f, AccZ: %f", accX, accY, accZ);

//   }
//   }
//  pthread_exit(NULL);
// }

// void readDroneCommand(const plutodrone::PlutoMsg::ConstPtr& msg){
//   userRC[0] = msg->rcRoll;
//   userRC[1] = msg->rcPitch;
//   userRC[2] = msg->rcThrottle;
//   userRC[3] = msg->rcYaw;
//   userRC[4] = msg->rcAUX1;
//   userRC[5] = msg->rcAUX2;
//   userRC[6] = msg->rcAUX3;
//   userRC[7] = msg->rcAUX4;

//   isAutoPilotOn = msg->isAutoPilotOn;
//   if(commandType == NONE_COMMNAD)
//     commandType = msg->commandType;

//   if(msg->trim_roll != 0 || msg->trim_pitch != 0) {
//     trim_roll += msg->trim_roll;
//     trim_pitch += msg->trim_pitch;
//     if(trim_roll > TRIM_MAX)
//       trim_roll = TRIM_MAX;
//     else if(trim_roll < TRIM_MIN)
//       trim_roll = TRIM_MIN;
//     if(trim_pitch > TRIM_MAX)
//         trim_pitch = TRIM_MAX;
//     else if(trim_pitch < TRIM_MIN)
//         trim_pitch = TRIM_MIN;
//     pro.sendRequestMSP_SET_ACC_TRIM(trim_roll, trim_pitch);
//     pro.sendRequestMSP_EEPROM_WRITE();
//   }

// }

// void readDroneAPCommand(const plutodrone::PlutoMsgAP::ConstPtr& msg){


//   cout << "Received Drone AP Command\n";
//   cout << "Yaw: "<<msg->rcYaw<<endl;

//   userRCAP[0] = msg->rcRoll;
//   userRCAP[1] = msg->rcPitch;
//   userRCAP[2] = msg->rcThrottle;
//   userRCAP[3] = msg->rcYaw;
//   if(commandTypeAP == NONE_COMMNAD)
//     commandTypeAP = msg->commandType;
// }


// int main(int argc, char **argv){
//     pthread_t thread, readThread, writeThread, serviceThread;
//     int rc;
//     ros::init(argc, argv, "plutonode");
//     ros::NodeHandle n;
//     ros::Subscriber sub = n.subscribe("drone_command", 1000, readDroneCommand);
//     ros::Subscriber subAutoPilot = n.subscribe("drone_ap_command", 1000, readDroneAPCommand);
    
//     rc = pthread_create(&thread, NULL, createSocket, 	(void *)1);
//     if (rc)
//     {
//      cout << "Error:unable to create communication thread," << rc << endl;
//      exit(-1);
//     }
//     pthread_join( thread, NULL);

//     if(isSocketCreated)
//     {
//       rc = pthread_create(&writeThread, NULL, writeFunction, 	(void *)2);
//       if (rc)
//       {
//         cout << "Error:unable to create write thread," << rc << endl;
//         exit(-1);
//       }
//       rc = pthread_create(&readThread, NULL, readFunction, 	(void *)3);
//       if (rc)
//       {
//         cout << "Error:unable to read create thread," << rc << endl;
//         exit(-1);
//       }

//       serviceClient = n.serviceClient<plutodrone::PlutoPilot>("PlutoService",true);
//       rc = pthread_create(&serviceThread, NULL, serviceFunction, 	(void *)4);

//       if (rc)
//       {
//         cout << "Error:unable to service create thread," << rc << endl;
//         exit(-1);
//       }
//     }

//     ros::spin();
//     return 0;
// }


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "plutodrone/PlutoPilot.h"
#include <geometry_msgs/PoseArray.h>
#include <sys/time.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <pthread.h>
#include <unistd.h>
#include <plutodrone/Common.h>
#include <plutodrone/Protocol.h>
#include <plutodrone/PlutoMsg.h>
#include <plutodrone/PlutoMsgAP.h>
#include <plutodrone/Drone_stats.h>

#include <ros/time.h>

ros::Time start_time;  // Variable to store the timestamp of the first a1 value
bool a1_received = false;

#define TRIM_MAX 1000
#define TRIM_MIN -1000

using namespace std;


bool isSocketCreated;

int isAutoPilotOn = 0;

Communication com;
Protocol pro;

ros::ServiceClient serviceClient;
plutodrone::PlutoPilot service;

int userRC[8]={1500,1500,1500,1500,1000,1000,1000,1000};

int userRCAP[4]={1500,1500,1500,1500};

int droneRC[8] = {1500,1500,1500,1500,1000,1000,1000,1000};

int commandType = 0, commandTypeAP = 0;

void *createSocket(void *threadid){
  isSocketCreated=com.connectSock();
  pthread_exit(NULL);
}

void *writeFunction(void *threadid){
  
  std::vector<int> requests;
  requests.push_back(MSP_RC);
  requests.push_back(MSP_ATTITUDE);
  requests.push_back(MSP_RAW_IMU);
  requests.push_back(MSP_ALTITUDE);
  requests.push_back(MSP_ANALOG);
  requests.push_back(MSP_UWB);

  // pro.sendRequestMSP_ACC_TRIM();

  while(1)
  {
    memcpy(droneRC, userRC, sizeof(userRC));

    if(isAutoPilotOn && droneRC[7] == 1500) {
      droneRC[0] += userRCAP[0] - 1500;
      droneRC[1] += userRCAP[1] - 1500;
      droneRC[2] += userRCAP[2] - 1500;
      droneRC[3] += userRCAP[3] - 1500;
    }

    pro.sendRequestMSP_SET_RAW_RC(droneRC);
    pro.sendRequestMSP_GET_DEBUG(requests);

    if(commandType != NONE_COMMNAD) {
        pro.sendRequestMSP_SET_COMMAND(commandType);
        commandType = NONE_COMMNAD;
    }
    else if(commandTypeAP != NONE_COMMNAD && isAutoPilotOn && droneRC[7] == 1500) {
       pro.sendRequestMSP_SET_COMMAND(commandTypeAP);
       commandTypeAP = NONE_COMMNAD;
    }

    usleep(22000);
  }
  pthread_exit(NULL);
}

void *readFunction(void *threadid){
  do
  {
    com.readFrame();
  }
  while(1);
  pthread_exit(NULL);
}

void *serviceFunction(void* args)
{
  ros::Publisher* drone_stat_pub = (ros::Publisher*)args;
  plutodrone::Drone_stats new_msg; // Create a new message to publish
  ros::Rate loop_rate(100);
  
  while (ros::ok()) 
  {

    // Set the values to the new message
    new_msg.accX = accX;
    new_msg.accY = accY;
    new_msg.accZ = accZ;
    new_msg.gyroX = gyroX;
    new_msg.gyroY = gyroY;
    new_msg.gyroZ = gyroZ;
    new_msg.magX = magX;
    new_msg.magY = magY;
    new_msg.magZ = magZ;
    new_msg.roll = roll;
    new_msg.pitch = pitch;
    new_msg.yaw = yaw;
    new_msg.alt = alt;
    new_msg.battery = battery;
    new_msg.rssi = rssi;
    // new_msg.a1 = ((a1/536870912) < 5 && (a1/536870912) >= 0) ? (a1/536870912) : new_msg.a1;
    // new_msg.a2 = ((a2/536870912) < 5 && (a2/536870912) >= 0) ? (a2/536870912) : new_msg.a2;
    // new_msg.a3 = ((a3/536870912) < 5 && (a3/536870912) >= 0) ? (a3/536870912) : new_msg.a3;

    // if (!a1_received && (a1) < 5 && (a1) >= 0) 
    
    if (!a1_received) 
    {
      start_time = ros::Time::now();  // Capture the current time
      a1_received = true;  // Mark that the first a1 value is received
    }

    if (a1_received && (a1) != new_msg.a1) 
    {
      ros::Time end_time = ros::Time::now();  // Capture the current time
      ros::Duration time_diff = end_time - start_time;  // Calculate the time difference
      ROS_INFO("Time difference for a1 update: %.2f seconds", time_diff.toSec());
      
      a1_received = false;  // Reset the flag after the value is updated
    }

     // Update the new_msg a1, a2, a3 with condition
    new_msg.a1 = (a1);
    // ROS_INFO("Time difference",new_msg.a1);
    new_msg.a2 = (a2);
    new_msg.a3 = (a3);
    // new_msg.a1 = ((a1) < 5 && (a1) >= 0) ? (a1) : new_msg.a1;
    // new_msg.a2 = ((a2) < 5 && (a2) >= 0) ? (a2) : new_msg.a2;
    // new_msg.a3 = ((a3) < 5 && (a3) >= 0) ? (a3) : new_msg.a3;
    
    


    // Publish the new message
    drone_stat_pub->publish(new_msg);
    ros::spinOnce();  // Call callbacks (if any)
    loop_rate.sleep();  // Sleep to maintain the desired rate
  }
  pthread_exit(NULL);
}

void readDroneCommand(const plutodrone::PlutoMsg::ConstPtr& msg){
  userRC[0] = msg->rcRoll;
  userRC[1] = msg->rcPitch;
  userRC[2] = msg->rcThrottle;
  userRC[3] = msg->rcYaw;
  userRC[4] = msg->rcAUX1;
  userRC[5] = msg->rcAUX2;
  userRC[6] = msg->rcAUX3;
  userRC[7] = msg->rcAUX4;

  isAutoPilotOn = msg->isAutoPilotOn;
  if(commandType == NONE_COMMNAD)
    commandType = msg->commandType;

  if(msg->trim_roll != 0 || msg->trim_pitch != 0) {
    trim_roll += msg->trim_roll;
    trim_pitch += msg->trim_pitch;
    if(trim_roll > TRIM_MAX)
      trim_roll = TRIM_MAX;
    else if(trim_roll < TRIM_MIN)
      trim_roll = TRIM_MIN;
    if(trim_pitch > TRIM_MAX)
        trim_pitch = TRIM_MAX;
    else if(trim_pitch < TRIM_MIN)
        trim_pitch = TRIM_MIN;
    pro.sendRequestMSP_SET_ACC_TRIM(trim_roll, trim_pitch);
    pro.sendRequestMSP_EEPROM_WRITE();
  }

}

void readDroneAPCommand(const plutodrone::PlutoMsgAP::ConstPtr& msg){


  cout << "Received Drone AP Command\n";
  cout << "Yaw: "<<msg->rcYaw<<endl;

  userRCAP[0] = msg->rcRoll;
  userRCAP[1] = msg->rcPitch;
  userRCAP[2] = msg->rcThrottle;
  userRCAP[3] = msg->rcYaw;
  if(commandTypeAP == NONE_COMMNAD)
    commandTypeAP = msg->commandType;
}


int main(int argc, char **argv){
    pthread_t thread, readThread, writeThread, serviceThread;
    int rc;
    ros::init(argc, argv, "plutonode");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("drone_command", 1000, readDroneCommand);
    ros::Subscriber subAutoPilot = n.subscribe("drone_ap_command", 1000, readDroneAPCommand);
    ros::Publisher drone_stat_pub = n.advertise<plutodrone::Drone_stats>("drone_data", 1000);

    rc = pthread_create(&thread, NULL, createSocket, 	(void *)1);
    if (rc)
    {
     cout << "Error:unable to create communication thread," << rc << endl;
     exit(-1);
    }
    pthread_join( thread, NULL);

    if(isSocketCreated)
    {
      rc = pthread_create(&writeThread, NULL, writeFunction, 	(void *)2);
      if (rc)
      {
        cout << "Error:unable to create write thread," << rc << endl;
        exit(-1);
      }
      rc = pthread_create(&readThread, NULL, readFunction, 	(void *)3);
      if (rc)
      {
        cout << "Error:unable to read create thread," << rc << endl;
        exit(-1);
      }

      // serviceClient = n.serviceClient<plutodrone::PlutoPilot>("PlutoService",true);


      // rc = pthread_create(&serviceThread, NULL, serviceFunction, 	(void *)4);
      rc = pthread_create(&serviceThread, NULL, serviceFunction, (void*)&drone_stat_pub);

      if (rc)
      {
        cout << "Error:unable to service create thread," << rc << endl;
        exit(-1);
      }
    }

    ros::spin();
    return 0;
}

