#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
#include "plutodrone/Drone_stats.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Dense>
#include <vector>
#include <numeric>

using namespace Eigen;
using namespace std;
using namespace cv;

// Camera parameters for RealSense D455F
const double fx = 390.23;
const double fy = 390.23;
const double cx = 317.58273274;
const double cy = 231.94288763;
const double tag_size = 0.05;  // 5 cm

plutodrone::Drone_stats current_drone_data;
std::mutex data_mutex; 

int pos_step = 50;
int alt_hold = 500;

int frame_centre_x = 320,frame_centre_y = 240;
////////////////////////////////////////////////////////////////

double xA1 = 0, yA1 = 1, zA1 = 0;
double xA2 = 0, yA2 = 0, zA2 = 0;
double xA3 = 1, yA3 = 0, zA3 = 0;

double s1_true = 0.87;
double s2_true = 0.64;
double s3_true = 0.65;

double s1_filtered,s2_filtered, s3_filtered;

class kalman_filter
{
private:
float Q, R, P, K = 0, x = 0;
public:
    kalman_filter(float Q_set,float R_set,float P_set)
    {   
        Q = Q_set;
        R = R_set;
        P = P_set;
    
    };
    
    float kalman_filter_calc(float measurement)
    {
        P = P + Q;
        K = P / (P + R);
        x = x + K * (measurement - x);
        P = (1 - K) * P;
        return x;
    };
};



void dataCallback(const plutodrone::Drone_stats::ConstPtr& msg) {
    
    std::lock_guard<std::mutex> lock(data_mutex);
    // Store the received data in the global variable
    current_drone_data = *msg;
    // Print the received data
    // ROS_INFO("Received data: ");
    // ROS_INFO("AccX: %f, AccY: %f, AccZ: %f", msg->accX, msg->accY, msg->accZ);
    // ROS_INFO("GyroX: %f, GyroY: %f, GyroZ: %f", msg->gyroX, msg->gyroY, msg->gyroZ);
    // ROS_INFO("MagX: %f, MagY: %f, MagZ: %f", msg->magX, msg->magY, msg->magZ);
    // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", msg->roll, msg->pitch, msg->yaw);
    // ROS_INFO("Altitude: %f, Battery: %f, RSSI: %f", msg->alt, msg->battery, msg->rssi);
    // ROS_INFO("Anchor 1: %f, Anchor 2: %f, Anchor 3: %f", msg->a1, msg->a2, msg->a3);
  }
  

class PIDController {
public:

    float Kp, Ki, Kd, output_min, output_max;

    float prev_error = 0, derivative = 0, output = 0;
    float dt = 0.1;

    PIDController(float Kp, float Ki, float Kd, float output_min, float output_max)
        : Kp(Kp), Ki(Ki), Kd(Kd), output_min(output_min), output_max(output_max) {}

    float PID_Output(float error, bool operation) {

        derivative = error - prev_error;

        if (operation) {
            output = ((Kp * error) + (Kd * derivative)) + 1500;
        }
        else {
            output = -((Kp * error) + (Kd * derivative)) + 1500;
        }

        prev_error = error;

        // Apply Ceiling
        if (output < output_min) { output = output_min; }
        else if (output > output_max) { output = output_max; }

        return output;
    }

};

void change_setpoint(const std_msgs::Int16::ConstPtr& msg) {
    int key = msg->data;
    int new_hold;
    switch (key) {
        case 63: new_hold = frame_centre_x + pos_step; frame_centre_x = new_hold; break;
        case 64: new_hold = frame_centre_x - pos_step; frame_centre_x = new_hold; break;
    }
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "apriltag_pid_publisher");
    ros::NodeHandle nh;
    ros::Publisher pid_pub = nh.advertise<std_msgs::Int16MultiArray>("/pid_values", 10);
    ros::Subscriber dataSubscriber = nh.subscribe("drone_data", 1000, dataCallback);
    ros::Subscriber set_point = nh.subscribe("/input_key", 1, change_setpoint);
    ros::Rate rate(100);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 10);
        
    // PIDController roll_pid(0.40625, 0.0,6.4, 1300, 1700);
    PIDController roll_pid(0.25, 0.0,6.35, 1300, 1700);

    PIDController pitch_pid(0.25, 0.0, 6.35, 1300, 1700);

    // PIDController yaw_pid(0.0, 0.0, 0.0, 1200, 1800);
    PIDController throttle_pid(1.425, 0.0, 0.5, 1300, 1800);

    // PIDController roll_pid(0.0, 0.0, 0.0, 1300, 1700);
    // PIDController pitch_pid(0.0, 0.0, 0.0, 1300, 1700);
    PIDController yaw_pid(0.0, 0.0, 0.0, 1200, 1800);
    // PIDController throttle_pid(0.0, 0.0, 0.0, 1300, 1800);

    kalman_filter z_kal(0.03,0.1,1);
    kalman_filter x_kal(0.09,0.5,1);
    kalman_filter y_kal(0.09,0.5,1);


    float error_x = 0, error_y = 0, error_yaw = 0, alt_error = 0, estimated_distance = 0;

    float roll_pid_output = 0, pitch_pid_output = 0, yaw_pid_output = 0, throttle_pid_output = 0;


    float depth_value,tof_data; //because realsense provides data in 16 bits
    
    bool detected = false;

    float last_known_distance = 0, last_yaw_error = 0;
    std::vector<cv::Point2f> prev_points, next_points;
    std::vector<int> last_known_position;

    std::vector<uchar> status;
    std::vector<float> err;

    apriltag_family_t* tf = tag16h5_create();
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.05;
    td->decode_sharpening = 0.25;
    td->refine_edges = 0;
    cv::VideoCapture cap(6);
    if (!cap.isOpened()) {
        ROS_ERROR("Error: Could not open the camera.");
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    // // cap.set(CAP_PROP_FPS, 30);
    // // cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
    // // cap.set(CAP_PROP_EXPOSURE, -4);

    // rs2::pipeline pipeline;
    // rs2::config config;
    // config.enable_stream(RS2_STREAM_COLOR, 640,480, RS2_FORMAT_BGR8, 60);
    // config.enable_stream(RS2_STREAM_DEPTH,  640,480, RS2_FORMAT_Z16, 60);
    // pipeline.start(config);


    cv::Mat frame, gray, prev_gray;

    while (ros::ok())
    {
        
        tof_data = z_kal.kalman_filter_calc(current_drone_data.alt);

        // kf1.update((current_drone_data.a1/1000) - kf1.s_offset);
        // kf2.update((current_drone_data.a2/1000) - kf2.s_offset);
        // kf3.update((current_drone_data.a3/1000) - kf3.s_offset);
        
        // s1_filtered = kf1.getFilteredValue();
        // s2_filtered = kf2.getFilteredValue();
        // s3_filtered = kf3.getFilteredValue();

        // trilateration(xA1, yA1, zA1, xA2, yA2, zA2, xA3, yA3, zA3, s1_filtered, s2_filtered, s3_filtered);
        
        detected = false;
        // rs2::frameset frames = pipeline.wait_for_frames();
        // rs2::frame color_frame = frames.get_color_frame();
        // rs2::frame depth_frame = frames.get_depth_frame();
        
        // cv::Mat frame(cv::Size( 640,480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        // cv::Mat depth = cv::Mat(cv::Size( 640,480), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);  // Convert depth frame to Mat
        


        cap.read(frame);

        if (frame.empty()) break;

        cv_bridge::CvImagePtr cv_ptr;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t img_header = {gray.cols, gray.rows, gray.cols, gray.data};
        zarray_t* detections = apriltag_detector_detect(td, &img_header);
        


        for (int i = 0; i < zarray_size(detections); i++) 
        {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            if (det->id == 0 && det->decision_margin > 50) 
            {
                detected = true;

                apriltag_detection_info_t info = {det, tag_size, fx, fy, cx, cy};
                apriltag_pose_t pose;
                estimate_tag_pose(&info, &pose);

                std::lock_guard<std::mutex> lock(data_mutex);
                // estimated_distance = (pose.t->data[2]*1000) + 30;
                estimated_distance = tof_data;
                
                

                int tag_center_x = det->c[0];
                int tag_center_y = det->c[1];

                // error_x = tag_center_x - (frame.cols / 2);
                // error_y = tag_center_y - (frame.rows / 2);

                error_x = tag_center_x - frame_centre_x;
                error_y = tag_center_y - frame_centre_y;

                error_yaw = atan2(pose.R->data[3], pose.R->data[0]) * 180.0 / CV_PI;

                last_known_position = {tag_center_x, tag_center_y};
                last_known_distance = estimated_distance;
                last_yaw_error = error_yaw;

                for (int j = 0; j < 4; j++) 
                {
                    line(frame, Point(det->p[j][0], det->p[j][1]), Point(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]), Scalar(0, 255, 0), 2);
                }
                circle(frame, Point(tag_center_x, tag_center_y), 5, Scalar(0, 0, 255), -1);
                break;
            }
            else
            {
                estimated_distance = tof_data;
                // estimated_distance = last_known_distance;
            }
        }
        

        if (!detected && !last_known_position.empty()) {
            if (!prev_gray.empty() && !prev_points.empty()) {
                calcOpticalFlowPyrLK(prev_gray, gray, prev_points, next_points, status, err);
                if (!next_points.empty() && status[0] == 1) {
                    last_known_position = {static_cast<int>(next_points[0].x), static_cast<int>(next_points[0].y)};
                    error_x = last_known_position[0] - (frame.cols / 2);
                    error_y = last_known_position[1] - (frame.rows / 2);
                    error_yaw = last_yaw_error;

                    circle(frame, Point(last_known_position[0], last_known_position[1]), 10, Scalar(255, 0, 0), -1);
                    putText(frame, "Optical Flow Tracking", Point(last_known_position[0] + 10, last_known_position[1]), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
                }
            }
        }


        if (estimated_distance < 2000)
        {
            alt_error = alt_hold - estimated_distance;
            error_x = x_kal.kalman_filter_calc((error_x/fx)*estimated_distance);
            error_y = y_kal.kalman_filter_calc((error_y/fy)*estimated_distance);

            roll_pid_output = roll_pid.PID_Output(error_x, false);
            pitch_pid_output = pitch_pid.PID_Output(error_y, false);
            yaw_pid_output = yaw_pid.PID_Output(error_yaw, true);
            throttle_pid_output = throttle_pid.PID_Output(alt_error, true);
        }

        std_msgs::Int16MultiArray msg;
        msg.data.clear();
        msg.data.push_back(static_cast<int>(roll_pid_output));
        msg.data.push_back(static_cast<int>(pitch_pid_output));
        msg.data.push_back(static_cast<int>(yaw_pid_output));
        msg.data.push_back(static_cast<int>(throttle_pid_output));
        pid_pub.publish(msg);


        // ROS_INFO("X_Hold = %d ,Published: Dist = %f, Roll=%d, Pitch=%d, Yaw=%d, Throttle=%d",static_cast<int>(frame_centre_x),
        //         static_cast<float>(estimated_distance),static_cast<int>(roll_pid_output), static_cast<int>(pitch_pid_output),
        //         static_cast<int>(yaw_pid_output), static_cast<int>(throttle_pid_output));

        prev_gray = gray.clone();
        
        if (!last_known_position.empty()) {
            prev_points = {cv::Point2f(last_known_position[0], last_known_position[1])};
        }
        putText(frame, "Battery: " + to_string(current_drone_data.battery), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        
        putText(frame, "X Error: " + to_string(int(error_x)), Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Y Error: " + to_string(int(error_y)), Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Yaw: " + to_string(int(error_yaw)), Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Distance: " + to_string(estimated_distance), Point(10, 150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);

        int text_x = frame.cols - 250;
        putText(frame, "Roll PID: " + to_string(int(roll_pid_output)), Point(text_x, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Pitch PID: " + to_string(int(pitch_pid_output)), Point(text_x, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Yaw PID: " + to_string(int(yaw_pid_output)), Point(text_x, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(frame, "Throttle PID: " + to_string(int(throttle_pid_output)), Point(text_x, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);


        cv::imshow("April Tracking with PID", frame);

        cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
        cv_ptr->header.stamp = ros::Time::now(); // Time stamp
        cv_ptr->header.frame_id = "camera";      // Frame ID
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // Encoding format
        cv_ptr->image = frame; // The captured frame

        // Publish the image
        image_pub.publish(cv_ptr->toImageMsg());
        // ROS_INFO("publishing");

        if (cv::waitKey(1) == 'q') break;

        apriltag_detections_destroy(detections);
        rate.sleep();
        ros::spinOnce();
    }

    // pipeline.stop();
    cap.release();
    cv::destroyAllWindows();

    

        return 0;
}





// ????????????????????????????????????????????????????????????????????????????????????????????????

// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <cmath>

// #include <Eigen/Dense>

// #include <local_tools.h>

// #include <ros/ros.h>
// #include <std_msgs/Int16MultiArray.h>
// #include <std_msgs/Int16.h>

// #include <librealsense2/rs.hpp>

// #include <apriltag/apriltag.h>
// #include <apriltag/apriltag_pose.h>
// #include <apriltag/tag16h5.h>
// #include "plutodrone/Drone_stats.h"

// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/Image.h>

// using namespace std;
// using namespace cv;

// // Camera parameters for RealSense D455F
// const double fx = 390.23;
// const double fy = 390.23;
// const double cx = 317.58273274;
// const double cy = 231.94288763;
// const double tag_size = 0.05;  // 5 cm

// plutodrone::Drone_stats current_drone_data;
// std::mutex data_mutex; 

// int pos_step = 50;
// int alt_hold = 500;

// int frame_centre_x = 320,frame_centre_y = 240;

// ///////////////////////////////////////////////////////////////////////////

// void trilateration(double x1, double y1, double z1, 
//     double x2, double y2, double z2, 
//     double x3, double y3, double z3, 
//     double s1, double s2, double s3) {
// // Create the matrix A and vector B
// Eigen::MatrixXd A(3, 3);
// Eigen::VectorXd B(3);

// // Subtract equations (1) and (2), (2) and (3), etc., to linearize the system
// A(0, 0) = 2 * (x2 - x1);
// A(0, 1) = 2 * (y2 - y1);
// A(0, 2) = 2 * (z2 - z1);
// B(0) = s1 * s1 - s2 * s2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2 - z1 * z1 + z2 * z2;

// A(1, 0) = 2 * (x3 - x2);
// A(1, 1) = 2 * (y3 - y2);
// A(1, 2) = 2 * (z3 - z2);
// B(1) = s2 * s2 - s3 * s3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3 - z2 * z2 + z3 * z3;

// A(2, 0) = 2 * (x1 - x3);
// A(2, 1) = 2 * (y1 - y3);
// A(2, 2) = 2 * (z1 - z3);
// B(2) = s3 * s3 - s1 * s1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1 - z3 * z3 + z1 * z1;

// // Solve the linear system
// Eigen::VectorXd result = A.colPivHouseholderQr().solve(B);

// // Output the result (x, y, z coordinates of the point)
// std::cout << "The coordinates of the point are: " << std::endl;
// std::cout << "x: " << result(0) << std::endl;
// std::cout << "y: " << result(1) << std::endl;
// std::cout << "z: " << result(2) << std::endl;
// }

// void Kalman_init(double dt, Eigen::VectorXd &x, Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::MatrixXd &H,
//   Eigen::MatrixXd &P, Eigen::MatrixXd &Q, double &R) {

// // Define the base state transition matrix A
// A = Eigen::MatrixXd(3, 3);
// A << 1, dt, 0.5 * dt * dt,
// 0, 1, dt,
// 0, 0, 1;

// // Define the control input matrix B
// B = Eigen::VectorXd(3);
// B << 0.5 * dt * dt, dt, 1;

// // Define the measurement model matrix H
// H = Eigen::MatrixXd(1, 3);
// H << 1, 0, 0;

// // Define the initial state covariance matrix P
// P = Eigen::MatrixXd(3, 3);
// P.setZero();
// P.diagonal() << 1, 1, 1;  // Initial covariance on the diagonal

// // Define the process noise covariance matrix Q
// Q = Eigen::MatrixXd(3, 3);
// Q.setZero();
// Q.diagonal() << 0, 8.4154e-05, 7.0818e-09;

// // Define the measurement noise covariance R
// R = 0.0015;

// // Now expand A, B, H, P, Q to their Kronecker products (3x3 block diagonal)
// Eigen::MatrixXd A_expanded = Eigen::MatrixXd(9, 9);
// Eigen::MatrixXd B_expanded = Eigen::MatrixXd(9, 1);
// Eigen::MatrixXd H_expanded = Eigen::MatrixXd(3, 9);
// Eigen::MatrixXd P_expanded = Eigen::MatrixXd(9, 9);
// Eigen::MatrixXd Q_expanded = Eigen::MatrixXd(9, 9);

// // Kronecker product: A, B, H, P, Q for 3x3 blocks
// A_expanded.setZero();
// B_expanded.setZero();
// H_expanded.setZero();
// P_expanded.setZero();
// Q_expanded.setZero();

// // A = kron(eye(3), A)
// A_expanded.block(0, 0, 3, 3) = A;
// A_expanded.block(3, 3, 3, 3) = A;
// A_expanded.block(6, 6, 3, 3) = A;

// // B = kron(eye(3), B)
// B_expanded.block(0, 0, 3, 1) = B;
// B_expanded.block(3, 0, 3, 1) = B;
// B_expanded.block(6, 0, 3, 1) = B;

// // H = kron(eye(3), H)
// H_expanded.block(0, 0, 1, 3) = H;
// H_expanded.block(1, 3, 1, 3) = H;
// H_expanded.block(2, 6, 1, 3) = H;

// // P = kron(eye(3), P)
// P_expanded.block(0, 0, 3, 3) = P;
// P_expanded.block(3, 3, 3, 3) = P;
// P_expanded.block(6, 6, 3, 3) = P;

// // Q = kron(eye(3), Q)
// Q_expanded.block(0, 0, 3, 3) = Q;
// Q_expanded.block(3, 3, 3, 3) = Q;
// Q_expanded.block(6, 6, 3, 3) = Q;

// // Initialize state vector x
// x = Eigen::VectorXd(9);
// x << 3.5046, 0, 0,  // x, vx, ax (for first object)
// 3.5046, 0, 0,  // y, vy, ay (for second object)
// 3.5045, 0, 0;  // z, vz, az (for third object)

// // Print the matrices and state vector to verify
// std::cout << "State transition matrix A:" << std::endl << A_expanded << std::endl;
// std::cout << "Control input matrix B:" << std::endl << B_expanded << std::endl;
// std::cout << "Measurement model matrix H:" << std::endl << H_expanded << std::endl;
// std::cout << "State covariance matrix P:" << std::endl << P_expanded << std::endl;
// std::cout << "Process noise covariance matrix Q:" << std::endl << Q_expanded << std::endl;
// std::cout << "Measurement noise covariance R:" << std::endl << R << std::endl;
// std::cout << "Initial state vector x:" << std::endl << x << std::endl;
// }





// //////////////////////////////////////////////////////////////////////
// class kalman_filter
// {
// private:
// float Q, R, P, K = 0, x = 0;
// public:
//     kalman_filter(float Q_set,float R_set,float P_set)
//     {   
//         Q = Q_set;
//         R = R_set;
//         P = P_set;
    
//     };
    
//     float kalman_filter_calc(float measurement)
//     {
//         P = P + Q;
//         K = P / (P + R);
//         x = x + K * (measurement - x);
//         P = (1 - K) * P;
//         return x;
//     };
// };



// void dataCallback(const plutodrone::Drone_stats::ConstPtr& msg) {
    
//     std::lock_guard<std::mutex> lock(data_mutex);
//     // Store the received data in the global variable
//     current_drone_data = *msg;
//     // Print the received data
//     // ROS_INFO("Received data: ");
//     // ROS_INFO("AccX: %f, AccY: %f, AccZ: %f", msg->accX, msg->accY, msg->accZ);
//     // ROS_INFO("GyroX: %f, GyroY: %f, GyroZ: %f", msg->gyroX, msg->gyroY, msg->gyroZ);
//     // ROS_INFO("MagX: %f, MagY: %f, MagZ: %f", msg->magX, msg->magY, msg->magZ);
//     // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", msg->roll, msg->pitch, msg->yaw);
//     // ROS_INFO("Altitude: %f, Battery: %f, RSSI: %f", msg->alt, msg->battery, msg->rssi);
//     // ROS_INFO("Anchor 1: %f, Anchor 2: %f, Anchor 3: %f", msg->a1, msg->a2, msg->a3);
//   }
  

// class PIDController {
// public:

//     float Kp, Ki, Kd, output_min, output_max;

//     float prev_error = 0, derivative = 0, output = 0;
//     float dt = 0.1;

//     PIDController(float Kp, float Ki, float Kd, float output_min, float output_max)
//         : Kp(Kp), Ki(Ki), Kd(Kd), output_min(output_min), output_max(output_max) {}

//     float PID_Output(float error, bool operation) {

//         derivative = error - prev_error;

//         if (operation) {
//             output = ((Kp * error) + (Kd * derivative)) + 1500;
//         }
//         else {
//             output = -((Kp * error) + (Kd * derivative)) + 1500;
//         }

//         prev_error = error;

//         // Apply Ceiling
//         if (output < output_min) { output = output_min; }
//         else if (output > output_max) { output = output_max; }

//         return output;
//     }

// };

// void change_setpoint(const std_msgs::Int16::ConstPtr& msg) {
//     int key = msg->data;
//     int new_hold;
//     switch (key) {
//         case 63: new_hold = frame_centre_x + pos_step; frame_centre_x = new_hold; break;
//         case 64: new_hold = frame_centre_x - pos_step; frame_centre_x = new_hold; break;
//     }
    
// }

// void trilateration(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, double s1, double s2, double s3) {
//     // Create the matrix A and vector B
//     Eigen::MatrixXd A(3, 3);
//     Eigen::VectorXd B(3);


//     // Subtract equations (1) and (2), (2) and (3), etc., to linearize the system
//     A(0, 0) = 2 * (x2 - x1);
//     A(0, 1) = 2 * (y2 - y1);
//     A(0, 2) = 2 * (z2 - z1);
//     B(0) = s1 * s1 - s2 * s2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2 - z1 * z1 + z2 * z2;

//     A(1, 0) = 2 * (x3 - x2);
//     A(1, 1) = 2 * (y3 - y2);
//     A(1, 2) = 2 * (z3 - z2);
//     B(1) = s2 * s2 - s3 * s3 - x2 * x2 + x3 * x3 - y2 * y2 + y3 * y3 - z2 * z2 + z3 * z3;

//     A(2, 0) = 2 * (x1 - x3);
//     A(2, 1) = 2 * (y1 - y3);
//     A(2, 2) = 2 * (z1 - z3);
//     B(2) = s3 * s3 - s1 * s1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1 - z3 * z3 + z1 * z1;

//     // Solve the linear system
//     Eigen::VectorXd result = A.colPivHouseholderQr().solve(B);

//     // Output the result (x, y, z coordinates of the point)
//     std::cout << "The coordinates of the point are: " << std::endl;
//     std::cout << "x: " << result(0) << std::endl;
//     std::cout << "y: " << result(1) << std::endl;
//     std::cout << "z: " << result(2) << std::endl;
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "apriltag_pid_publisher");
//     ros::NodeHandle nh;
//     ros::Publisher pid_pub = nh.advertise<std_msgs::Int16MultiArray>("/pid_values", 10);
//     ros::Subscriber dataSubscriber = nh.subscribe("drone_data", 1000, dataCallback);
//     ros::Subscriber set_point = nh.subscribe("/input_key", 1, change_setpoint);
//     ros::Rate rate(100);

//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 10);
        
//     // PIDController roll_pid(0.40625, 0.0,6.4, 1300, 1700);
//     PIDController roll_pid(0.25, 0.0,6.35, 1300, 1700);

//     PIDController pitch_pid(0.25, 0.0, 6.35, 1300, 1700);

//     // PIDController yaw_pid(0.0, 0.0, 0.0, 1200, 1800);
//     PIDController throttle_pid(1.425, 0.0, 0.5, 1300, 1800);

//     // PIDController roll_pid(0.0, 0.0, 0.0, 1300, 1700);
//     // PIDController pitch_pid(0.0, 0.0, 0.0, 1300, 1700);
//     PIDController yaw_pid(0.0, 0.0, 0.0, 1200, 1800);
//     // PIDController throttle_pid(0.0, 0.0, 0.0, 1300, 1800);

//     kalman_filter z_kal(0.03,0.1,1);
//     kalman_filter x_kal(0.09,0.5,1);
//     kalman_filter y_kal(0.09,0.5,1);

//     float error_x = 0, error_y = 0, error_yaw = 0, alt_error = 0, estimated_distance = 0;

//     float roll_pid_output = 0, pitch_pid_output = 0, yaw_pid_output = 0, throttle_pid_output = 0;


//     float depth_value,tof_data; //because realsense provides data in 16 bits
    
//     bool detected = false;

//     float last_known_distance = 0, last_yaw_error = 0;
//     std::vector<cv::Point2f> prev_points, next_points;
//     std::vector<int> last_known_position;

//     std::vector<uchar> status;
//     std::vector<float> err;

//     apriltag_family_t* tf = tag16h5_create();
//     apriltag_detector_t* td = apriltag_detector_create();
//     apriltag_detector_add_family(td, tf);
//     td->quad_decimate = 2.0;
//     td->quad_sigma = 0.05;
//     td->decode_sharpening = 0.25;
//     td->refine_edges = 0;
//     cv::VideoCapture cap(6);
//     if (!cap.isOpened()) {
//         ROS_ERROR("Error: Could not open the camera.");
//         return -1;
//     }

//     cap.set(CAP_PROP_FRAME_WIDTH, 640);
//     cap.set(CAP_PROP_FRAME_HEIGHT, 480);
//     // // cap.set(CAP_PROP_FPS, 30);
//     // // cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
//     // // cap.set(CAP_PROP_EXPOSURE, -4);

//     // rs2::pipeline pipeline;
//     // rs2::config config;
//     // config.enable_stream(RS2_STREAM_COLOR, 640,480, RS2_FORMAT_BGR8, 60);
//     // config.enable_stream(RS2_STREAM_DEPTH,  640,480, RS2_FORMAT_Z16, 60);
//     // pipeline.start(config);


//     cv::Mat frame, gray, prev_gray;

//     ////////////////////////////////////
//     double dt = 0.05;  // Time step (dt)
//     Eigen::VectorXd x;
//     Eigen::MatrixXd A, B, H, P, Q;
//     double R;

//     /////////////////////////////////////
//     while (ros::ok())
//     {
//         detected = false;
//         // rs2::frameset frames = pipeline.wait_for_frames();
//         // rs2::frame color_frame = frames.get_color_frame();
//         // rs2::frame depth_frame = frames.get_depth_frame();
        
//         // cv::Mat frame(cv::Size( 640,480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
//         // cv::Mat depth = cv::Mat(cv::Size( 640,480), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);  // Convert depth frame to Mat
        
//         cap.read(frame);

//         cv::Mat gray;
//         if (frame.empty()) break;

//         cv_bridge::CvImagePtr cv_ptr;

//         cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
//         image_u8_t img_header = {gray.cols, gray.rows, gray.cols, gray.data};
//         zarray_t* detections = apriltag_detector_detect(td, &img_header);
        
        


//         tof_data = z_kal.kalman_filter_calc(current_drone_data.alt);

//         Kalman_init(dt, x, A, B, H, P, Q, R);

//         for (int i = 0; i < zarray_size(detections); i++) 
//         {
//             apriltag_detection_t* det;
//             zarray_get(detections, i, &det);

//             if (det->id == 0 && det->decision_margin > 50) 
//             {
//                 detected = true;

//                 apriltag_detection_info_t info = {det, tag_size, fx, fy, cx, cy};
//                 apriltag_pose_t pose;
//                 estimate_tag_pose(&info, &pose);

//                 std::lock_guard<std::mutex> lock(data_mutex);
//                 // estimated_distance = (pose.t->data[2]*1000) + 30;
//                 estimated_distance = tof_data;
                
                

//                 int tag_center_x = det->c[0];
//                 int tag_center_y = det->c[1];

//                 // error_x = tag_center_x - (frame.cols / 2);
//                 // error_y = tag_center_y - (frame.rows / 2);

//                 error_x = tag_center_x - frame_centre_x;
//                 error_y = tag_center_y - frame_centre_y;

//                 error_yaw = atan2(pose.R->data[3], pose.R->data[0]) * 180.0 / CV_PI;

//                 last_known_position = {tag_center_x, tag_center_y};
//                 last_known_distance = estimated_distance;
//                 last_yaw_error = error_yaw;

//                 for (int j = 0; j < 4; j++) 
//                 {
//                     line(frame, Point(det->p[j][0], det->p[j][1]), Point(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]), Scalar(0, 255, 0), 2);
//                 }
//                 circle(frame, Point(tag_center_x, tag_center_y), 5, Scalar(0, 0, 255), -1);
//                 break;
//             }
//             else
//             {
//                 estimated_distance = tof_data;
//                 // estimated_distance = last_known_distance;
//             }
//         }
        

//         if (!detected && !last_known_position.empty()) {
//             if (!prev_gray.empty() && !prev_points.empty()) {
//                 calcOpticalFlowPyrLK(prev_gray, gray, prev_points, next_points, status, err);
//                 if (!next_points.empty() && status[0] == 1) {
//                     last_known_position = {static_cast<int>(next_points[0].x), static_cast<int>(next_points[0].y)};
//                     error_x = last_known_position[0] - (frame.cols / 2);
//                     error_y = last_known_position[1] - (frame.rows / 2);
//                     error_yaw = last_yaw_error;

//                     circle(frame, Point(last_known_position[0], last_known_position[1]), 10, Scalar(255, 0, 0), -1);
//                     putText(frame, "Optical Flow Tracking", Point(last_known_position[0] + 10, last_known_position[1]), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
//                 }
//             }
//         }


//         if (estimated_distance < 2000)
//         {
//             alt_error = alt_hold - estimated_distance;
//             error_x = x_kal.kalman_filter_calc((error_x/fx)*estimated_distance);
//             error_y = y_kal.kalman_filter_calc((error_y/fy)*estimated_distance);

//             roll_pid_output = roll_pid.PID_Output(error_x, false);
//             pitch_pid_output = pitch_pid.PID_Output(error_y, false);
//             yaw_pid_output = yaw_pid.PID_Output(error_yaw, true);
//             throttle_pid_output = throttle_pid.PID_Output(alt_error, true);
//         }

//         std_msgs::Int16MultiArray msg;
//         msg.data.clear();
//         msg.data.push_back(static_cast<int>(roll_pid_output));
//         msg.data.push_back(static_cast<int>(pitch_pid_output));
//         msg.data.push_back(static_cast<int>(yaw_pid_output));
//         msg.data.push_back(static_cast<int>(throttle_pid_output));
//         pid_pub.publish(msg);


//         ROS_INFO("X_Hold = %d ,Published: Dist = %f, Roll=%d, Pitch=%d, Yaw=%d, Throttle=%d",static_cast<int>(frame_centre_x),
//                 static_cast<float>(estimated_distance),static_cast<int>(roll_pid_output), static_cast<int>(pitch_pid_output),
//                 static_cast<int>(yaw_pid_output), static_cast<int>(throttle_pid_output));

//         prev_gray = gray.clone();
        
//         if (!last_known_position.empty()) {
//             prev_points = {cv::Point2f(last_known_position[0], last_known_position[1])};
//         }
//         putText(frame, "Battery: " + to_string(current_drone_data.battery), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        
//         putText(frame, "X Error: " + to_string(int(error_x)), Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
//         putText(frame, "Y Error: " + to_string(int(error_y)), Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
//         putText(frame, "Yaw: " + to_string(int(error_yaw)), Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
//         putText(frame, "Distance: " + to_string(estimated_distance), Point(10, 150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);

//         int text_x = frame.cols - 250;
//         putText(frame, "Roll PID: " + to_string(int(roll_pid_output)), Point(text_x, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
//         putText(frame, "Pitch PID: " + to_string(int(pitch_pid_output)), Point(text_x, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
//         putText(frame, "Yaw PID: " + to_string(int(yaw_pid_output)), Point(text_x, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
//         putText(frame, "Throttle PID: " + to_string(int(throttle_pid_output)), Point(text_x, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);


//         // cv::imshow("April Tracking with PID", frame);

//         cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
//         cv_ptr->header.stamp = ros::Time::now(); // Time stamp
//         cv_ptr->header.frame_id = "camera";      // Frame ID
//         cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // Encoding format
//         cv_ptr->image = frame; // The captured frame

//         // Publish the image
//         image_pub.publish(cv_ptr->toImageMsg());
//         ROS_INFO("publishing");

//         if (cv::waitKey(1) == 'q') break;

//         apriltag_detections_destroy(detections);
//         rate.sleep();
//         ros::spinOnce();
//     }

//     // pipeline.stop();
//     cap.release();
//     cv::destroyAllWindows();

    

//         return 0;
// }